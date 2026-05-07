#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <chrono>
#include <sstream>
#include <iomanip>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <eigen_conversions/eigen_msg.h>
// #include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include "open3d_registration/open3d_registration.h"
#include "open3d_conversions/open3d_conversions.h"

/// @brief 单张地图的所有运行期数据
// XmlRpc 值安全转 double（兼容 int 和 double 类型）
static double XmlRpcToDouble(const XmlRpc::XmlRpcValue &v)
{
    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        return static_cast<double>(v);
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
        return static_cast<double>(static_cast<int>(v));
    return 0.0;
}

static double NormalizeAngleRad(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

static double MatrixYawRad(const Eigen::Matrix4d &mat)
{
    return std::atan2(mat(1, 0), mat(0, 0));
}

static double PoseDistanceXY(const Eigen::Matrix4d &a, const Eigen::Matrix4d &b)
{
    Eigen::Vector2d da = a.block<2, 1>(0, 3) - b.block<2, 1>(0, 3);
    return da.norm();
}

static double PoseYawDiffDeg(const Eigen::Matrix4d &a, const Eigen::Matrix4d &b)
{
    return std::fabs(NormalizeAngleRad(MatrixYawRad(a) - MatrixYawRad(b))) * 180.0 / M_PI;
}

static bool IsAllZeros(const std::vector<double> &v)
{
    for (auto x : v)
        if (std::abs(x) > 1e-6) return false;
    return true;
}

static std::string MatToXYYaw(const Eigen::Matrix4d &m)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(3)
       << "(" << m(0, 3) << "," << m(1, 3) << ",yaw=" << (MatrixYawRad(m) * 180.0 / M_PI) << "deg)";
    return os.str();
}

struct MapEntry
{
    std::string name;
    std::string path;
    std::string frame_id;
    std::shared_ptr<open3d::geometry::PointCloud> map_coarse;
    std::shared_ptr<open3d::geometry::PointCloud> map_fine;
    Eigen::Vector3d bbox_min{0, 0, 0};
    Eigen::Vector3d bbox_max{0, 0, 0};
    // 切换触发区域 [xmin, ymin, xmax, ymax]，当前地图坐标系下，只判 XY
    // 默认全范围，表示未设置（退回 AABB 行为）
    Eigen::Vector4d switch_zone{-1e9, -1e9, 1e9, 1e9};
    bool switch_zone_set = false;
    // 该地图自身的初始位姿 [x, y, z, roll, pitch, yaw]（度）
    std::vector<double> initialpose{0, 0, 0, 0, 0, 0};
    bool initialpose_set = false;
    // 切换到下一张地图时使用的初值 [x, y, z, roll, pitch, yaw]（度）
    std::vector<double> next_initialpose{0, 0, 0, 0, 0, 0};
    bool next_initialpose_set = false;
    // next_initialpose 是否已标定（非默认全零，或显式声明有效）
    bool next_initialpose_calibrated = false;
};

#define PI 3.1415926

class KalmanFilter
{
public:
    KalmanFilter()
    {
    }

    void KalmanFilterInit(double processVar, double estimatedMeasVar, double posteriEstimate = 0.0, double posteriErrorEstimate = 1.0)
    {
        processVar_ = processVar;
        estimatedMeasVar_ = estimatedMeasVar;
        posteriEstimate_ = posteriEstimate;
        posteriErrorEstimate_ = posteriErrorEstimate;
    }
    void inputLatestNoisyMeasurement(double measurement)
    {
        double prioriEstimate = posteriEstimate_;
        double prioriErrorEstimate = posteriErrorEstimate_ + processVar_;

        double blendingFactor = prioriErrorEstimate / (prioriErrorEstimate + estimatedMeasVar_);
        posteriEstimate_ = prioriEstimate + blendingFactor * (measurement - prioriEstimate);
        posteriErrorEstimate_ = (1 - blendingFactor) * prioriErrorEstimate;
    }

    double getLatestEstimatedMeasurement()
    {
        return posteriEstimate_;
    }

private:
    double processVar_;
    double estimatedMeasVar_;
    double posteriEstimate_;
    double posteriErrorEstimate_;
};

class GloabalLocalization
{
private:
    /* data */
public:
    GloabalLocalization(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~GloabalLocalization();

    /// @brief 初始化定位
    void LocalizationInitialize();

    /// @brief 订阅fast_lio里程计信息
    void CallbackBaselink2Odom(const nav_msgs::Odometry::ConstPtr &baselink2odom);
    /// @brief 订阅在baselink下的点云
    void CallbackScan(const sensor_msgs::PointCloud2::ConstPtr &scan_in_baselink);

    /// @brief 订阅在初始位姿
    void CallbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initialpose);

    /// @brief 启动地图监视线程
    void StartLoc();

    void Localization();

    /// @brief 后台线程,在重合区域对候选地图静默做 ICP,成功就触发切图
    void MapSwitchMonitor();

    /// @brief 切图(原子)
    /// @param new_idx maps_ 中的下标
    /// @param reason 用于状态日志
    void SwitchActiveMap(int new_idx, const std::string &reason,
                         const Eigen::Matrix4d *verified_odom2map = nullptr);

    /// @brief 单张地图加载 + 降采样 + 算 AABB
    bool LoadMap(const std::string &name, const std::string &path, MapEntry &out);

    /// @brief 当前位姿是否落在某张地图(AABB - shrink)内
    bool PoseInsideMap(const Eigen::Vector3d &xyz, const MapEntry &m) const;

    /// @brief 当前位姿是否在当前地图的切换触发区域内
    bool PoseInSwitchZone(const Eigen::Vector3d &xyz, const MapEntry &m) const;

    /// @brief 欧拉角转mat3x3
    /// @param euler
    /// @return
    Eigen::Matrix3d Euler2Matrix3d(const Eigen::Vector3d euler);

    /// @brief 获取tf关系到矩阵
    /// @param frame_id
    /// @param child_frame_id
    /// @param matrix
    /// @return
    bool GetTfTransformToMatrix(
        std::string frame_id, std::string child_frame_id, Eigen::Matrix4d &matrix);

    /// @brief compute 3d distance between two points
    /// @param a
    /// @param b
    /// @return 距离值
    double ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    /// @brief 订阅baselink2odom,即fast_lio的里程计信息
    ros::Subscriber sub_baselink2odom_;

    /// @brief 订阅当前帧点云
    ros::Subscriber sub_scan_cur_;

    /// @brief 订阅初始位姿
    ros::Subscriber sub_initialpose_;

    /// @brief baselink到odom的pose表达
    nav_msgs::Odometry pose_baselink2odom_;

    /// @brief bselink到odom的变换矩阵表达
    Eigen::Matrix4d mat_baselink2odom_;
    /// @brief odom到map的矩阵
    Eigen::Matrix4d mat_odom2map_;
    Eigen::Matrix4d mat_odom2map_kalman_;
    /// @brief baselink到map = mat_odom2map * mat_baselink2odom
    Eigen::Matrix4d mat_baselink2map_;
    /// @brief initialpose初始位姿
    Eigen::Matrix4d mat_initialpose_;

    std::mutex lock_state_;
    std::string active_map_frame_;
    std::atomic<bool> manual_reinit_pending_{false};

    /// @brief map epoch: incremented on each switch, Localization() discards stale ICP results
    std::atomic<uint64_t> map_epoch_{0};
    /// @brief T_world_active_map for RViz display continuity (方案 B)
    Eigen::Matrix4d mat_world_to_active_map_;
    /// @brief cached submap epoch and map_idx binding
    uint64_t cached_submap_epoch_ = 0;
    int cached_submap_map_idx_ = -1;

    /// @brief baselink和运动中心
    Eigen::Matrix4d mat_baselink2motionlink_;

    /// @brief imulink到baselink
    Eigen::Matrix4d mat_imulink2baselink_;

    /// @brief 初始位姿, x, y, z, roll, pitch, yaw (单位:度degrees)
    std::vector<float> initialpose_;

    /// @brief 原始地图点云
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_ori_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_coarse_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_fine_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_cur_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_cur_;

    /// @brief 多地图列表与当前激活地图索引(同坐标系,可静默切换)
    std::vector<MapEntry> maps_;
    std::atomic<int> active_map_idx_{0};
    std::atomic<int> candidate_map_idx_{-1};
    /// 主循环读、切图写。读多写极少
    mutable std::shared_mutex lock_maps_;
    /// @brief 切图后,主循环下一帧强制重切 submap、用 multiscale ICP 跑一帧
    std::atomic<bool> force_resubmap_{false};

    /// @brief 多地图相关参数
    bool map_switch_enable_ = false;
    double bbox_shrink_ = 1.0;
    double verify_fitness_ = 0.6;
    int verify_consecutive_ = 3;
    int verify_period_ms_ = 500;
    int cooldown_ms_ = 5000;
    double verify_icp_threshold_ = 0.0;
    double verify_max_translation_ = 1.0;
    double verify_max_yaw_deg_ = 20.0;
    double post_switch_min_fitness_ = 0.3;
    bool allow_uncalibrated_switch_ = false;
    /// @brief 后台监视线程
    std::thread thread_monitor_;

    std::queue<open3d::geometry::PointCloud> que_pcd_scan_;
    int queue_maxsize_;
    double voxelsize_coarse_;
    double voxelsize_fine_;

    /// @brief 定位配准fitness(overlap)阈值
    double threshold_fitness_;
    /// @brief 配准fitness(overlap)阈值
    double threshold_fitness_init_;

    std::thread thread_loc_;
    std::mutex lock_scan_;
    std::mutex lock_exit_;
    bool flag_exit_;

    ros::Publisher pub_baselink2map_;
    ros::Publisher pub_baselink2map_kalman_;
    ros::Publisher pub_motionlink2map_;
    ros::Publisher pub_odom2map_;
    ros::Publisher pub_odom2map_kalman_;
    ros::Time timestamp_odom_;
    std::mutex lock_timestamp_;

    ros::Publisher pub_map_;
    ros::Publisher pub_scan_;
    ros::Publisher pub_scan2map_;
    ros::Publisher pub_submap_;
    ros::Publisher pub_localization_3d_;
    ros::Publisher pub_localization_3d_confidence_;
    ros::Publisher pub_localization_3d_delay_ms_;
    ros::Publisher pub_active_map_;
    ros::Publisher pub_status_;

    geometry_msgs::PoseStamped localization_3d_;
    std_msgs::Float32 localization_3d_confidence_;
    std_msgs::Float32 localization_3d_delay_ms_;

    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    bool save_scan_;

    /// @brief 定位频率(定位间隔时间，多少秒1次)
    double loc_frequence_;

    /// @brief source点云最大点数量
    int maxpoints_source_ = 50000;
    /// @brief target点云最大点数量
    int maxpoints_target_ = 200000;

    /// @brief 初始化成功标志
    bool loc_initialized_ = false;

    /// @brief 当前定位overlap，confidence
    double loc_fitness_;

    /// @brief 定位置信度阈值
    double confidence_loc_th_;

    /// 卡尔曼滤波器
    KalmanFilter kf_baselink_x_;
    KalmanFilter kf_baselink_y_;
    KalmanFilter kf_baselink_z_;
    KalmanFilter kalman_filter_odom2map_;

    // 0:kf_processVar 1:kf_estimatedMeasVar
    std::vector<float> kf_param_x_;
    std::vector<float> kf_param_y_;
    std::vector<float> kf_param_z_;

    /// @brief 对odom2map进行kalman滤波
    bool filter_odom2map_ = false;
    double kalman_processVar2_ = 0.0;
    double kalman_estimatedMeasVar2_ = 0.0;

    /// 1202
    /// @brief 上次更新定位时的定位值
    Eigen::Vector3d last_loc_;
    // Eigen::Vector3d cur_loc_;
    /// @brief 更新地图子图的距离,超过则更新地图子图
    double dis_updatemap_;
};

GloabalLocalization::GloabalLocalization(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : nh_(nh),
                                                                                             nh_private_(nh_private)
{
    flag_exit_ = false;
    loc_initialized_ = false;
    mat_baselink2odom_ = Eigen::Matrix4d::Identity();
    mat_odom2map_ = Eigen::Matrix4d::Identity();
    mat_initialpose_ = Eigen::Matrix4d::Identity();
    mat_world_to_active_map_ = Eigen::Matrix4d::Identity();
    last_loc_ = Eigen::Vector3d(0, 0, -5000);

    pcd_map_ori_.reset(new open3d::geometry::PointCloud);
    pcd_map_coarse_.reset(new open3d::geometry::PointCloud);
    pcd_map_cur_.reset(new open3d::geometry::PointCloud);
    pcd_scan_cur_.reset(new open3d::geometry::PointCloud);
    pcd_map_fine_.reset(new open3d::geometry::PointCloud);
    queue_maxsize_ = 5;

    pub_baselink2map_ = nh.advertise<nav_msgs::Odometry>("/baselink2map", 100000);
    pub_baselink2map_kalman_ = nh.advertise<nav_msgs::Odometry>("/baselink2map_kalman", 100000);
    pub_motionlink2map_ = nh.advertise<nav_msgs::Odometry>("/motionlink2map", 100000);
    pub_odom2map_ = nh.advertise<nav_msgs::Odometry>("/odom2map", 100000);
    pub_odom2map_kalman_ = nh.advertise<nav_msgs::Odometry>("/odom2map_kalman", 100000);

    pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("/map", 1, true);
    pub_submap_ = nh.advertise<sensor_msgs::PointCloud2>("/submap", 1, true);
    pub_scan2map_ = nh.advertise<sensor_msgs::PointCloud2>("/scan2map", 1, true);
    pub_scan_ = nh.advertise<sensor_msgs::PointCloud2>("/scan", 1, true);
    pub_localization_3d_ = nh.advertise<geometry_msgs::PoseStamped>("/localization_3d", 1, false);
    pub_localization_3d_confidence_ = nh.advertise<std_msgs::Float32>("/localization_3d_confidence", 1, false);
    pub_localization_3d_delay_ms_ = nh.advertise<std_msgs::Float32>("/localization_3d_delay_ms", 1, false);
    pub_active_map_ = nh.advertise<std_msgs::String>("/localization_3d_active_map", 1, true);
    pub_status_ = nh.advertise<std_msgs::String>("/localization_3d_status", 1, false);

    loc_frequence_ = 2.0; //
    loc_fitness_ = 0.0;
    // 注册回调函数
    sub_baselink2odom_ = nh_.subscribe("/Odometry_loc", 50, &GloabalLocalization::CallbackBaselink2Odom, this);
    sub_scan_cur_ = nh_.subscribe("/cloud_registered_1", 50, &GloabalLocalization::CallbackScan, this);
    sub_initialpose_ = nh_.subscribe("/initialpose", 50, &GloabalLocalization::CallbackInitialPose, this);

    pose_baselink2odom_ = nav_msgs::Odometry();
    pose_baselink2odom_.header.frame_id = "odom";
    pose_baselink2odom_.child_frame_id = "base_link";
    // geometry_msgs的Quaternion会被初始化为0,0,0,0,而不是正确的0,0,0,1
    pose_baselink2odom_.pose.pose.orientation.w = 1;
    ROS_INFO("pose baselink2odom:\nx: %f, y: %f, z: %f, qx: %f, \
                            qy: %f, qz: %f, qw: %f",
             pose_baselink2odom_.pose.pose.position.x,
             pose_baselink2odom_.pose.pose.position.y,
             pose_baselink2odom_.pose.pose.position.z,
             pose_baselink2odom_.pose.pose.orientation.x,
             pose_baselink2odom_.pose.pose.orientation.y,
             pose_baselink2odom_.pose.pose.orientation.z,
             pose_baselink2odom_.pose.pose.orientation.w);

    // 队列最大数量
    nh_private_.param<int>("pcd_queue_maxsize", queue_maxsize_, 5);
    nh_private_.param<bool>("save_scan", save_scan_, false);
    /// 最大点数量限制
    nh_private_.param<int>("maxpoints_source", maxpoints_source_, 50000);
    nh_private_.param<int>("maxpoints_target", maxpoints_target_, 200000);

    // 定位间隔时间
    nh_private_.param<double>("loc_frequence", loc_frequence_, 2.0);

    /// 定位阈值
    nh_private_.param<double>("confidence_loc_th", confidence_loc_th_, 0.6);

    /// 卡尔曼参数
    nh_private_.param<std::vector<float>>("kf_baselink2map/x", kf_param_x_, std::vector<float>(2));
    nh_private_.param<std::vector<float>>("kf_baselink2map/y", kf_param_y_, std::vector<float>(2));
    nh_private_.param<std::vector<float>>("kf_baselink2map/z", kf_param_z_, std::vector<float>(2));

    nh_private_.param<bool>("filter_odom2map", filter_odom2map_, false);
    nh_private_.param<double>("kalman_processVar2", kalman_processVar2_, 0.02);
    nh_private_.param<double>("kalman_estimatedMeasVar2", kalman_estimatedMeasVar2_, 0.04);
    // voxelsize
    nh_private_.param<double>("voxelsize_coarse", voxelsize_coarse_, 0.2);
    nh_private_.param<double>("voxelsize_fine", voxelsize_fine_, 0.05);
    nh_private_.param<double>("threshold_fitness_init", threshold_fitness_init_, 0.9);
    nh_private_.param<double>("threshold_fitness", threshold_fitness_, 0.9);
    nh_private_.param<std::vector<float>>("initialpose", initialpose_, std::vector<float>());
    nh_private_.param<double>("dis_updatemap", dis_updatemap_, 5);

    for (auto i : initialpose_)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    mat_initialpose_.block<3, 3>(0, 0) = Euler2Matrix3d(Eigen::Vector3d(initialpose_[3], initialpose_[4], initialpose_[5]));
    mat_initialpose_.block<3, 1>(0, 3) = Eigen::Vector3d(initialpose_[0], initialpose_[1], initialpose_[2]);

    // ========== 读取地图(支持单地图 path_map / 多地图 maps) ==========
    std::string path_map = "";
    nh_private_.param<std::string>("path_map", path_map, "");

    // 多地图相关参数
    nh_private_.param<bool>("map_switch/enable", map_switch_enable_, false);
    nh_private_.param<double>("map_switch/bbox_shrink", bbox_shrink_, 1.0);
    nh_private_.param<double>("map_switch/verify_fitness", verify_fitness_, 0.6);
    nh_private_.param<int>("map_switch/verify_consecutive", verify_consecutive_, 3);
    nh_private_.param<int>("map_switch/verify_period_ms", verify_period_ms_, 500);
    nh_private_.param<int>("map_switch/cooldown_ms", cooldown_ms_, 5000);
    nh_private_.param<double>("map_switch/verify_icp_threshold", verify_icp_threshold_, 0.0);
    nh_private_.param<double>("map_switch/verify_max_translation", verify_max_translation_, 1.0);
    nh_private_.param<double>("map_switch/verify_max_yaw_deg", verify_max_yaw_deg_, 20.0);
    nh_private_.param<double>("map_switch/post_switch_min_fitness", post_switch_min_fitness_, 0.3);
    nh_private_.param<bool>("map_switch/allow_uncalibrated_switch", allow_uncalibrated_switch_, false);
    if (verify_icp_threshold_ <= 0.0)
    {
        verify_icp_threshold_ = voxelsize_fine_ * 3.0;
    }

    // 1) 优先读 maps 列表
    XmlRpc::XmlRpcValue maps_param;
    bool has_maps = nh_private_.getParam("maps", maps_param) &&
                    maps_param.getType() == XmlRpc::XmlRpcValue::TypeArray &&
                    maps_param.size() > 0;
    if (has_maps)
    {
        for (int i = 0; i < maps_param.size(); ++i)
        {
            XmlRpc::XmlRpcValue &item = maps_param[i];
            if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
                !item.hasMember("name") || !item.hasMember("path"))
            {
                ROS_ERROR("maps[%d] must have name and path fields", i);
                ros::shutdown();
                return;
            }
            MapEntry entry;
            entry.name = static_cast<std::string>(item["name"]);
            entry.path = static_cast<std::string>(item["path"]);
            entry.frame_id = "map_" + entry.name;
            // switch_zone: [xmin, ymin, xmax, ymax]
            if (item.hasMember("switch_zone"))
            {
                XmlRpc::XmlRpcValue &sz = item["switch_zone"];
                if (sz.getType() == XmlRpc::XmlRpcValue::TypeArray && sz.size() == 4)
                {
                    entry.switch_zone = Eigen::Vector4d(
                        XmlRpcToDouble(sz[0]), XmlRpcToDouble(sz[1]),
                        XmlRpcToDouble(sz[2]), XmlRpcToDouble(sz[3]));
                    entry.switch_zone_set = true;
                    ROS_WARN("maps[%d] '%s' switch_zone: [%.2f, %.2f, %.2f, %.2f]",
                             i, entry.name.c_str(),
                             entry.switch_zone.x(), entry.switch_zone.y(),
                             entry.switch_zone.z(), entry.switch_zone.w());
                }
            }
            // initialpose: [x, y, z, roll, pitch, yaw] degrees
            if (item.hasMember("initialpose"))
            {
                XmlRpc::XmlRpcValue &ip = item["initialpose"];
                if (ip.getType() == XmlRpc::XmlRpcValue::TypeArray && ip.size() == 6)
                {
                    entry.initialpose.clear();
                    for (int j = 0; j < 6; ++j)
                        entry.initialpose.push_back(XmlRpcToDouble(ip[j]));
                    entry.initialpose_set = true;
                    ROS_WARN("maps[%d] '%s' initialpose: [%.2f, %.2f, %.2f, %.1f, %.1f, %.1f]",
                             i, entry.name.c_str(), entry.initialpose[0], entry.initialpose[1],
                             entry.initialpose[2], entry.initialpose[3], entry.initialpose[4], entry.initialpose[5]);
                }
            }
            // next_initialpose: [x, y, z, roll, pitch, yaw] degrees
            if (item.hasMember("next_initialpose"))
            {
                XmlRpc::XmlRpcValue &nip = item["next_initialpose"];
                if (nip.getType() == XmlRpc::XmlRpcValue::TypeArray && nip.size() == 6)
                {
                    entry.next_initialpose.clear();
                    for (int j = 0; j < 6; ++j)
                        entry.next_initialpose.push_back(XmlRpcToDouble(nip[j]));
                    entry.next_initialpose_set = true;
                    // 全零 next_initialpose 视为未标定，不允许自动切图
                    entry.next_initialpose_calibrated = !IsAllZeros(entry.next_initialpose);
                    ROS_WARN("maps[%d] '%s' next_initialpose: [%.2f, %.2f, %.2f, %.1f, %.1f, %.1f] calibrated=%s",
                             i, entry.name.c_str(), entry.next_initialpose[0], entry.next_initialpose[1],
                             entry.next_initialpose[2], entry.next_initialpose[3], entry.next_initialpose[4], entry.next_initialpose[5],
                             entry.next_initialpose_calibrated ? "true" : "false");
                }
            }
            if (!LoadMap(entry.name, entry.path, entry))
            {
                ROS_ERROR("LoadMap failed for %s (%s)", entry.name.c_str(), entry.path.c_str());
                ros::shutdown();
                return;
            }
            maps_.push_back(std::move(entry));
        }
    }
    // 2) 退化为单地图
    else if (!path_map.empty())
    {
        MapEntry entry;
        entry.name = "default";
        entry.path = path_map;
        entry.frame_id = "map";
        if (!LoadMap(entry.name, entry.path, entry))
        {
            ROS_ERROR("LoadMap failed for path_map: %s", path_map.c_str());
            ros::shutdown();
            return;
        }
        maps_.push_back(std::move(entry));
        map_switch_enable_ = false; // 单地图无意义
    }
    else
    {
        ROS_ERROR("neither 'maps' nor 'path_map' is set, cannot start");
        ros::shutdown();
        return;
    }

    // 3) 选 active_map
    std::string active_map_name;
    nh_private_.param<std::string>("map_switch/active_map", active_map_name, maps_[0].name);
    int active_idx = -1;
    for (size_t i = 0; i < maps_.size(); ++i)
    {
        if (maps_[i].name == active_map_name)
        {
            active_idx = static_cast<int>(i);
            break;
        }
    }
    if (active_idx < 0)
    {
        ROS_WARN("active_map '%s' not in maps list, fallback to '%s'",
                 active_map_name.c_str(), maps_[0].name.c_str());
        active_idx = 0;
    }
    active_map_idx_.store(active_idx);
    active_map_frame_ = maps_[active_idx].frame_id;
    ROS_WARN("active map: %s (frame=%s, %zu maps loaded, switch=%s)",
             maps_[active_idx].name.c_str(), active_map_frame_.c_str(), maps_.size(),
             map_switch_enable_ ? "on" : "off");

    // 4) 兼容旧字段:把 active 的 fine/coarse 复制给老指针,供历史代码继续用
    pcd_map_coarse_ = maps_[active_idx].map_coarse;
    pcd_map_fine_ = maps_[active_idx].map_fine;

    // 5) 发布初始 /map(latched)
    {
        sensor_msgs::PointCloud2 pc2_map;
        open3d_conversions::open3dToRos(*maps_[active_idx].map_coarse, pc2_map);
        pc2_map.header.frame_id = active_map_frame_;
        pc2_map.header.stamp = ros::Time::now();
        pub_map_.publish(pc2_map);

        std_msgs::String s;
        s.data = maps_[active_idx].name;
        pub_active_map_.publish(s);

        // 方案 B: 初始 T_world_map = Identity, 切换时更新以保持视觉连续
        mat_world_to_active_map_ = Eigen::Matrix4d::Identity();

        // 发布初始静态 TF, 让 RViz 在 odom 回调启动前就能解析 TF 链
        geometry_msgs::TransformStamped world_to_map;
        world_to_map.header.frame_id = "localization_world";
        world_to_map.child_frame_id = active_map_frame_;
        world_to_map.transform.translation.x = 0;
        world_to_map.transform.translation.y = 0;
        world_to_map.transform.translation.z = 0;
        world_to_map.transform.rotation.w = 1;
        world_to_map.transform.rotation.x = 0;
        world_to_map.transform.rotation.y = 0;
        world_to_map.transform.rotation.z = 0;
        static_broadcaster_.sendTransform(world_to_map);
    }

    GetTfTransformToMatrix("base_link", "imu_link", mat_imulink2baselink_);
    std::cout << "mat_imulink2baselink_:\n"
              << mat_imulink2baselink_ << std::endl;

    GetTfTransformToMatrix("motion_link", "base_link", mat_baselink2motionlink_);
    std::cout << "mat_baselink2motionlink_:\n"
              << mat_baselink2motionlink_ << std::endl;
    ROS_WARN("initialize finished");
}

GloabalLocalization::~GloabalLocalization()
{
    lock_exit_.lock();
    flag_exit_ = true;
    lock_exit_.unlock();
    if (thread_monitor_.joinable())
    {
        thread_monitor_.join();
    }
    if (thread_loc_.joinable())
    {
        thread_loc_.join();
    }
}

bool GloabalLocalization::LoadMap(const std::string &name, const std::string &path, MapEntry &out)
{
    auto pcd_ori = std::make_shared<open3d::geometry::PointCloud>();
    if (!open3d::io::ReadPointCloud(path, *pcd_ori) || pcd_ori->IsEmpty())
    {
        ROS_ERROR("LoadMap: read '%s' failed (path=%s)", name.c_str(), path.c_str());
        return false;
    }
    pcd_ori->PaintUniformColor({1, 0, 0});

    out.name = name;
    out.path = path;
    out.map_coarse = pcd_ori->VoxelDownSample(voxelsize_coarse_);
    out.map_coarse->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_coarse_ * 2, 30));
    out.map_fine = pcd_ori->VoxelDownSample(voxelsize_fine_);
    out.map_fine->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_fine_ * 2, 30));

    auto aabb = pcd_ori->GetAxisAlignedBoundingBox();
    out.bbox_min = aabb.min_bound_;
    out.bbox_max = aabb.max_bound_;
    ROS_WARN("loaded map '%s': %zu pts (fine), bbox [%.2f,%.2f,%.2f]~[%.2f,%.2f,%.2f]",
             name.c_str(), out.map_fine->points_.size(),
             out.bbox_min.x(), out.bbox_min.y(), out.bbox_min.z(),
             out.bbox_max.x(), out.bbox_max.y(), out.bbox_max.z());
    return true;
}

bool GloabalLocalization::PoseInsideMap(const Eigen::Vector3d &xyz, const MapEntry &m) const
{
    Eigen::Vector3d lo = m.bbox_min + Eigen::Vector3d::Constant(bbox_shrink_);
    Eigen::Vector3d hi = m.bbox_max - Eigen::Vector3d::Constant(bbox_shrink_);
    return xyz.x() >= lo.x() && xyz.x() <= hi.x() &&
           xyz.y() >= lo.y() && xyz.y() <= hi.y();
    // 不限制 z(z 方向通常很窄,会误判出图)
}

bool GloabalLocalization::PoseInSwitchZone(const Eigen::Vector3d &xyz, const MapEntry &m) const
{
    if (!m.switch_zone_set) return PoseInsideMap(xyz, m);
    return xyz.x() >= m.switch_zone.x() && xyz.x() <= m.switch_zone.z() &&
           xyz.y() >= m.switch_zone.y() && xyz.y() <= m.switch_zone.w();
}

Eigen::Matrix3d GloabalLocalization::Euler2Matrix3d(const Eigen::Vector3d euler)
{
    Eigen::Matrix3d mat3d;
    // convert degrees to radians
    auto eulerAngle = euler / 180 * M_PI;
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitZ()));
    mat3d = rollAngle * pitchAngle * yawAngle;
    return mat3d;
}
bool GloabalLocalization::GetTfTransformToMatrix(std::string frame_id, std::string child_frame_id, Eigen::Matrix4d &matrix)
{
    // 获取pose
    tf::StampedTransform pose_;
    tf::TransformListener tf_listener_;
    try
    {
        tf_listener_.waitForTransform(frame_id, child_frame_id, ros::Time(0), ros::Duration(3));
        tf_listener_.lookupTransform(frame_id, child_frame_id, ros::Time(0), pose_);
    }
    catch (tf::TransformException &e)
    {
        ROS_ERROR("[GetTransformMatrix]: %s", e.what());
        return false;
    }

    Eigen::Vector3d translation = Eigen::Vector3d(pose_.getOrigin().x(), pose_.getOrigin().y(), pose_.getOrigin().z());
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

    quat = Eigen::Quaterniond(pose_.getRotation().w(),
                              pose_.getRotation().x(),
                              pose_.getRotation().y(),
                              pose_.getRotation().z());
    Eigen::Matrix3d rotation = quat.matrix();

    matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = rotation;
    matrix.matrix().block<3, 1>(0, 3) = translation;
    return true;
}

void GloabalLocalization::CallbackBaselink2Odom(
    const nav_msgs::Odometry::ConstPtr &baselink2odom)
{
    auto odom_cbk_s = std::chrono::high_resolution_clock::now();
    lock_timestamp_.lock();
    timestamp_odom_ = baselink2odom->header.stamp;
    lock_timestamp_.unlock();
    Eigen::Isometry3d mat_current = Eigen::Isometry3d::Identity();
    tf::poseMsgToEigen(baselink2odom->pose.pose, mat_current);
    auto mat_imulink2odom = mat_current.matrix();

    std::string frame;
    Eigen::Matrix4d baselink2odom_new = mat_imulink2odom * mat_imulink2baselink_.inverse();
    Eigen::Matrix4d baselink2map_cur;
    Eigen::Matrix4d odom2map_cur;
    {
        std::lock_guard<std::mutex> lk(lock_state_);
        mat_baselink2odom_ = baselink2odom_new;
        mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;
        baselink2map_cur = mat_baselink2map_;
        odom2map_cur = mat_odom2map_;
        frame = active_map_frame_;
    }

    Eigen::Isometry3d Isometry3d_baselink2map;
    Isometry3d_baselink2map.matrix() = baselink2map_cur;
    nav_msgs::Odometry baselink2map;
    tf::poseEigenToMsg(Isometry3d_baselink2map, baselink2map.pose.pose);
    baselink2map.header.frame_id = frame;
    baselink2map.child_frame_id = "base_link";
    baselink2map.header.stamp = baselink2odom->header.stamp;
    pub_baselink2map_.publish(baselink2map);

    Eigen::Isometry3d Isometry3d_odom2map;
    Isometry3d_odom2map.matrix() = odom2map_cur;
    nav_msgs::Odometry odom2map;
    tf::poseEigenToMsg(Isometry3d_odom2map, odom2map.pose.pose);
    odom2map.header.frame_id = frame;
    odom2map.child_frame_id = "odom";
    odom2map.header.stamp = baselink2odom->header.stamp;
    pub_odom2map_.publish(odom2map);

    static tf::TransformBroadcaster br_odom2map;
    tf::Transform transform_odom2map;
    tf::Quaternion q_odom2map;
    transform_odom2map.setOrigin(tf::Vector3(odom2map.pose.pose.position.x,
                                             odom2map.pose.pose.position.y,
                                             odom2map.pose.pose.position.z));
    q_odom2map.setW(odom2map.pose.pose.orientation.w);
    q_odom2map.setX(odom2map.pose.pose.orientation.x);
    q_odom2map.setY(odom2map.pose.pose.orientation.y);
    q_odom2map.setZ(odom2map.pose.pose.orientation.z);
    transform_odom2map.setRotation(q_odom2map);
    br_odom2map.sendTransform(tf::StampedTransform(transform_odom2map, baselink2odom->header.stamp, frame, "odom"));

    // 方案 B: 动态发布 localization_world -> active_map_frame, 使用 mat_world_to_active_map_
    {
        Eigen::Matrix4d w2m;
        {
            std::lock_guard<std::mutex> lk(lock_state_);
            w2m = mat_world_to_active_map_;
        }
        static tf::TransformBroadcaster br_world;
        tf::Transform tf_world;
        Eigen::Quaterniond q_w(w2m.block<3,3>(0,0));
        tf_world.setOrigin(tf::Vector3(w2m(0,3), w2m(1,3), w2m(2,3)));
        tf::Quaternion q_tf;
        q_tf.setW(q_w.w()); q_tf.setX(q_w.x()); q_tf.setY(q_w.y()); q_tf.setZ(q_w.z());
        tf_world.setRotation(q_tf);
        br_world.sendTransform(tf::StampedTransform(tf_world, baselink2odom->header.stamp, "localization_world", frame));
    }

    if (loc_initialized_)
    {
        Eigen::Matrix4d mat_baselink2map_kalman = Eigen::Matrix4d::Identity();

        if (filter_odom2map_)
        {
            Eigen::Matrix4d odom2map_kalman_snap;
            Eigen::Matrix4d baselink2odom_snap;
            {
                std::lock_guard<std::mutex> lk(lock_state_);
                odom2map_kalman_snap = mat_odom2map_kalman_;
                baselink2odom_snap = mat_baselink2odom_;
            }
            Eigen::Isometry3d Isometry3d_odom2map_kalman;
            Isometry3d_odom2map_kalman.matrix() = odom2map_kalman_snap;
            nav_msgs::Odometry odom2map_kalman;
            tf::poseEigenToMsg(Isometry3d_odom2map_kalman, odom2map_kalman.pose.pose);
            odom2map_kalman.header.frame_id = frame;
            odom2map_kalman.child_frame_id = "odom_kalman";
            odom2map_kalman.header.stamp = baselink2odom->header.stamp;
            pub_odom2map_kalman_.publish(odom2map_kalman);

            kf_baselink_z_.inputLatestNoisyMeasurement((odom2map_kalman_snap * baselink2odom_snap)(2, 3));
            mat_baselink2map_kalman = odom2map_kalman_snap * baselink2odom_snap;
        }
        else
        {
            kf_baselink_x_.inputLatestNoisyMeasurement((baselink2map_cur)(0, 3));
            kf_baselink_y_.inputLatestNoisyMeasurement((baselink2map_cur)(1, 3));
            kf_baselink_z_.inputLatestNoisyMeasurement((baselink2map_cur)(2, 3));
            mat_baselink2map_kalman = baselink2map_cur;
        }

        mat_baselink2map_kalman(2, 3) = kf_baselink_z_.getLatestEstimatedMeasurement();
        Eigen::Isometry3d Isometry3d_baselink2map_kalman;

        Isometry3d_baselink2map_kalman.matrix() = mat_baselink2map_kalman;
        nav_msgs::Odometry baselink2map_kalman;
        tf::poseEigenToMsg(Isometry3d_baselink2map_kalman, baselink2map_kalman.pose.pose);
        baselink2map_kalman.header.frame_id = frame;
        baselink2map_kalman.header.stamp = baselink2odom->header.stamp;
        pub_baselink2map_kalman_.publish(baselink2map_kalman);

        Eigen::Matrix4d mat_motionlink2map = mat_baselink2map_kalman * mat_baselink2motionlink_.inverse();
        Eigen::Isometry3d Isometry3d_motionlink2map;
        Isometry3d_motionlink2map.matrix() = mat_motionlink2map;
        nav_msgs::Odometry motionlink2map;
        tf::poseEigenToMsg(Isometry3d_motionlink2map, motionlink2map.pose.pose);
        motionlink2map.header.frame_id = frame;
        motionlink2map.header.stamp = baselink2odom->header.stamp;
        pub_motionlink2map_.publish(motionlink2map);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(motionlink2map.pose.pose.position.x,
                                        motionlink2map.pose.pose.position.y,
                                        motionlink2map.pose.pose.position.z));
        q.setW(motionlink2map.pose.pose.orientation.w);
        q.setX(motionlink2map.pose.pose.orientation.x);
        q.setY(motionlink2map.pose.pose.orientation.y);
        q.setZ(motionlink2map.pose.pose.orientation.z);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, baselink2odom->header.stamp, frame, "motion_link"));

        localization_3d_confidence_.data = loc_fitness_;
        pub_localization_3d_confidence_.publish(localization_3d_confidence_);
        localization_3d_delay_ms_.data = (ros::Time::now().toSec() - baselink2odom->header.stamp.toSec()) * 1000.0;
        pub_localization_3d_delay_ms_.publish(localization_3d_delay_ms_);
        localization_3d_.header.frame_id = frame;
        localization_3d_.header.stamp = baselink2odom->header.stamp;
        localization_3d_.pose = motionlink2map.pose.pose;
        pub_localization_3d_.publish(localization_3d_);
    }
}
void GloabalLocalization::CallbackScan(
    const sensor_msgs::PointCloud2::ConstPtr &scan_in_baselink)
{
    auto cbk_s = std::chrono::high_resolution_clock::now();
    open3d::geometry::PointCloud pcd_recieved;
    // 单帧转换为open3d，几百us
    open3d_conversions::rosToOpen3d(scan_in_baselink, pcd_recieved);
    // 入队列
    // pcd_recieved
    if (que_pcd_scan_.size() >= queue_maxsize_)
    {
        std::queue<open3d::geometry::PointCloud> que_temp;
        lock_scan_.lock();
        pcd_scan_cur_->Clear();
        while (!que_pcd_scan_.empty())
        {
            *pcd_scan_cur_ += que_pcd_scan_.front();
            que_temp.push(que_pcd_scan_.front());
            que_pcd_scan_.pop();
        }
        lock_scan_.unlock();
        while (!que_temp.empty())
        {
            que_pcd_scan_.push(que_temp.front());
            que_temp.pop();
        }
        // 丢弃一个最旧的数据
        que_pcd_scan_.pop();
    }
    // 放入最新数据
    que_pcd_scan_.push(pcd_recieved);

    auto cbk_e = std::chrono::high_resolution_clock::now();
}

void GloabalLocalization::LocalizationInitialize()
{
    /// 裁剪后的地图
    std::shared_ptr<open3d::geometry::PointCloud> map_coarse_crop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_fine_crop(new open3d::geometry::PointCloud);

    /// 当前环境感知子图点云
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan(new open3d::geometry::PointCloud);
    /// 环境感知子图转换到地图坐标系
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan2map(new open3d::geometry::PointCloud);

    /// 用于配准的source target
    std::shared_ptr<open3d::geometry::PointCloud> source(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);

    /// cropbox,用于裁剪地图和当前环境感知子图
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_map(new open3d::geometry::OrientedBoundingBox);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_scan(new open3d::geometry::OrientedBoundingBox);

    /// 当前baselink到odom(camera_init)和map坐标系的关系
    Eigen::Matrix4d mat_baselink2odom_cur = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_baselink2map_cur = Eigen::Matrix4d::Identity();

    /// 固定感知子图/历史地图子图大小
    OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_map->color_ = Eigen::Vector3d(1, 0.5, 0);
    OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan->color_ = Eigen::Vector3d(0, 1, 0);

    double fitness_initial; /// overlap
    double loc_cost = 0;    /// 定位耗时(ms)
    int count_success = 0;
    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lk(lock_exit_);
            if (flag_exit_) return;
        }
        auto loc_s = std::chrono::high_resolution_clock::now(); /// 开始定位计时
        lock_scan_.lock();
        if (pcd_scan_cur_->IsEmpty())
        {
            lock_scan_.unlock();
            open3d::utility::LogInfo("wait for pcd_scan_cur_");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        else
        {
            {
                std::lock_guard<std::mutex> lk(lock_state_);
                mat_baselink2odom_cur = mat_baselink2odom_;
                mat_baselink2map_cur = mat_baselink2map_;
            }
            *pcd_scan = *pcd_scan_cur_;
            lock_scan_.unlock();

            OBB_map->center_ = mat_baselink2map_cur.block<3, 1>(0, 3);
            OBB_map->R_ = mat_baselink2map_cur.block<3, 3>(0, 0);
            OBB_scan->center_ = mat_baselink2odom_cur.block<3, 1>(0, 3);
            OBB_scan->R_ = mat_baselink2odom_cur.block<3, 3>(0, 0);
            {
                std::shared_lock<std::shared_mutex> rlock(lock_maps_);
                *map_fine_crop = *maps_[active_map_idx_.load()].map_fine->Crop(*OBB_map);
            }

            auto reg0_s = std::chrono::high_resolution_clock::now();

            Eigen::Matrix4d reg_matrix = Eigen::Matrix4d::Identity();
            {
                std::lock_guard<std::mutex> lk(lock_state_);
                reg_matrix = mat_odom2map_;
            }

            *target = *map_fine_crop;
            open3d::utility::LogInfo("before sample, target size: {}, has normal: {}", target->points_.size(), target->HasNormals() ? "true" : "false");
            if (target->points_.size() > maxpoints_target_)
            {
                target = target->RandomDownSample(double(maxpoints_target_) / target->points_.size());
            }
            open3d::utility::LogInfo("after sample, target size: {}, has normal: {}", target->points_.size(), target->HasNormals() ? "true" : "false");

            source = pcd_scan->Crop(*OBB_scan);
            open3d::utility::LogInfo("source size: {}, has normal: {}", source->points_.size(), source->HasNormals() ? "true" : "false");
            if (source->points_.size() > maxpoints_source_)
            {
                source = source->RandomDownSample(double(maxpoints_source_) / source->points_.size());
            }

            open3d::utility::LogInfo("source size: {}, has normal: {}", source->points_.size(), source->HasNormals() ? "true" : "false");

            source->Transform(reg_matrix);
            *pcd_scan2map = *source;

            auto multiScale_reg_matrix = pcd_tools::RegistrationMultiScaleIcp(source, target, voxelsize_fine_, 1, {1, 4, 6});
            reg_matrix = multiScale_reg_matrix * reg_matrix;
            source->Transform(multiScale_reg_matrix);
            auto eva_result_coarse = open3d::pipelines::registration::EvaluateRegistration(*source, *target, voxelsize_fine_ * 3);
            open3d::utility::LogInfo("eva fitness: {}", eva_result_coarse.fitness_);
            fitness_initial = eva_result_coarse.fitness_;
            *pcd_scan2map = *source;

            {
                std::lock_guard<std::mutex> lk(lock_state_);
                mat_odom2map_ = reg_matrix;
            }
            auto loc_e = std::chrono::high_resolution_clock::now(); /// 结束定位计时
            loc_cost = std::chrono::duration_cast<std::chrono::microseconds>(loc_e - loc_s).count() / 1000.0;
            ROS_INFO("localization cost: %f ms", loc_cost);

            if (fitness_initial > threshold_fitness_init_)
            {
                count_success += 1;
                /// 连续两次定位成功后定位初始化成功
                if (count_success >= 2)
                {
                    break;
                }
            }
            else
            {
                count_success = 0;
            }
        }
    }


    open3d::utility::LogInfo("\n\n\nlocalization initialize success!!!!\n\n\n");
}
void GloabalLocalization::Localization()
{
    ROS_INFO("wait for Odometry_loc");
    auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/Odometry_loc", nh_);
    if (!odom_msg || !ros::ok()) return;
    ROS_INFO("wait for cloud_registered_1");
    auto scan_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cloud_registered_1", nh_);
    if (!scan_msg || !ros::ok()) return;
    // initialize
    /****初始化定位****/
    // 如果当前 active 地图有 per-map initialpose,优先使用
    {
        int active = active_map_idx_.load();
        std::shared_lock<std::shared_mutex> rlock(lock_maps_);
        if (maps_[active].initialpose_set)
        {
            auto &ip = maps_[active].initialpose;
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 1>(0, 3) = Eigen::Vector3d(ip[0], ip[1], ip[2]);
            T.block<3, 3>(0, 0) = Euler2Matrix3d(Eigen::Vector3d(ip[3], ip[4], ip[5]));
            mat_initialpose_ = T;
            ROS_WARN("Localization: use per-map initialpose (T_map_base) for '%s': "
                     "[%.2f, %.2f, %.2f, %.1f, %.1f, %.1f]",
                     maps_[active].name.c_str(), ip[0], ip[1], ip[2], ip[3], ip[4], ip[5]);
        }
    }
    {
        std::lock_guard<std::mutex> lk(lock_state_);
        mat_odom2map_ = mat_initialpose_ * mat_baselink2odom_.inverse();
    }
    LocalizationInitialize();
    {
        std::lock_guard<std::mutex> lk(lock_exit_);
        if (flag_exit_ || !ros::ok()) return;
    }

    /// 卡尔曼滤波初始化
    {
        Eigen::Matrix4d bl2m;
        {
            std::lock_guard<std::mutex> lk(lock_state_);
            bl2m = mat_baselink2map_;
        }
        kf_baselink_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1], bl2m(0, 3), 1);
        kf_baselink_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1], bl2m(1, 3), 1);
        kf_baselink_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1], bl2m(2, 3), 1);
        kalman_filter_odom2map_.KalmanFilterInit(kalman_processVar2_, kalman_estimatedMeasVar2_, bl2m(2, 3), 1);
    }

    loc_initialized_ = true; /// 初始化成功

    double fitness = 0;
    auto coordinate_ori = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    auto coordinate_loc = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    auto coordinate_OBB_scan = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scancrop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan2map(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> source(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_coarse_crop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_fine_crop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_submap(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_map(new open3d::geometry::OrientedBoundingBox);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_scan(new open3d::geometry::OrientedBoundingBox);
    OBB_map->color_ = Eigen::Vector3d(1, 0.5, 0);
    OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);

    OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan->color_ = Eigen::Vector3d(0, 1, 0);
    ros::Time time_current = timestamp_odom_;
    ros::Time time_last = time_current - ros::Duration(3.0);

    ROS_INFO("time_last: %f", time_last.toSec());
    ROS_INFO("time_current: %f", time_current.toSec());
    int scan_count = 0;

    std::string save_path = "/home/carlos/mount/E/lixin/data/yq_bag/scan_submap/";

    double time_diff_loc = 5;                                     /// 前后两次定位的时间差(s)
    std::chrono::high_resolution_clock::time_point time_last_loc; /// 上次定位的完成时间点
    std::chrono::high_resolution_clock::time_point time_this_loc; /// 当前定位的开始时间点
    double loc_cost = 0;                                          /// 定位耗时(ms)
    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lk(lock_exit_);
            if (flag_exit_) break;
        }

        // Snapshot epoch and active_map_idx at start of each iteration
        uint64_t epoch_begin = map_epoch_.load();
        int active_idx_begin = active_map_idx_.load();

        if (manual_reinit_pending_.exchange(false))
        {
            LocalizationInitialize();
            {
                std::lock_guard<std::mutex> lk(lock_state_);
                Eigen::Matrix4d bl2m = mat_odom2map_ * mat_baselink2odom_;
                kf_baselink_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1], bl2m(0, 3), 1);
                kf_baselink_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1], bl2m(1, 3), 1);
                kf_baselink_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1], bl2m(2, 3), 1);
                kalman_filter_odom2map_.KalmanFilterInit(kalman_processVar2_, kalman_estimatedMeasVar2_, bl2m(2, 3), 1);
            }
            loc_initialized_ = true;
            ROS_WARN("Localization: manual reinit completed");
        }

        lock_timestamp_.lock();
        time_current = timestamp_odom_;
        lock_timestamp_.unlock();
        auto time_diff_frame = time_current.toSec() - time_last.toSec();
        time_last = time_current;
        if (std::fabs(time_diff_frame) < 1e-6)
        {
            loc_cost = 0.0;
            continue;
        }

        time_this_loc = std::chrono::high_resolution_clock::now();
        time_diff_loc = std::chrono::duration_cast<std::chrono::microseconds>(time_this_loc - time_last_loc).count() / 1000000.0 + loc_cost / 1000.0;

        if (time_diff_loc < loc_frequence_)
        {
            int wait_time = int((loc_frequence_ - time_diff_loc) * 1000);
            open3d::utility::LogInfo("\n\ntime_this_loc: {}, time_last: {},\ntime_diff: {} s, sleep {} ms",
                                     std::chrono::duration_cast<std::chrono::milliseconds>(time_this_loc.time_since_epoch()).count() / 1000.0,
                                     std::chrono::duration_cast<std::chrono::milliseconds>(time_last_loc.time_since_epoch()).count() / 1000.0, time_diff_loc, wait_time);
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
        }
        else
        {
            open3d::utility::LogInfo("\n\ntime_diff:{} s, localization right now", time_diff_loc);
        }
        auto loc_s = std::chrono::high_resolution_clock::now();

        lock_scan_.lock();
        if (pcd_scan_cur_->IsEmpty())
        {
            lock_scan_.unlock();
            ROS_INFO("wait for pcd_scan_cur_");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        else
        {
            Eigen::Matrix4d mat_baselink2odom_cur = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d mat_baselink2map_cur = Eigen::Matrix4d::Identity();
            {
                std::lock_guard<std::mutex> lk(lock_state_);
                if (filter_odom2map_)
                {
                    kalman_filter_odom2map_.inputLatestNoisyMeasurement(mat_odom2map_(2, 3));
                    kalman_filter_odom2map_.inputLatestNoisyMeasurement(mat_odom2map_(2, 3));
                    mat_odom2map_kalman_ = mat_odom2map_;
                    mat_odom2map_kalman_(2, 3) = kalman_filter_odom2map_.getLatestEstimatedMeasurement();
                }
                mat_baselink2odom_cur = mat_baselink2odom_;
                mat_baselink2map_cur = mat_odom2map_ * mat_baselink2odom_;
            }
            *pcd_scan = *pcd_scan_cur_;
            lock_scan_.unlock();
            Eigen::Vector3d cur_loc(mat_baselink2map_cur(0, 3), mat_baselink2map_cur(1, 3), mat_baselink2map_cur(2, 3));
            auto dis_motion = ComputeMotionDis(last_loc_, cur_loc);
            bool just_switched = force_resubmap_.exchange(false);
            // Check if cached submap is stale (map epoch or idx changed)
            bool submap_stale = (cached_submap_epoch_ != epoch_begin) ||
                                (cached_submap_map_idx_ != active_idx_begin);
            if (dis_motion > dis_updatemap_ || just_switched || submap_stale)
            {
                auto submap_s = std::chrono::high_resolution_clock::now();

                open3d::utility::LogInfo("\n***\n****\n***\n\n\nlast map update loc: x: {}, y: {}, z{},\n\
                now loc: x: {}, y: {}, z{}, 3d distance: {}, now needpdate submap (just_switched={})",
                                         last_loc_.x(), last_loc_.y(), last_loc_.z(), cur_loc.x(), cur_loc.y(), cur_loc.z(), dis_motion, just_switched);
                last_loc_ = cur_loc;
                OBB_map->center_ = mat_baselink2map_cur.block<3, 1>(0, 3);
                OBB_map->R_ = mat_baselink2map_cur.block<3, 3>(0, 0);

                {
                    std::shared_lock<std::shared_mutex> rlock(lock_maps_);
                    *map_fine_crop = *maps_[active_map_idx_.load()].map_fine->Crop(*OBB_map);
                }
                cached_submap_epoch_ = epoch_begin;
                cached_submap_map_idx_ = active_idx_begin;

                auto submap_e = std::chrono::high_resolution_clock::now();
                auto submap_cost = std::chrono::duration_cast<std::chrono::microseconds>(submap_e - submap_s).count() / 1000.0;
                ROS_INFO("submap_cost: %f ms", submap_cost);
            }

            OBB_scan->center_ = mat_baselink2odom_cur.block<3, 1>(0, 3);
            OBB_scan->R_ = mat_baselink2odom_cur.block<3, 3>(0, 0);

            auto reg0_s = std::chrono::high_resolution_clock::now();

            Eigen::Matrix4d reg_matrix = Eigen::Matrix4d::Identity();
            {
                std::lock_guard<std::mutex> lk(lock_state_);
                reg_matrix = mat_odom2map_;
            }

            *target = *map_fine_crop;
            open3d::utility::LogInfo("before sample, target size: {}, has normal: {}", target->points_.size(), target->HasNormals() ? "true" : "false");
            if (target->points_.size() > maxpoints_target_)
            {
                target = target->RandomDownSample(double(maxpoints_target_) / target->points_.size());
            }
            open3d::utility::LogInfo("after sample, target size: {}, has normal: {}", target->points_.size(), target->HasNormals() ? "true" : "false");

            source = pcd_scan->Crop(*OBB_scan);
            open3d::utility::LogInfo("source size: {}, maxpoints_source_: {}", source->points_.size(), maxpoints_source_);
            source = source->VoxelDownSample(voxelsize_fine_);
            open3d::utility::LogInfo("source size after voxel downsample: {}", source->points_.size());
            if (source->points_.size() > maxpoints_source_)
            {
                source = source->RandomDownSample(double(maxpoints_source_) / source->points_.size());
            }
            open3d::utility::LogInfo("after prerpocess: {}", source->points_.size());

            auto reg_result2 = pcd_tools::RegistrationIcp(source, target, voxelsize_fine_ * 2, reg_matrix, 1);
            reg_matrix = reg_result2.transformation_ * reg_matrix;
            auto eva_result2 = open3d::pipelines::registration::EvaluateRegistration(*source, *target, voxelsize_fine_ * 4, reg_matrix);
            loc_fitness_ = eva_result2.fitness_;
            open3d::utility::LogInfo("reg_result.fitness: {}, eva fitness: {}", reg_result2.fitness_, eva_result2.fitness_);
            if (loc_fitness_ > threshold_fitness_)
            {
                // Discard this ICP result if map epoch or active idx changed during computation
                if (epoch_begin != map_epoch_.load() || active_idx_begin != active_map_idx_.load())
                {
                    ROS_WARN("Localization: discarding stale ICP result (epoch %lu->%lu or idx %d->%d)",
                             (unsigned long)epoch_begin, (unsigned long)map_epoch_.load(),
                             active_idx_begin, active_map_idx_.load());
                }
                else
                {
                    std::lock_guard<std::mutex> lk(lock_state_);
                    mat_odom2map_ = reg_matrix;
                }
            }

            if (save_scan_)
            {
                pcd_scan->Transform(mat_baselink2odom_cur.inverse());
                pcd_scan2map->Transform(mat_baselink2map_cur.inverse());
                open3d::io::WritePointCloud(save_path + std::to_string(scan_count) + "_ori.ply", *pcd_scan);
                open3d::io::WritePointCloud(save_path + std::to_string(scan_count) + "_crop.ply", *pcd_scan2map);
                scan_count += 1;
            }

            auto loc_e = std::chrono::high_resolution_clock::now();
            time_last_loc = loc_e;
            loc_cost = std::chrono::duration_cast<std::chrono::microseconds>(loc_e - loc_s).count() / 1000.0;
            ROS_INFO("localization cost: %f ms", loc_cost);
        }
    }
}

void GloabalLocalization::StartLoc()
{
    thread_loc_ = std::thread(&GloabalLocalization::Localization, this);
    if (map_switch_enable_ && maps_.size() > 1)
    {
        thread_monitor_ = std::thread(&GloabalLocalization::MapSwitchMonitor, this);
        ROS_WARN("MapSwitchMonitor started (period=%dms, verify_fitness=%.2f, consec=%d, icp_threshold=%.2f, max_delta=%.2fm/%.1fdeg)",
                 verify_period_ms_, verify_fitness_, verify_consecutive_,
                 verify_icp_threshold_, verify_max_translation_, verify_max_yaw_deg_);
    }
}

void GloabalLocalization::SwitchActiveMap(int new_idx, const std::string &reason,
                                          const Eigen::Matrix4d *verified_odom2map)
{
    if (new_idx < 0 || new_idx >= static_cast<int>(maps_.size()))
    {
        return;
    }
    int old_idx = active_map_idx_.load();
    if (new_idx == old_idx)
    {
        return;
    }

    uint64_t old_epoch = map_epoch_.load();
    std::string old_frame;
    Eigen::Matrix4d T_old_map_base_before;
    Eigen::Matrix4d T_old_world_base_before;
    bool used_verified_pose = false;

    {
        std::lock_guard<std::mutex> lk(lock_state_);
        old_frame = active_map_frame_;
        T_old_map_base_before = mat_odom2map_ * mat_baselink2odom_;
        T_old_world_base_before = mat_world_to_active_map_ * T_old_map_base_before;
    }

    bool need_reinit = false;
    Eigen::Matrix4d bl2m;
    Eigen::Matrix4d T_candidate_map_base_verified;

    {
        std::shared_lock<std::shared_mutex> rlock(lock_maps_);
        if (verified_odom2map != nullptr)
        {
            std::lock_guard<std::mutex> lk(lock_state_);
            mat_odom2map_ = *verified_odom2map;
            mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;
            bl2m = mat_baselink2map_;
            T_candidate_map_base_verified = bl2m;
            used_verified_pose = true;
        }
        else if (maps_[old_idx].next_initialpose_set)
        {
            auto &ip = maps_[old_idx].next_initialpose;
            Eigen::Matrix4d T_next_map_base = Eigen::Matrix4d::Identity();
            T_next_map_base.block<3, 1>(0, 3) = Eigen::Vector3d(ip[0], ip[1], ip[2]);
            T_next_map_base.block<3, 3>(0, 0) = Euler2Matrix3d(Eigen::Vector3d(ip[3], ip[4], ip[5]));
            {
                std::lock_guard<std::mutex> lk(lock_state_);
                mat_initialpose_ = T_next_map_base;
                mat_odom2map_ = T_next_map_base * mat_baselink2odom_.inverse();
                mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;
                bl2m = mat_baselink2map_;
                T_candidate_map_base_verified = bl2m;
            }
            need_reinit = true;
        }
    }

    // 方案 B: 计算 T_world_map_new 以保持 RViz 视觉连续
    // T_world_base_before = T_world_map_old * T_map_old_base_before
    // T_world_map_new = T_world_base_before * inverse(T_map_new_base_verified)
    Eigen::Matrix4d T_world_map_new = T_old_world_base_before * T_candidate_map_base_verified.inverse();

    std::string new_frame;
    {
        std::unique_lock<std::shared_mutex> wlock(lock_maps_);
        active_map_idx_.store(new_idx);
        pcd_map_coarse_ = maps_[new_idx].map_coarse;
        pcd_map_fine_ = maps_[new_idx].map_fine;
        new_frame = maps_[new_idx].frame_id;
    }
    {
        std::lock_guard<std::mutex> lk(lock_state_);
        active_map_frame_ = new_frame;
        mat_world_to_active_map_ = T_world_map_new;
    }

    // Increment epoch to invalidate any in-flight ICP results
    map_epoch_.store(old_epoch + 1);
    force_resubmap_.store(true);
    last_loc_ = Eigen::Vector3d(0, 0, -5000);

    // Publish new map pointcloud with candidate frame
    sensor_msgs::PointCloud2 pc2_map;
    open3d_conversions::open3dToRos(*maps_[new_idx].map_coarse, pc2_map);
    pc2_map.header.frame_id = new_frame;
    pc2_map.header.stamp = ros::Time::now();
    pub_map_.publish(pc2_map);

    std_msgs::String s;
    s.data = maps_[new_idx].name;
    pub_active_map_.publish(s);

    // Publish initial static TF for the new frame (will be overridden by dynamic TF in odom callback)
    {
        Eigen::Quaterniond q_w(T_world_map_new.block<3,3>(0,0));
        geometry_msgs::TransformStamped world_to_map;
        world_to_map.header.frame_id = "localization_world";
        world_to_map.child_frame_id = new_frame;
        world_to_map.transform.translation.x = T_world_map_new(0,3);
        world_to_map.transform.translation.y = T_world_map_new(1,3);
        world_to_map.transform.translation.z = T_world_map_new(2,3);
        world_to_map.transform.rotation.w = q_w.w();
        world_to_map.transform.rotation.x = q_w.x();
        world_to_map.transform.rotation.y = q_w.y();
        world_to_map.transform.rotation.z = q_w.z();
        static_broadcaster_.sendTransform(world_to_map);
    }

    // Comprehensive diagnostic logging
    {
        std::ostringstream os;
        os << "SWITCHED:" << maps_[old_idx].name << "->" << maps_[new_idx].name
           << " epoch=" << old_epoch << "->" << (old_epoch + 1)
           << " " << reason;
        std_msgs::String st;
        st.data = os.str();
        pub_status_.publish(st);
    }

    ROS_WARN("=== SwitchActiveMap: %s -> %s (frame=%s->%s, epoch=%lu->%lu, %s) ===",
             maps_[old_idx].name.c_str(), maps_[new_idx].name.c_str(),
             old_frame.c_str(), new_frame.c_str(),
             (unsigned long)old_epoch, (unsigned long)(old_epoch + 1), reason.c_str());
    ROS_WARN("  T_old_map_base_before: %s", MatToXYYaw(T_old_map_base_before).c_str());
    ROS_WARN("  T_candidate_map_base_verified: %s", MatToXYYaw(T_candidate_map_base_verified).c_str());
    ROS_WARN("  T_world_map_new: %s", MatToXYYaw(T_world_map_new).c_str());
    {
        Eigen::Matrix4d o2m;
        {
            std::lock_guard<std::mutex> lk(lock_state_);
            o2m = mat_odom2map_;
        }
        ROS_WARN("  T_candidate_map_odom_committed: %s", MatToXYYaw(o2m).c_str());
    }

    // Reset filters
    kf_baselink_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1], bl2m(0, 3), 1);
    kf_baselink_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1], bl2m(1, 3), 1);
    kf_baselink_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1], bl2m(2, 3), 1);
    kalman_filter_odom2map_.KalmanFilterInit(kalman_processVar2_, kalman_estimatedMeasVar2_, bl2m(2, 3), 1);

    if (used_verified_pose)
    {
        loc_initialized_ = true;
        ROS_WARN("SwitchActiveMap: verified switch on '%s' done", maps_[new_idx].name.c_str());
    }
    else if (need_reinit)
    {
        loc_initialized_ = false;
        LocalizationInitialize();
        {
            std::lock_guard<std::mutex> lk(lock_state_);
            bl2m = mat_odom2map_ * mat_baselink2odom_;
        }
        kf_baselink_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1], bl2m(0, 3), 1);
        kf_baselink_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1], bl2m(1, 3), 1);
        kf_baselink_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1], bl2m(2, 3), 1);
        kalman_filter_odom2map_.KalmanFilterInit(kalman_processVar2_, kalman_estimatedMeasVar2_, bl2m(2, 3), 1);
        loc_initialized_ = true;
        ROS_WARN("SwitchActiveMap: re-initialization on '%s' done", maps_[new_idx].name.c_str());
    }
}

void GloabalLocalization::MapSwitchMonitor()
{
    open3d::geometry::OrientedBoundingBox OBB_map;
    open3d::geometry::OrientedBoundingBox OBB_scan;
    OBB_map.extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan.extent_ = Eigen::Vector3d(60, 60, 40);

    int last_candidate = -1;
    int consecutive_success = 0;
    auto cooldown_until = std::chrono::steady_clock::now();

    const int max_src = std::max(10000, maxpoints_source_ / 2);
    const int max_tgt = std::max(40000, maxpoints_target_ / 2);

    // 保存验证窗口内最优候选
    struct CandidateResult {
        double fitness = 0.0;
        double delta_xy = 1e9;
        double delta_yaw = 1e9;
        Eigen::Matrix4d odom2map = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d baselink2cand_map = Eigen::Matrix4d::Identity();
    };
    CandidateResult best_candidate;

    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lk(lock_exit_);
            if (flag_exit_) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(verify_period_ms_));

        if (!loc_initialized_) continue;
        if (std::chrono::steady_clock::now() < cooldown_until) continue;

        // 1) snapshot 当前位姿
        Eigen::Matrix4d baselink2map_snap;
        Eigen::Matrix4d baselink2odom_snap;
        Eigen::Matrix4d odom2map_snap;
        Eigen::Vector3d xyz;
        {
            std::lock_guard<std::mutex> lk(lock_state_);
            baselink2map_snap = mat_baselink2map_;
            baselink2odom_snap = mat_baselink2odom_;
            odom2map_snap = mat_odom2map_;
            xyz = baselink2map_snap.block<3, 1>(0, 3);
        }

        // 2) 只有进入当前地图 switch_zone 后,才开始静默验证下一张地图。
        int active = active_map_idx_.load();
        bool in_zone = false;
        int cand = -1;
        Eigen::Matrix4d cand_init = odom2map_snap;
        std::string active_name;
        std::string cand_name;
        bool cand_calibrated = false;
        {
            std::shared_lock<std::shared_mutex> rlock(lock_maps_);
            active_name = maps_[active].name;
            in_zone = PoseInSwitchZone(xyz, maps_[active]);
            if (in_zone && active + 1 < static_cast<int>(maps_.size()))
            {
                cand = active + 1;
                cand_name = maps_[cand].name;
                cand_calibrated = maps_[active].next_initialpose_calibrated || allow_uncalibrated_switch_;
                Eigen::Matrix4d T_cand_map_base = Eigen::Matrix4d::Identity();
                if (maps_[active].next_initialpose_set)
                {
                    const auto &ip = maps_[active].next_initialpose;
                    T_cand_map_base.block<3, 1>(0, 3) = Eigen::Vector3d(ip[0], ip[1], ip[2]);
                    T_cand_map_base.block<3, 3>(0, 0) = Euler2Matrix3d(Eigen::Vector3d(ip[3], ip[4], ip[5]));
                }
                else if (maps_[cand].initialpose_set)
                {
                    const auto &ip = maps_[cand].initialpose;
                    T_cand_map_base.block<3, 1>(0, 3) = Eigen::Vector3d(ip[0], ip[1], ip[2]);
                    T_cand_map_base.block<3, 3>(0, 0) = Euler2Matrix3d(Eigen::Vector3d(ip[3], ip[4], ip[5]));
                }
                cand_init = T_cand_map_base * baselink2odom_snap.inverse();
            }
        }
        candidate_map_idx_.store(cand);
        if (cand < 0)
        {
            last_candidate = -1;
            consecutive_success = 0;
            continue;
        }
        if (cand != last_candidate)
        {
            last_candidate = cand;
            consecutive_success = 0;
            best_candidate = CandidateResult();
            if (!cand_calibrated)
            {
                ROS_WARN("[MapSwitchMonitor] entered switch_zone on '%s', candidate '%s' NEED_CALIBRATION "
                         "(next_initialpose is all zeros / not calibrated)",
                         active_name.c_str(), cand_name.c_str());
                std_msgs::String st;
                st.data = "NEED_CALIBRATION:" + cand_name +
                          " reason=bad_initialpose_or_missing_entry_pose";
                pub_status_.publish(st);
            }
            else
            {
                ROS_WARN("[MapSwitchMonitor] entered switch_zone on '%s', start silent verify '%s'",
                         active_name.c_str(), cand_name.c_str());
            }
        }

        // 即使未标定也继续做 VERIFYING 以便用户看到 fitness, 但不允许自动提交切换
        Eigen::Matrix4d expected_baselink2cand_map = cand_init * baselink2odom_snap;

        // 3) 在候选地图坐标下裁剪 target
        std::shared_ptr<open3d::geometry::PointCloud> target;
        {
            std::shared_lock<std::shared_mutex> rlock(lock_maps_);
            OBB_map.center_ = expected_baselink2cand_map.block<3, 1>(0, 3);
            OBB_map.R_ = expected_baselink2cand_map.block<3, 3>(0, 0);
            target = maps_[cand].map_fine->Crop(OBB_map);
        }
        if (target->points_.size() < 1000)
        {
            consecutive_success = 0;
            ROS_WARN("[MapSwitchMonitor] candidate=%s target too small (%zu pts), reset streak",
                     cand_name.c_str(), target->points_.size());
            continue;
        }
        if (static_cast<int>(target->points_.size()) > max_tgt)
        {
            target = target->RandomDownSample(double(max_tgt) / target->points_.size());
        }

        // 4) 取当前 scan,仍然在 odom 坐标中裁剪。
        std::shared_ptr<open3d::geometry::PointCloud> source(new open3d::geometry::PointCloud);
        {
            std::lock_guard<std::mutex> lk(lock_scan_);
            if (pcd_scan_cur_->IsEmpty()) continue;
            OBB_scan.center_ = baselink2odom_snap.block<3, 1>(0, 3);
            OBB_scan.R_ = baselink2odom_snap.block<3, 3>(0, 0);
            source = pcd_scan_cur_->Crop(OBB_scan);
        }
        source = source->VoxelDownSample(voxelsize_fine_);
        if (source->points_.size() < 1000)
        {
            consecutive_success = 0;
            continue;
        }
        if (static_cast<int>(source->points_.size()) > max_src)
        {
            source = source->RandomDownSample(double(max_src) / source->points_.size());
        }

        // 5) 用候选地图做一次静默定位
        auto reg_result = pcd_tools::RegistrationIcp(
            source, target, verify_icp_threshold_, cand_init, 1, 30);
        Eigen::Matrix4d verified_odom2map = reg_result.transformation_ * cand_init;
        Eigen::Matrix4d verified_baselink2cand_map = verified_odom2map * baselink2odom_snap;
        auto eva = open3d::pipelines::registration::EvaluateRegistration(
            *source, *target, voxelsize_fine_ * 4, verified_odom2map);
        double fit = eva.fitness_;
        double delta_xy = PoseDistanceXY(verified_baselink2cand_map, expected_baselink2cand_map);
        double delta_yaw = PoseYawDiffDeg(verified_baselink2cand_map, expected_baselink2cand_map);
        bool pose_delta_ok = delta_xy <= verify_max_translation_ &&
                             delta_yaw <= verify_max_yaw_deg_;

        // Update best candidate in verification window
        if (fit > best_candidate.fitness && pose_delta_ok)
        {
            best_candidate.fitness = fit;
            best_candidate.delta_xy = delta_xy;
            best_candidate.delta_yaw = delta_yaw;
            best_candidate.odom2map = verified_odom2map;
            best_candidate.baselink2cand_map = verified_baselink2cand_map;
        }

        {
            std_msgs::String st;
            std::ostringstream os;
            os << "VERIFYING:" << cand_name << " fit=" << std::fixed << std::setprecision(3) << fit
               << " guess_base=" << MatToXYYaw(expected_baselink2cand_map)
               << " verified_base=" << MatToXYYaw(verified_baselink2cand_map)
               << " dxy=" << std::setprecision(2) << delta_xy
               << " dyaw=" << std::setprecision(1) << delta_yaw
               << " streak=" << consecutive_success << "/" << verify_consecutive_;
            if (!pose_delta_ok)
            {
                os << " rejected=pose_delta";
            }
            if (!cand_calibrated)
            {
                os << " NEED_CALIBRATION";
            }
            st.data = os.str();
            pub_status_.publish(st);
        }
        ROS_INFO("[MapSwitchMonitor] candidate=%s fitness=%.3f (need >%.2f), delta=%.2fm/%.1fdeg (max %.2fm/%.1fdeg), streak %d/%d, calibrated=%s",
                 cand_name.c_str(), fit, verify_fitness_, delta_xy, delta_yaw,
                 verify_max_translation_, verify_max_yaw_deg_,
                 consecutive_success, verify_consecutive_,
                 cand_calibrated ? "true" : "false");

        if (fit > verify_fitness_ && pose_delta_ok)
        {
            consecutive_success += 1;
            if (consecutive_success >= verify_consecutive_)
            {
                // 未标定: 不允许自动提交
                if (!cand_calibrated)
                {
                    ROS_WARN("[MapSwitchMonitor] candidate=%s reached consecutive threshold but NEED_CALIBRATION, not committing",
                             cand_name.c_str());
                    std_msgs::String st;
                    st.data = "REJECTED:" + cand_name + " reason=bad_initialpose_or_missing_entry_pose";
                    pub_status_.publish(st);
                    cooldown_until = std::chrono::steady_clock::now() +
                                     std::chrono::milliseconds(cooldown_ms_);
                    continue;
                }

                // ===== PRECOMMIT =====
                ROS_WARN("[MapSwitchMonitor] PRECOMMIT: verifying best candidate for '%s' "
                         "(best_fit=%.3f, best_dxy=%.2f, best_dyaw=%.1f)",
                         cand_name.c_str(), best_candidate.fitness,
                         best_candidate.delta_xy, best_candidate.delta_yaw);

                Eigen::Matrix4d best_baselink2cand = best_candidate.baselink2cand_map;
                open3d::geometry::OrientedBoundingBox OBB_precommit;
                OBB_precommit.extent_ = Eigen::Vector3d(60, 60, 40);
                OBB_precommit.center_ = best_baselink2cand.block<3, 1>(0, 3);
                OBB_precommit.R_ = best_baselink2cand.block<3, 3>(0, 0);

                std::shared_ptr<open3d::geometry::PointCloud> precommit_target;
                {
                    std::shared_lock<std::shared_mutex> rlock(lock_maps_);
                    precommit_target = maps_[cand].map_fine->Crop(OBB_precommit);
                }
                if (precommit_target->points_.size() > maxpoints_target_)
                {
                    precommit_target = precommit_target->RandomDownSample(
                        double(maxpoints_target_) / precommit_target->points_.size());
                }

                std::shared_ptr<open3d::geometry::PointCloud> precommit_source(new open3d::geometry::PointCloud);
                {
                    std::lock_guard<std::mutex> lk(lock_scan_);
                    if (!pcd_scan_cur_->IsEmpty())
                    {
                        OBB_scan.center_ = baselink2odom_snap.block<3, 1>(0, 3);
                        OBB_scan.R_ = baselink2odom_snap.block<3, 3>(0, 0);
                        *precommit_source = *pcd_scan_cur_->Crop(OBB_scan);
                    }
                }
                precommit_source = precommit_source->VoxelDownSample(voxelsize_fine_);
                if (precommit_source->points_.size() > maxpoints_source_)
                {
                    precommit_source = precommit_source->RandomDownSample(
                        double(maxpoints_source_) / precommit_source->points_.size());
                }

                // 主定位路径验证
                double precommit_fitness = 0.0;
                bool precommit_ok = false;
                if (precommit_source->points_.size() > 1000 && precommit_target->points_.size() > 1000)
                {
                    auto precommit_reg = pcd_tools::RegistrationIcp(
                        precommit_source, precommit_target, voxelsize_fine_ * 2,
                        best_candidate.odom2map, 1);
                    Eigen::Matrix4d precommit_odom2map = precommit_reg.transformation_ * best_candidate.odom2map;
                    auto precommit_eva = open3d::pipelines::registration::EvaluateRegistration(
                        *precommit_source, *precommit_target, voxelsize_fine_ * 4, precommit_odom2map);
                    precommit_fitness = precommit_eva.fitness_;

                    Eigen::Matrix4d precommit_baselink2cand = precommit_odom2map * baselink2odom_snap;
                    double precommit_delta_xy = PoseDistanceXY(precommit_baselink2cand, expected_baselink2cand_map);
                    double precommit_delta_yaw = PoseYawDiffDeg(precommit_baselink2cand, expected_baselink2cand_map);

                    ROS_WARN("[MapSwitchMonitor] PRECOMMIT: fitness=%.3f (need >%.2f), "
                             "delta=%.2fm/%.1fdeg from guess",
                             precommit_fitness, post_switch_min_fitness_,
                             precommit_delta_xy, precommit_delta_yaw);

                    {
                        std_msgs::String st;
                        std::ostringstream os;
                        os << "PRECOMMIT:" << cand_name
                           << " fit=" << std::fixed << std::setprecision(3) << precommit_fitness
                           << " dxy=" << std::setprecision(2) << precommit_delta_xy
                           << " dyaw=" << std::setprecision(1) << precommit_delta_yaw;
                        st.data = os.str();
                        pub_status_.publish(st);
                    }

                    precommit_ok = (precommit_fitness >= post_switch_min_fitness_);
                }
                else
                {
                    ROS_WARN("[MapSwitchMonitor] PRECOMMIT: insufficient points (src=%zu, tgt=%zu)",
                             precommit_source->points_.size(), precommit_target->points_.size());
                }

                if (!precommit_ok)
                {
                    ROS_WARN("[MapSwitchMonitor] PRECOMMIT FAILED for '%s': fitness=%.3f < %.2f, not committing",
                             cand_name.c_str(), precommit_fitness, post_switch_min_fitness_);
                    std_msgs::String st;
                    st.data = "REJECTED:" + cand_name + " reason=bad_precommit";
                    pub_status_.publish(st);
                    consecutive_success = 0;
                    best_candidate = CandidateResult();
                    cooldown_until = std::chrono::steady_clock::now() +
                                     std::chrono::milliseconds(cooldown_ms_);
                    continue;
                }

                // ===== COMMIT =====
                std::ostringstream reason;
                reason << "verified fit=" << std::fixed << std::setprecision(3) << best_candidate.fitness
                       << " precommit_fit=" << std::setprecision(3) << precommit_fitness
                       << " streak=" << verify_consecutive_;
                loc_initialized_ = false;
                SwitchActiveMap(cand, reason.str(), &best_candidate.odom2map);
                consecutive_success = 0;
                best_candidate = CandidateResult();
                last_candidate = -1;
                cooldown_until = std::chrono::steady_clock::now() +
                                 std::chrono::milliseconds(cooldown_ms_);
            }
        }
        else
        {
            consecutive_success = 0;
        }
    }
}

void GloabalLocalization::CallbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initialpose)
{
    // RViz 的 /initialpose 表示机器人在 header.frame_id 坐标系下的位姿
    // 当 Fixed Frame 是 localization_world 时, frame_id = "localization_world"
    // 当 Fixed Frame 是 map_xxx 时, frame_id = "map_xxx"
    // 需要统一转到 active map frame 后再计算 T_map_odom

    Eigen::Matrix4d T_frame_base = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond rotation_q;
    rotation_q.w() = initialpose->pose.pose.orientation.w;
    rotation_q.x() = initialpose->pose.pose.orientation.x;
    rotation_q.y() = initialpose->pose.pose.orientation.y;
    rotation_q.z() = initialpose->pose.pose.orientation.z;
    T_frame_base.block<3, 3>(0, 0) = rotation_q.matrix();
    T_frame_base.block<3, 1>(0, 3) = Eigen::Vector3d(
        initialpose->pose.pose.position.x,
        initialpose->pose.pose.position.y,
        initialpose->pose.pose.position.z);

    std::string pose_frame = initialpose->header.frame_id;
    std::string active_frame;
    Eigen::Matrix4d T_world_to_active;
    {
        std::lock_guard<std::mutex> lk(lock_state_);
        active_frame = active_map_frame_;
        T_world_to_active = mat_world_to_active_map_;
    }

    Eigen::Matrix4d T_map_base;
    if (pose_frame == active_frame || pose_frame.empty())
    {
        // Pose 已经在 active map frame 下
        T_map_base = T_frame_base;
    }
    else if (pose_frame == "localization_world")
    {
        // Pose 在 localization_world 下, 需要转到 active map frame
        // T_map_base = T_map_world * T_world_base
        // T_map_world = inverse(T_world_map)
        T_map_base = T_world_to_active.inverse() * T_frame_base;
        ROS_WARN("CallbackInitialPose: converted from localization_world to %s", active_frame.c_str());
    }
    else
    {
        // 其他 frame: 尝试用 TF 转换, 失败则直接使用
        T_map_base = T_frame_base;
        ROS_WARN("CallbackInitialPose: unexpected frame_id='%s', using pose directly", pose_frame.c_str());
    }

    Eigen::Matrix4d T_odom_base;
    Eigen::Matrix4d bl2m;
    {
        std::lock_guard<std::mutex> lk(lock_state_);
        T_odom_base = mat_baselink2odom_;
        Eigen::Matrix4d T_map_odom = T_map_base * T_odom_base.inverse();
        mat_initialpose_ = T_map_base;
        mat_odom2map_ = T_map_odom;
        mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;
        bl2m = mat_baselink2map_;
    }

    // Increment epoch so in-flight ICP results are discarded
    map_epoch_.store(map_epoch_.load() + 1);

    kf_baselink_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1], bl2m(0, 3), 1);
    kf_baselink_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1], bl2m(1, 3), 1);
    kf_baselink_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1], bl2m(2, 3), 1);
    kalman_filter_odom2map_.KalmanFilterInit(kalman_processVar2_, kalman_estimatedMeasVar2_, bl2m(2, 3), 1);

    force_resubmap_.store(true);
    last_loc_ = Eigen::Vector3d(0, 0, -5000);
    manual_reinit_pending_.store(true);

    std_msgs::String st;
    st.data = "MANUAL_INITIALPOSE";
    pub_status_.publish(st);

    ROS_WARN("CallbackInitialPose: T_map_base=%s, T_map_odom = T_map_base * inv(T_odom_base), "
             "epoch=%lu, manual_reinit_pending=true",
             MatToXYYaw(T_map_base).c_str(), (unsigned long)map_epoch_.load());
}
double GloabalLocalization::ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2) + std::pow(a.z() - b.z(), 2));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "global_loc_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // global_odom.
    std::cout << "start spin" << std::endl;
    // 创建异步对象
    ros::AsyncSpinner spinner(3);
    // 开始异步处理
    spinner.start();

    GloabalLocalization global_loc(nh, nh_private);
    global_loc.StartLoc();

    // 等待节点关闭
    ros::waitForShutdown();

    return 0;
}
