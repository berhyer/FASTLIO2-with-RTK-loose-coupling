#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <filesystem>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "interface/srv/save_maps.hpp"
#include <pcl/io/io.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace std::chrono_literals;


struct NodeConfig
{
    std::string cloud_topic = "/lio/body_cloud";
    std::string odom_topic = "/lio/odom";
    std::string rtk_topic = "/ublox/fix";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
};

struct NodeState
{
    std::mutex message_mutex;
    std::queue<CloudWithPose> cloud_buffer;
    double last_message_time;
    // --- 新增: RTK 的数据队列和线程锁 ---
    std::mutex rtk_mutex;
    std::queue<RTKData> rtk_buffer;

    // --- 新增: GeographicLib 转换器与原点标志位 ---
    GeographicLib::LocalCartesian geo_converter;
    bool rtk_initialized = false;
    // --- 新增：用于动态缝合的变量 ---
    bool has_lio_odom = false;
    double current_lio_x = 0.0;
    double current_lio_y = 0.0;
    double current_lio_z = 0.0;
    
    double rtk_offset_x = 0.0;
    double rtk_offset_y = 0.0;
    double rtk_offset_z = 0.0;
};

class PGONode : public rclcpp::Node
{
public:
    PGONode() : Node("pgo_node")
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started");
        loadParameters();
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pgo/loop_markers", 10000);

        m_rtk_path_pub = this->create_publisher<nav_msgs::msg::Path>("/pgo/rtk_path", 10000);
        m_optimized_path_pub = this->create_publisher<nav_msgs::msg::Path>("/pgo/optimized_path", 10000);
        m_rtk_path.header.frame_id = m_node_config.map_frame; // 保持与全局地图坐标系一致
        m_rtk_path.poses.clear();
        m_rtk_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(m_node_config.rtk_topic, rclcpp::SensorDataQoS(), std::bind(&PGONode::rtkCB, this, std::placeholders::_1)); //topic name  change

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>("/pgo/save_maps", std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
        m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_node_config.odom_topic = config["odom_topic"].as<std::string>();
        m_node_config.rtk_topic = config["rtk_topic"].as<std::string>();

        m_node_config.map_frame = config["map_frame"].as<std::string>();
        m_node_config.local_frame = config["local_frame"].as<std::string>();

        m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
        m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
        m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
        m_pgo_config.loop_time_tresh = config["loop_time_tresh"].as<double>();
        m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
        m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
        m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
        m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();
        m_pgo_config.rtk_outlier_threshold = config["rtk_outlier_threshold"].as<double>();
        
        m_pgo_config.rtk_cauchy_scale = config["rtk_cauchy_scale"].as<double>();
        std::vector<double> ext_t = config["ext_t"].as<std::vector<double>>();
        m_pgo_config.ext_x = ext_t[0];
        m_pgo_config.ext_y = ext_t[1];
        m_pgo_config.ext_z = ext_t[2];
        m_pgo_config.yaw_offset_deg = config["yaw_offset_deg"].as<double>();
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex>(m_state.message_mutex);
        m_state.current_lio_x = odom_msg->pose.pose.position.x;
        m_state.current_lio_y = odom_msg->pose.pose.position.y;
        m_state.current_lio_z = odom_msg->pose.pose.position.z;
        m_state.has_lio_odom = true;

        CloudWithPose cp;
        cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
        if (cp.pose.second < m_state.last_message_time)
        {
            RCLCPP_WARN(this->get_logger(), "Received out of order message");
            return;
        }
        m_state.last_message_time = cp.pose.second;

        cp.pose.r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                       odom_msg->pose.pose.orientation.x,
                                       odom_msg->pose.pose.orientation.y,
                                       odom_msg->pose.pose.orientation.z)
                        .toRotationMatrix();
        cp.pose.t = V3D(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
        cp.cloud = CloudType::Ptr(new CloudType);
        pcl::fromROSMsg(*cloud_msg, *cp.cloud);
        m_state.cloud_buffer.push(cp);
    }
    void rtkCB(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg)
    {
        // 过滤掉无效的 RTK 数据（比如没有 Fixed 解）
        // msg->status.status == -1 表示 NO_FIX
        if (msg->status.status < 0) {
            return;
        }
        static int rtk_msg_count = 0;
        rtk_msg_count++;
        if (rtk_msg_count < 20) {
            return;
        }
        // 2. 初始化局部 ENU 坐标系原点 (Datum)
        if (!m_state.rtk_initialized) {
            if (!m_state.has_lio_odom) {
                RCLCPP_WARN(this->get_logger(), "Waiting for LIO Odometry to stitch RTK...");
                return;
            }
            // 将稳定后的这一帧设为 RTK 局部 ENU 坐标系的原点 (0,0,0)
            m_state.geo_converter.Reset(msg->latitude, msg->longitude, msg->altitude);
            
            // 🌟 记录这 20 帧期间 LIO 已经“偷跑”的绝对距离
            m_state.rtk_offset_x = m_state.current_lio_x;
            m_state.rtk_offset_y = m_state.current_lio_y;
            m_state.rtk_offset_z = m_state.current_lio_z;
    
            m_state.rtk_initialized = true;
            RCLCPP_INFO(this->get_logger(), "RTK Stitched! LIO Offset applied: [%.2f, %.2f, %.2f]", 
                        m_state.rtk_offset_x, m_state.rtk_offset_y, m_state.rtk_offset_z);
            return; // 第一帧通常作为原点丢弃，不加入优化
        }
       
        // 4. 日常方差质检 (阈值放宽到 15.0，过滤严重飞点，放行 8~9 的普通数据)
        double accuracy_variance = msg->position_covariance[0];
        if (accuracy_variance > 15.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Signal degraded! Variance: %.2f. Dropping RTK frame.", accuracy_variance);
            return;
        }
        // 3. 实例化轻量级结构体
        RTKData rtk_data;

        // 转换 ROS2 时间戳为 double 秒
        rtk_data.time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        double raw_x ,raw_y,raw_z;
        // 4. 经纬度 -> 局部 ENU 笛卡尔坐标转换
        m_state.geo_converter.Forward(msg->latitude, msg->longitude, msg->altitude, 
                                      raw_x, raw_y, raw_z);

        double yaw_rad = m_pgo_config.yaw_offset_deg * M_PI / 180.0;
        double rotated_x = raw_x * cos(yaw_rad) + raw_y * sin(yaw_rad);
        double rotated_y = -raw_x * sin(yaw_rad) + raw_y * cos(yaw_rad);
        
        rtk_data.enu_x = rotated_x + m_state.rtk_offset_x;
        rtk_data.enu_y = rotated_y + m_state.rtk_offset_y;
        rtk_data.enu_z = m_state.current_lio_z;

        //RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"yaw para:%.1f|raw xy:[%.1f,%.1f]->rotated:[%.1f,%.1f]",m_pgo_config.yaw_offset_deg,raw_x,raw_y,rotated_x,rotated_y);
        // 5. 将 9 维一维数组 (Row-major) 映射为 Eigen::Matrix3d
        // NavSatFix 的 covariance 是按 [row0, row1, row2] 存储的
        rtk_data.covariance << msg->position_covariance[0], msg->position_covariance[1], msg->position_covariance[2],
                               msg->position_covariance[3], msg->position_covariance[4], msg->position_covariance[5],
                               msg->position_covariance[6], msg->position_covariance[7], msg->position_covariance[8];
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = m_node_config.map_frame; // 必须是全局坐标系
        pose.header.stamp = msg->header.stamp;
        
        // 设置平移
        pose.pose.position.x = rtk_data.enu_x;
        pose.pose.position.y = rtk_data.enu_y;
        pose.pose.position.z = rtk_data.enu_z;
        // RTK 只提供位置，没有姿态，所以给定一个单位四元数（无旋转）
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        // 将当前点加入路径数组
        m_rtk_path.poses.push_back(pose);
        m_rtk_path.header.stamp = msg->header.stamp; // 更新整条路径的时间戳
        // 如果 RViz 正在订阅，则发布
        if (m_rtk_path_pub->get_subscription_count() > 0) {
            m_rtk_path_pub->publish(m_rtk_path);
        }
        std::lock_guard<std::mutex> lock(m_state.rtk_mutex);
        m_state.rtk_buffer.push(rtk_data);
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_node_config.map_frame;
        transformStamped.child_frame_id = m_node_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_pgo->offsetR());
        V3D t = m_pgo->offsetT();
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void publishLoopMarkers(builtin_interfaces::msg::Time &time)
    {
        if (m_loop_marker_pub->get_subscription_count() == 0)
            return;
        if (m_pgo->historyPairs().size() == 0)
            return;

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker;
        visualization_msgs::msg::Marker edges_marker;
        nodes_marker.header.frame_id = m_node_config.map_frame;
        nodes_marker.header.stamp = time;
        nodes_marker.ns = "pgo_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        edges_marker.header.frame_id = m_node_config.map_frame;
        edges_marker.header.stamp = time;
        edges_marker.ns = "pgo_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;

        std::vector<KeyPoseWithCloud> &poses = m_pgo->keyPoses();
        std::vector<std::pair<size_t, size_t>> &pairs = m_pgo->historyPairs();
        for (size_t i = 0; i < pairs.size(); i++)
        {
            size_t i1 = pairs[i].first;
            size_t i2 = pairs[i].second;
            geometry_msgs::msg::Point p1, p2;
            p1.x = poses[i1].t_global.x();
            p1.y = poses[i1].t_global.y();
            p1.z = poses[i1].t_global.z();

            p2.x = poses[i2].t_global.x();
            p2.y = poses[i2].t_global.y();
            p2.z = poses[i2].t_global.z();

            nodes_marker.points.push_back(p1);
            nodes_marker.points.push_back(p2);
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }

        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        m_loop_marker_pub->publish(marker_array);
    }
    void publishOptimizedPath(builtin_interfaces::msg::Time &time)
    {
        if (m_optimized_path_pub->get_subscription_count() == 0) return;
        
        nav_msgs::msg::Path path;
        path.header.frame_id = m_node_config.map_frame;
        path.header.stamp = time;
        
        auto &poses = m_pgo->keyPoses();
        for (size_t i = 0; i < poses.size(); i++)
        {
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = m_node_config.map_frame;
            
            // =========================================================
            // 🌟 核心修复：将 simple_pgo.h 中存的 double time 转换为真实 ROS 绝对时间戳
            // =========================================================
            double t = poses[i].time; 
            p.header.stamp.sec = static_cast<int32_t>(t);
            p.header.stamp.nanosec = static_cast<uint32_t>((t - p.header.stamp.sec) * 1e9);
            // =========================================================

            p.pose.position.x = poses[i].t_global.x();
            p.pose.position.y = poses[i].t_global.y();
            p.pose.position.z = poses[i].t_global.z();
            
            Eigen::Quaterniond q(poses[i].r_global);
            p.pose.orientation.w = q.w();
            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            
            path.poses.push_back(p);
        }
        m_optimized_path_pub->publish(path);
    }
    void timerCB()
    {
        if (m_state.cloud_buffer.size() == 0)
            return;
        CloudWithPose cp = m_state.cloud_buffer.front();
        // 清理队列
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            while (!m_state.cloud_buffer.empty())
            {
                m_state.cloud_buffer.pop();
            }
        }

        RTKData* matched_rtk = nullptr;
        double lidar_time = cp.pose.second;

        {
            std::lock_guard<std::mutex> lock(m_state.rtk_mutex);
            // 1. 丢弃过时的 RTK 数据 (比如比当前 Lidar 帧早 0.1 秒以上的数据)
            while (!m_state.rtk_buffer.empty() && m_state.rtk_buffer.front().time < lidar_time - 0.2) {
                m_state.rtk_buffer.pop();
            }
            // 2. 检查队列首部的 RTK 是否与当前 Lidar 帧时间戳足够接近 (例如 0.05 秒以内)
            if (!m_state.rtk_buffer.empty() && std::abs(m_state.rtk_buffer.front().time - lidar_time) < 0.1) {
                matched_rtk = &m_state.rtk_buffer.front();
            }
        }

        builtin_interfaces::msg::Time cur_time;
        cur_time.sec = cp.pose.sec;
        cur_time.nanosec = cp.pose.nsec;
        if (!m_pgo->addKeyPose(cp,matched_rtk))
        {

            sendBroadCastTF(cur_time);
            return;
        }

        m_pgo->searchForLoopPairs();

        m_pgo->smoothAndUpdate();

        sendBroadCastTF(cur_time);

        publishLoopMarkers(cur_time);
        publishOptimizedPath(cur_time);
    }

    void saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request, std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        if (!std::filesystem::exists(request->file_path))
        {
            response->success = false;
            response->message = request->file_path + " IS NOT EXISTS!";
            return;
        }

        if (m_pgo->keyPoses().size() == 0)
        {
            response->success = false;
            response->message = "NO POSES!";
            return;
        }

        std::filesystem::path p_dir(request->file_path);
        std::filesystem::path patches_dir = p_dir / "patches";
        std::filesystem::path poses_txt_path = p_dir / "poses.txt";
        std::filesystem::path map_path = p_dir / "map.pcd";

        if (request->save_patches)
        {
            if (std::filesystem::exists(patches_dir))
            {
                std::filesystem::remove_all(patches_dir);
            }

            std::filesystem::create_directories(patches_dir);

            if (std::filesystem::exists(poses_txt_path))
            {
                std::filesystem::remove(poses_txt_path);
            }
            RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
        }
        RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

        std::ofstream txt_file(poses_txt_path);

        CloudType::Ptr ret(new CloudType);
        for (size_t i = 0; i < m_pgo->keyPoses().size(); i++)
        {

            CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;
            if (request->save_patches)
            {
                std::string patch_name = std::to_string(i) + ".pcd";
                std::filesystem::path patch_path = patches_dir / patch_name;
                pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);
                Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
                V3D t = m_pgo->keyPoses()[i].t_global;
                txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
            }
            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(*body_cloud, *world_cloud, m_pgo->keyPoses()[i].t_global, Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));
            *ret += *world_cloud;
        }
        txt_file.close();
        pcl::io::savePCDFileBinary(map_path.string(), *ret);
        response->success = true;
        response->message = "SAVE SUCCESS!";
    }

private:
    NodeConfig m_node_config;
    Config m_pgo_config;
    NodeState m_state;
    std::shared_ptr<SimplePGO> m_pgo;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_rtk_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_rtk_path_pub;
    nav_msgs::msg::Path m_rtk_path;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_optimized_path_pub;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PGONode>());
    rclcpp::shutdown();
    return 0;
}
