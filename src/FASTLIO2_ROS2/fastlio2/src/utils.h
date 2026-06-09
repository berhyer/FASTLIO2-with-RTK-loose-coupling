#pragma once
#include <iomanip>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <pcl_conversions/pcl_conversions.h>

#define RESET "\033[0m"
#define BLACK "\033[30m"  /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"  /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define PURPLE "\033[35m" /* Purple */
#define CYAN "\033[36m"   /* Cyan */
#define WHITE "\033[37m"  /* White */
struct OusterPoint {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;     // 纳秒偏移
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPoint,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) 
)

// --- 新增: Velodyne 专属点云格式注册 ---
struct VelodynePoint {
    PCL_ADD_POINT4D;
    float intensity;
    float time;     // Velodyne 的时间戳是 float，单位是秒
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePoint,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, time, time) (uint16_t, ring, ring)
)
class Utils {
public:
    static double getSec(std_msgs::msg::Header &header);
    static builtin_interfaces::msg::Time getTime(const double& sec);
    // 新增 Ouster 解析函数
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr ouster2PCL(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
        int filter_num, double min_range = 0.5, double max_range = 20.0);
    // 保留旧函数避免报错
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range = 0.5, double max_range = 20.0);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr commonLidar2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range = 0.5, double max_range = 20.0);
};
