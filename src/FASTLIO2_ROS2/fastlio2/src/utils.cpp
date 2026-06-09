#include "utils.h"
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = msg->point_num;
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::ouster2PCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<OusterPoint> temp_cloud;
    pcl::fromROSMsg(*msg, temp_cloud); // 现在字段完全匹配，不会再报错

    cloud->reserve(temp_cloud.size() / filter_num);
    for (size_t i = 0; i < temp_cloud.size(); i += filter_num) {
        auto &p_s = temp_cloud.points[i];
        float d2 = p_s.x*p_s.x + p_s.y*p_s.y + p_s.z*p_s.z;
        if (d2 < min_range*min_range || d2 > max_range*max_range) continue;

        pcl::PointXYZINormal p_d;
        p_d.x = p_s.x; p_d.y = p_s.y; p_d.z = p_s.z;
        p_d.intensity = p_s.intensity;
        
        // 将纳秒偏移转为毫秒存入 curvature，供后端去畸变使用
        p_d.curvature = static_cast<float>(p_s.t) / 1e6f; 
        cloud->push_back(p_d);
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::commonLidar2PCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    // 1. 嗅探检查关键的时间戳字段，判断雷达厂家类型
    bool is_velodyne = false;
    bool is_ouster = false;
    
    for (const auto& field : msg->fields) {
        if (field.name == "time")        is_velodyne = true;
        if (field.name == "t")           is_ouster = true;
    }

    // 2. 根据雷达类型进行多路分流处理
    if (is_velodyne) {
        // ==========================================
        // 分流一：Velodyne 雷达处理逻辑 (如 M2DGR、UrbanNav 数据集)
        // ==========================================
        pcl::PointCloud<VelodynePoint> temp_cloud;
        pcl::fromROSMsg(*msg, temp_cloud);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        cloud->reserve(temp_cloud.size() / filter_num);

        for (size_t i = 0; i < temp_cloud.size(); i += filter_num) {
            auto &p_s = temp_cloud.points[i];
            float d2 = p_s.x*p_s.x + p_s.y*p_s.y + p_s.z*p_s.z;
            if (d2 < min_range*min_range || d2 > max_range*max_range) continue;

            pcl::PointXYZINormal p_d;
            p_d.x = p_s.x; p_d.y = p_s.y; p_d.z = p_s.z;
            p_d.intensity = p_s.intensity;
            p_d.curvature = p_s.time * 1000.0f; // 秒 -> 毫秒
            cloud->push_back(p_d);
        }
        return cloud;

    } else if (is_ouster) {
        // ==========================================
        // 分流二：Ouster 雷达处理逻辑
        // 直接复用你之前写好的、完美的 ouster2PCL 函数！
        // ==========================================
        return ouster2PCL(msg, filter_num, min_range, max_range);

    }else {
        // ==========================================
        // 分流四：安全降级兜底 (不带任何点级时间戳的通用点云)
        // ==========================================
        pcl::PointCloud<pcl::PointXYZI> temp_cloud;
        pcl::fromROSMsg(*msg, temp_cloud);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        cloud->reserve(temp_cloud.size() / filter_num);

        for (size_t i = 0; i < temp_cloud.size(); i += filter_num) {
            auto &p_s = temp_cloud.points[i];
            float d2 = p_s.x*p_s.x + p_s.y*p_s.y + p_s.z*p_s.z;
            if (d2 < min_range*min_range || d2 > max_range*max_range) continue;

            pcl::PointXYZINormal p_d;
            p_d.x = p_s.x; p_d.y = p_s.y; p_d.z = p_s.z;
            p_d.intensity = p_s.intensity;
            p_d.curvature = 0.0f; // 迫不得已降级置0，至少保命不崩溃
            cloud->push_back(p_d);
        }
        return cloud;
    }
}