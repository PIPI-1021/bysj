#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudFusionNode : public rclcpp::Node {
public:
    PointCloudFusionNode() : Node("pointcloud_fusion_node") {
        // 参数设置
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("lidar_topic", "/points_raw");
        this->declare_parameter("camera_topic", "/camera/depth/color/points");

        target_frame_ = this->get_parameter("target_frame").as_string();

        // 1. TF 监听器初始化 (修正了类名错误)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 2. 订阅者
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("lidar_topic").as_string(), 10,
            std::bind(&PointCloudFusionNode::lidar_callback, this, std::placeholders::_1));

        camera_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("camera_topic").as_string(), 10,
            std::bind(&PointCloudFusionNode::camera_callback, this, std::placeholders::_1));

        // 3. 发布者
        fused_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fused_points", 10);

        RCLCPP_INFO(this->get_logger(), "点云融合节点已启动，目标坐标系: %s", target_frame_.c_str());
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        process_cloud(msg, "lidar");
    }

    void camera_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        process_cloud(msg, "camera");
    }

    void process_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, std::string sensor_type) {
        sensor_msgs::msg::PointCloud2 transformed_cloud;

        // 空间配准
        try {
            // 注意：tf2::durationFromSec 在某些版本可能需要替换为 rclcpp::Duration::from_seconds
            tf_buffer_->transform(*msg, transformed_cloud, target_frame_, tf2::durationFromSec(0.2));
        } catch (tf2::TransformException &ex) {
            // 降低警告频率，防止刷屏
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "等待 TF 变换 %s -> %s: %s", msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

        // 使用静态变量简单存储融合点云
        static pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *cumulative_cloud += *pcl_cloud;

        // 当点云积累到一定程度或定时进行滤波发布
        if (cumulative_cloud->points.size() > 2000) { 
            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud(cumulative_cloud);
            sor.setLeafSize(0.05f, 0.05f, 0.05f); 
            sor.filter(*cumulative_cloud);

            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*cumulative_cloud, output);
            output.header.frame_id = target_frame_;
            output.header.stamp = this->now();
            fused_pub_->publish(output);

            cumulative_cloud->clear(); 
        }
    }

    std::string target_frame_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // 修正了 std::shared_shared 和 Transform_listener
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFusionNode>());
    rclcpp::shutdown();
    return 0;
}
