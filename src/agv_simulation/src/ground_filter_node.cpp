#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class GroundFilterNode : public rclcpp::Node {
public:
    GroundFilterNode() : Node("ground_filter_node") {
        // 订阅 Gazebo 输出的原始点云 
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_raw", 10, std::bind(&GroundFilterNode::cloud_callback, this, std::placeholders::_1));
        
        // 发布过滤掉地面后的障碍物点云 [cite: 17]
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_points", 10);
        RCLCPP_INFO(this->get_logger(), "地面过滤节点已就绪，正在监听 /points_raw...");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        // RANSAC 平面分割 [cite: 6, 13]
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1); // 10cm 内的视为地面
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) return;

        // 提取非地面点 [cite: 14]
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true); // 移除地面，保留障碍物
        extract.filter(*obstacles);

        // 发布结果
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*obstacles, output);
        output.header = msg->header;
        publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundFilterNode>());
    rclcpp::shutdown();
    return 0;
}
