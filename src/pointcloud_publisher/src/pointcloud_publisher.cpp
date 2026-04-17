#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudPublisher : public rclcpp::Node
{
public:
    PointCloudPublisher()
    : Node("pointcloud_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/hesai/pandar",
            10,
            std::bind(&PointCloudPublisher::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PointCloud downsampling republisher started");
        RCLCPP_INFO(this->get_logger(), "Subscribed topic: /hesai/pandar");
        RCLCPP_INFO(this->get_logger(), "Publishing topic: /lidar_points");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *input_cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(input_cloud);
        voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f);
        voxel_filter.filter(*filtered_cloud);

        sensor_msgs::msg::PointCloud2 output_cloud;
        pcl::toROSMsg(*filtered_cloud, output_cloud);

        output_cloud.header = msg->header;

        publisher_->publish(output_cloud);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Input points: %zu, Filtered points: %zu",
            input_cloud->points.size(),
            filtered_cloud->points.size());
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
