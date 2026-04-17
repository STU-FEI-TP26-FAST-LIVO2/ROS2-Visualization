#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

class MapBuilder : public rclcpp::Node
{
public:
    MapBuilder()
    : Node("map_builder"),
      imu_received_(false)
    {
        input_cloud_topic_ = this->declare_parameter<std::string>(
            "input_cloud_topic", "/lidar_points");

        input_imu_topic_ = this->declare_parameter<std::string>(
            "input_imu_topic", "/alphasense/imu");

        output_map_topic_ = this->declare_parameter<std::string>(
            "output_map_topic", "/map_points");

        map_leaf_size_ = this->declare_parameter<double>(
            "map_leaf_size", 0.20);

        input_leaf_size_ = this->declare_parameter<double>(
            "input_leaf_size", 0.15);

        max_points_before_filter_ = this->declare_parameter<int>(
            "max_points_before_filter", 300000);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_map_topic_, 10);

        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_,
            10,
            std::bind(&MapBuilder::pointCloudCallback, this, std::placeholders::_1));

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_imu_topic_,
            50,
            std::bind(&MapBuilder::imuCallback, this, std::placeholders::_1));

        global_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());

        latest_orientation_.setIdentity();

        RCLCPP_INFO(this->get_logger(), "Map builder started");
        RCLCPP_INFO(this->get_logger(), "Subscribed LiDAR topic: %s", input_cloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribed IMU topic: %s", input_imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing map topic: %s", output_map_topic_.c_str());
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const auto & q = msg->orientation;

        Eigen::Quaternionf quat(
            static_cast<float>(q.w),
            static_cast<float>(q.x),
            static_cast<float>(q.y),
            static_cast<float>(q.z));

        if (quat.norm() < 1e-6f) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Received invalid IMU quaternion");
            return;
        }

        quat.normalize();
        latest_orientation_ = quat;
        imu_received_ = true;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!imu_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for IMU orientation before building map");
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty input cloud");
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_input(new pcl::PointCloud<pcl::PointXYZI>());
        {
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setInputCloud(input_cloud);
            voxel_filter.setLeafSize(
                static_cast<float>(input_leaf_size_),
                static_cast<float>(input_leaf_size_),
                static_cast<float>(input_leaf_size_));
            voxel_filter.filter(*filtered_input);
        }

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = latest_orientation_.toRotationMatrix();

        pcl::PointCloud<pcl::PointXYZI>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*filtered_input, *rotated_cloud, transform);

        *global_map_ += *rotated_cloud;

        if (static_cast<int>(global_map_->points.size()) > max_points_before_filter_) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZI>());

            pcl::VoxelGrid<pcl::PointXYZI> map_filter;
            map_filter.setInputCloud(global_map_);
            map_filter.setLeafSize(
                static_cast<float>(map_leaf_size_),
                static_cast<float>(map_leaf_size_),
                static_cast<float>(map_leaf_size_));
            map_filter.filter(*filtered_map);

            global_map_ = filtered_map;

            RCLCPP_INFO(
                this->get_logger(),
                "Map voxel filtered, current points: %zu",
                global_map_->points.size());
        }

        sensor_msgs::msg::PointCloud2 output_cloud;
        pcl::toROSMsg(*global_map_, output_cloud);

        output_cloud.header.stamp = msg->header.stamp;
        output_cloud.header.frame_id = "map";

        publisher_->publish(output_cloud);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Map points: %zu",
            global_map_->points.size());
    }

    std::string input_cloud_topic_;
    std::string input_imu_topic_;
    std::string output_map_topic_;

    double map_leaf_size_;
    double input_leaf_size_;
    int max_points_before_filter_;

    bool imu_received_;
    Eigen::Quaternionf latest_orientation_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapBuilder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
