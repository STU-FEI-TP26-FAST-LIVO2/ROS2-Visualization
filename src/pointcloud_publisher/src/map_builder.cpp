#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class MapBuilder : public rclcpp::Node
{
public:
    MapBuilder()
    : Node("map_builder")
    {
        input_cloud_topic_ = this->declare_parameter<std::string>(
            "input_cloud_topic", "/cloud_registered");

        // parameter nechávam kvôli kompatibilite so system_controller,
        // v tejto verzii sa IMU nepoužíva
        input_imu_topic_ = this->declare_parameter<std::string>(
            "input_imu_topic", "/imu");

        output_map_topic_ = this->declare_parameter<std::string>(
            "output_map_topic", "/map_points");

        input_leaf_size_ = this->declare_parameter<double>(
            "input_leaf_size", 0.18);

        map_leaf_size_ = this->declare_parameter<double>(
            "map_leaf_size", 0.25);

        web_leaf_size_ = this->declare_parameter<double>(
            "web_leaf_size", 0.30);

        max_points_before_filter_ = this->declare_parameter<int>(
            "max_points_before_filter", 250000);

        publish_period_ms_ = this->declare_parameter<int>(
            "publish_period_ms", 1000);

        global_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        last_publish_time_ =
            std::chrono::steady_clock::now() -
            std::chrono::milliseconds(publish_period_ms_);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_map_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)));

        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&MapBuilder::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "RGB Map builder started");
        RCLCPP_INFO(this->get_logger(), "Subscribed cloud topic: %s", input_cloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing map topic: %s", output_map_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Input leaf size: %.2f", input_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "Map leaf size: %.2f", map_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "Web leaf size: %.2f", web_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "Publish period: %d ms", publish_period_ms_);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>());

        // Tu sa načíta x, y, z aj rgb z /cloud_registered
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Received empty input cloud");
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_input(
            new pcl::PointCloud<pcl::PointXYZRGB>());

        pcl::VoxelGrid<pcl::PointXYZRGB> input_filter;
        input_filter.setInputCloud(input_cloud);
        input_filter.setLeafSize(
            static_cast<float>(input_leaf_size_),
            static_cast<float>(input_leaf_size_),
            static_cast<float>(input_leaf_size_));
        input_filter.filter(*filtered_input);

        if (filtered_input->empty()) {
            return;
        }

        // Pridanie aktuálneho farebného cloudu do globálnej mapy
        *global_map_ += *filtered_input;

        // Interná mapa sa občas zmenší, aby nerástla donekonečna
        if (global_map_->points.size() >
            static_cast<std::size_t>(max_points_before_filter_)) {

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_map(
                new pcl::PointCloud<pcl::PointXYZRGB>());

            pcl::VoxelGrid<pcl::PointXYZRGB> map_filter;
            map_filter.setInputCloud(global_map_);
            map_filter.setLeafSize(
                static_cast<float>(map_leaf_size_),
                static_cast<float>(map_leaf_size_),
                static_cast<float>(map_leaf_size_));
            map_filter.filter(*filtered_map);

            global_map_ = filtered_map;

            RCLCPP_INFO(
                this->get_logger(),
                "Internal RGB map filtered, current points: %zu",
                global_map_->points.size());
        }

        // Na web neposielame každú správu
        auto now = std::chrono::steady_clock::now();

        auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_publish_time_).count();

        if (elapsed_ms < publish_period_ms_) {
            return;
        }

        last_publish_time_ = now;

        // Pre web ešte redšia farebná mapa
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr web_map(
            new pcl::PointCloud<pcl::PointXYZRGB>());

        pcl::VoxelGrid<pcl::PointXYZRGB> web_filter;
        web_filter.setInputCloud(global_map_);
        web_filter.setLeafSize(
            static_cast<float>(web_leaf_size_),
            static_cast<float>(web_leaf_size_),
            static_cast<float>(web_leaf_size_));
        web_filter.filter(*web_map);

        if (web_map->empty()) {
            return;
        }

        sensor_msgs::msg::PointCloud2 output_cloud;
        pcl::toROSMsg(*web_map, output_cloud);

        output_cloud.header.stamp = msg->header.stamp;

        if (!msg->header.frame_id.empty()) {
            output_cloud.header.frame_id = msg->header.frame_id;
        } else {
            output_cloud.header.frame_id = "map";
        }

        publisher_->publish(output_cloud);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Internal RGB map: %zu points | Web RGB map: %zu points",
            global_map_->points.size(),
            web_map->points.size());
    }

    std::string input_cloud_topic_;
    std::string input_imu_topic_;
    std::string output_map_topic_;

    double input_leaf_size_;
    double map_leaf_size_;
    double web_leaf_size_;

    int max_points_before_filter_;
    int publish_period_ms_;

    std::chrono::steady_clock::time_point last_publish_time_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MapBuilder>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
