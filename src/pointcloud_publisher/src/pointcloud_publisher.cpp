#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <fstream>
#include <vector>
#include <iomanip>
#include <sstream>

class PointCloudPublisher : public rclcpp::Node
{
public:
    PointCloudPublisher()
    : Node("pointcloud_publisher"),
      frame_index_(0)
    {
        publisher_ =
            this->create_publisher<
            sensor_msgs::msg::PointCloud2>(
                "/lidar_points", 10);

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(200),
                std::bind(
                    &PointCloudPublisher::timerCallback,
                    this));

        dataset_path_ =
        "/home/dominika/ROS2-Visualization/dataset/kitti/velodyne/sequences/05/velodyne/";

        RCLCPP_INFO(
            this->get_logger(),
            "Sequential PointCloud publisher started");
    }

private:

    void timerCallback()
    {
        std::stringstream ss;

        ss << dataset_path_
           << std::setw(6)
           << std::setfill('0')
           << frame_index_
           << ".bin";

        std::string file_path = ss.str();

        std::ifstream file(
            file_path,
            std::ios::binary);

        if (!file.is_open()) {

            RCLCPP_WARN(
                this->get_logger(),
                "End of dataset reached");

            frame_index_ = 0;
            return;
        }

        std::vector<float> points;

        float x, y, z, intensity;

        while (file.read((char*)&x, sizeof(float)) &&
               file.read((char*)&y, sizeof(float)) &&
               file.read((char*)&z, sizeof(float)) &&
               file.read((char*)&intensity, sizeof(float)))
        {
            points.push_back(x);
            points.push_back(y);
            points.push_back(z);
        }

        file.close();

        sensor_msgs::msg::PointCloud2 cloud;

        cloud.header.frame_id = "map";
        cloud.header.stamp =
            this->get_clock()->now();

        cloud.height = 1;
        cloud.width = points.size() / 3;

        sensor_msgs::PointCloud2Modifier modifier(cloud);

        modifier.setPointCloud2FieldsByString(
            1, "xyz");

        modifier.resize(cloud.width);

        sensor_msgs::PointCloud2Iterator<float>
            iter_x(cloud, "x");

        for (size_t i = 0;
             i < points.size();
             i += 3)
        {
            iter_x[0] = points[i];
            iter_x[1] = points[i+1];
            iter_x[2] = points[i+2];

            ++iter_x;
        }

        publisher_->publish(cloud);

        RCLCPP_INFO(
            this->get_logger(),
            "Frame %d (%u points)",
            frame_index_,
            cloud.width);

        frame_index_++;
    }

    std::string dataset_path_;

    int frame_index_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PointCloudPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}