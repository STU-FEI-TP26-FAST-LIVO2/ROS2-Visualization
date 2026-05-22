#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher()
    : Node("camera_publisher")
    {
        input_topic_ = this->declare_parameter<std::string>(
            "input_topic", "/basler/image_raw");

        output_topic_ = this->declare_parameter<std::string>(
            "output_topic", "/camera_image/compressed");

        jpeg_quality_ = this->declare_parameter<int>(
            "jpeg_quality", 80);

        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            output_topic_, 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_,
            10,
            std::bind(&CameraPublisher::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Camera publisher started");
        RCLCPP_INFO(this->get_logger(), "Subscribed topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing topic: %s", output_topic_.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImageConstPtr cv_ptr;

            if (msg->encoding == "rgb8" || msg->encoding == "bgr8" || msg->encoding == "mono8") {
                cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
            } else {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            }

            cv::Mat image = cv_ptr->image;
            cv::Mat image_to_encode;

            if (msg->encoding == "rgb8") {
                cv::cvtColor(image, image_to_encode, cv::COLOR_RGB2BGR);
            } else {
                image_to_encode = image;
            }

            std::vector<uchar> buffer;
            std::vector<int> params = {
                cv::IMWRITE_JPEG_QUALITY, jpeg_quality_
            };

            if (!cv::imencode(".jpg", image_to_encode, buffer, params)) {
                RCLCPP_WARN(this->get_logger(), "Failed to encode image to JPEG");
                return;
            }

            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header = msg->header;
            compressed_msg.format = "jpeg";
            compressed_msg.data = buffer;

            publisher_->publish(compressed_msg);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Published compressed image: %zu bytes",
                compressed_msg.data.size());

        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    std::string input_topic_;
    std::string output_topic_;
    int jpeg_quality_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
