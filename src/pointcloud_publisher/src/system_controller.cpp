#include <csignal>
#include <memory>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cerrno>
#include <stdexcept>
#include <ctime>
#include <filesystem>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace fs = std::filesystem;

class SystemController : public rclcpp::Node
{
public:
    SystemController()
    : Node("system_controller"),
      state_("idle"),
      bag_pid_(-1),
      pointcloud_pid_(-1),
      camera_pid_(-1),
      record_pid_(-1)
    {
        bag_path_ = this->declare_parameter<std::string>(
            "bag_path",
            "/home/gabriela/exp14_basement_2");

        camera_input_topic_ = this->declare_parameter<std::string>(
            "camera_input_topic",
            "/alphasense/cam0/image_raw");

        recordings_dir_ = this->declare_parameter<std::string>(
            "recordings_dir",
            "/home/gabriela/ROS2-Visualization/recordings");

        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/ui/status", 10);

        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/start_system",
            std::bind(&SystemController::startCallback, this,
                std::placeholders::_1, std::placeholders::_2)
        );

        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/stop_system",
            std::bind(&SystemController::stopCallback, this,
                std::placeholders::_1, std::placeholders::_2)
        );

        list_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/list_recordings",
            std::bind(&SystemController::listCallback, this,
                std::placeholders::_1, std::placeholders::_2)
        );

        publishStatus();

        RCLCPP_INFO(this->get_logger(), "System controller started");
    }

    ~SystemController() override
    {
        stopAllProcesses();
    }

private:


    void publishStatus()
    {
        std_msgs::msg::String msg;
        msg.data = state_;
        status_pub_->publish(msg);
    }



    std::string getTimestamp()
    {
        std::time_t now = std::time(nullptr);
        std::tm *tm = std::localtime(&now);

        char buffer[64];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", tm);

        return std::string(buffer);
    }



    pid_t startCommand(const std::string & command)
    {
        pid_t pid = fork();

        if (pid == 0) {
            setpgid(0, 0);

            execlp("bash", "bash", "-lc", command.c_str(), nullptr);

            perror("exec failed");
            _exit(1);
        }

        if (pid < 0) {
            throw std::runtime_error("fork() failed");
        }

        setpgid(pid, pid);
        return pid;
    }

    bool isProcessAlive(pid_t pid)
    {
        if (pid <= 0) return false;
        return (kill(pid, 0) == 0);
    }

    bool waitForExit(pid_t pid)
    {
        for (int i = 0; i < 20; ++i) {
            int status;
            pid_t result = waitpid(pid, &status, WNOHANG);

            if (result == pid || result == -1) {
                return true;
            }

            usleep(100000);
        }
        return false;
    }

    void stopProcess(pid_t & pid, const std::string & name)
    {
        if (pid <= 0) return;

        RCLCPP_INFO(this->get_logger(), "Stopping %s (pid=%d)", name.c_str(), pid);

        kill(-pid, SIGINT);

        if (waitForExit(pid)) {
            pid = -1;
            return;
        }

        kill(-pid, SIGTERM);

        if (waitForExit(pid)) {
            pid = -1;
            return;
        }

        kill(-pid, SIGKILL);
        waitpid(pid, nullptr, 0);
        pid = -1;
    }

    void stopAllProcesses()
    {
        stopProcess(record_pid_, "rosbag_record");
        stopProcess(camera_pid_, "camera");
        stopProcess(pointcloud_pid_, "lidar");
        stopProcess(bag_pid_, "rosbag_play");
    }



    void startCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "START pressed");

        if (state_ == "running") {
            response->success = false;
            response->message = "Already running";
            return;
        }

        try {
            const std::string ros_setup =
                "source /opt/ros/foxy/setup.bash && "
                "source ~/ROS2-Visualization/install/setup.bash && ";

          
            bag_pid_ = startCommand(
                ros_setup + "ros2 bag play \"" + bag_path_ + "\""
            );

            sleep(2);

            if (!isProcessAlive(bag_pid_)) {
                throw std::runtime_error("bag play failed");
            }

          
            pointcloud_pid_ = startCommand(
                ros_setup +
                "ros2 run pointcloud_publisher pointcloud_publisher_node"
            );

            sleep(1);

          
            camera_pid_ = startCommand(
                ros_setup +
                "ros2 run pointcloud_publisher camera_publisher_node "
                "--ros-args -p input_topic:=" + camera_input_topic_
            );

            sleep(1);

          
            std::string session = "session_" + getTimestamp();

            std::string record_cmd =
                ros_setup +
                "mkdir -p " + recordings_dir_ + " && "
                "ros2 bag record -o " + recordings_dir_ + "/" + session + " "
                "/lidar_points /camera_image/compressed";

            record_pid_ = startCommand(record_cmd);

            sleep(1);

            if (!isProcessAlive(record_pid_)) {
                throw std::runtime_error("record failed");
            }

            state_ = "running";
            publishStatus();

            response->success = true;
            response->message = "Started";

        } catch (const std::exception & e) {
            stopAllProcesses();
            state_ = "error";
            publishStatus();

            response->success = false;
            response->message = e.what();
        }
    }



    void stopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "STOP pressed");

        stopAllProcesses();

        state_ = "stopped";
        publishStatus();

        response->success = true;
        response->message = "Stopped";
    }



    void listCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::stringstream result;

        try {
            if (!fs::exists(recordings_dir_)) {
                response->success = true;
                response->message = "";
                return;
            }

            for (const auto & entry : fs::directory_iterator(recordings_dir_)) {
                if (entry.is_directory()) {
                    result << entry.path().filename().string() << "\n";
                }
            }

            response->success = true;
            response->message = result.str();

        } catch (const std::exception & e) {
            response->success = false;
            response->message = e.what();
        }
    }



    std::string state_;
    std::string bag_path_;
    std::string camera_input_topic_;
    std::string recordings_dir_;

    pid_t bag_pid_;
    pid_t pointcloud_pid_;
    pid_t camera_pid_;
    pid_t record_pid_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr list_service_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
