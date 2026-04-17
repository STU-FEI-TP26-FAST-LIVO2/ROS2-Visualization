#include <csignal>
#include <memory>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cerrno>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class SystemController : public rclcpp::Node
{
public:
    SystemController()
    : Node("system_controller"),
      state_("idle"),
      bag_pid_(-1),
      pointcloud_pid_(-1),
      camera_pid_(-1)
    {
        bag_path_ = this->declare_parameter<std::string>(
            "bag_path",
            "/home/gabriela/exp14_basement_2");

        camera_input_topic_ = this->declare_parameter<std::string>(
            "camera_input_topic",
            "/alphasense/cam0/image_raw");

        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/ui/status", 10);

        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/start_system",
            std::bind(
                &SystemController::startCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/stop_system",
            std::bind(
                &SystemController::stopCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        publishStatus();

        RCLCPP_INFO(this->get_logger(), "System controller started");
        RCLCPP_INFO(this->get_logger(), "Bag path: %s", bag_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera input topic: %s", camera_input_topic_.c_str());
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

    pid_t startCommand(const std::string & command)
    {
        pid_t pid = fork();

        if (pid == 0) {
            // child: vlastná process group
            if (setpgid(0, 0) != 0) {
                perror("setpgid failed in child");
                _exit(1);
            }

            execlp(
                "bash",
                "bash",
                "-lc",
                command.c_str(),
                static_cast<char *>(nullptr)
            );

            perror("execlp failed");
            _exit(1);
        }

        if (pid < 0) {
            throw std::runtime_error("fork() failed");
        }

        // parent: pre istotu nastav process group tiež
        if (setpgid(pid, pid) != 0 && errno != EACCES) {
            RCLCPP_WARN(
                this->get_logger(),
                "setpgid failed in parent for pid=%d: %s",
                pid,
                strerror(errno));
        }

        return pid;
    }

    bool isProcessAlive(pid_t pid)
    {
        if (pid <= 0) {
            return false;
        }

        int ret = kill(pid, 0);
        return (ret == 0);
    }

    bool waitForExit(pid_t pid, int retries, useconds_t sleep_us)
    {
        for (int i = 0; i < retries; ++i) {
            int status = 0;
            pid_t result = waitpid(pid, &status, WNOHANG);

            if (result == pid) {
                return true;
            }

            if (result == -1) {
                return true;
            }

            usleep(sleep_us);
        }

        return false;
    }

    void stopProcess(pid_t & pid, const std::string & name)
    {
        if (pid <= 0) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Stopping %s (pid=%d)", name.c_str(), pid);

        // SIGINT celej process group
        if (kill(-pid, SIGINT) != 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to send SIGINT to %s group (pid=%d): %s",
                name.c_str(),
                pid,
                strerror(errno));
        }

        if (waitForExit(pid, 20, 100000)) {
            pid = -1;
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "%s did not stop after SIGINT, sending SIGTERM",
            name.c_str());

        if (kill(-pid, SIGTERM) != 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to send SIGTERM to %s group (pid=%d): %s",
                name.c_str(),
                pid,
                strerror(errno));
        }

        if (waitForExit(pid, 20, 100000)) {
            pid = -1;
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "%s did not stop after SIGTERM, sending SIGKILL",
            name.c_str());

        if (kill(-pid, SIGKILL) != 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to send SIGKILL to %s group (pid=%d): %s",
                name.c_str(),
                pid,
                strerror(errno));
        }

        waitpid(pid, nullptr, 0);
        pid = -1;
    }

    void stopAllProcesses()
    {
        stopProcess(camera_pid_, "camera_publisher_node");
        stopProcess(pointcloud_pid_, "pointcloud_publisher_node");
        stopProcess(bag_pid_, "rosbag");
    }

    void startCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        RCLCPP_INFO(this->get_logger(), "Received /start_system request");

        if (state_ == "running") {
            response->success = false;
            response->message = "System already running";
            return;
        }

        try {
            RCLCPP_INFO(this->get_logger(), "Starting system...");

            const std::string ros_setup =
                "source /opt/ros/foxy/setup.bash && "
                "source ~/ROS2-Visualization/install/setup.bash && ";

            bag_pid_ = startCommand(
                ros_setup +
                "ros2 bag play \"" + bag_path_ + "\""
            );

            sleep(2);

            if (!isProcessAlive(bag_pid_)) {
                throw std::runtime_error("ros2 bag play failed to stay running");
            }

            pointcloud_pid_ = startCommand(
                ros_setup +
                "ros2 run pointcloud_publisher pointcloud_publisher_node"
            );

            sleep(1);

            if (!isProcessAlive(pointcloud_pid_)) {
                throw std::runtime_error("pointcloud_publisher_node failed to stay running");
            }

            camera_pid_ = startCommand(
                ros_setup +
                "ros2 run pointcloud_publisher camera_publisher_node "
                "--ros-args -p input_topic:=" + camera_input_topic_
            );

            sleep(1);

            if (!isProcessAlive(camera_pid_)) {
                throw std::runtime_error("camera_publisher_node failed to stay running");
            }

            state_ = "running";
            publishStatus();

            response->success = true;
            response->message = "System started";
            RCLCPP_INFO(this->get_logger(), "System started");
        }
        catch (const std::exception & e) {
            stopAllProcesses();
            state_ = "error";
            publishStatus();

            response->success = false;
            response->message = e.what();
            RCLCPP_ERROR(this->get_logger(), "Start failed: %s", e.what());
        }
    }

    void stopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        RCLCPP_INFO(this->get_logger(), "Received /stop_system request");

        try {
            RCLCPP_INFO(this->get_logger(), "Stopping system...");

            stopAllProcesses();

            state_ = "stopped";
            publishStatus();

            response->success = true;
            response->message = "System stopped";
            RCLCPP_INFO(this->get_logger(), "System stopped");
        }
        catch (const std::exception & e) {
            state_ = "error";
            publishStatus();

            response->success = false;
            response->message = e.what();
            RCLCPP_ERROR(this->get_logger(), "Stop failed: %s", e.what());
        }
    }

    std::string state_;
    std::string bag_path_;
    std::string camera_input_topic_;

    pid_t bag_pid_;
    pid_t pointcloud_pid_;
    pid_t camera_pid_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
