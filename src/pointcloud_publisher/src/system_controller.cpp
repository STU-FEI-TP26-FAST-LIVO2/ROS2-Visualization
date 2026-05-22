#include <csignal>
#include <memory>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdexcept>
#include <ctime>
#include <filesystem>
#include <sstream>
#include <chrono>
#include <cstdlib>

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
      pointcloud_pid_(-1),
      camera_pid_(-1),
      map_builder_pid_(-1),
      record_pid_(-1),
      run_all_pid_(-1)
    {
        camera_input_topic_ = this->declare_parameter<std::string>(
            "camera_input_topic",
            "/rgb_img");

        imu_input_topic_ = this->declare_parameter<std::string>(
            "imu_input_topic",
            "/imu");

        recordings_dir_ = this->declare_parameter<std::string>(
            "recordings_dir",
            "/home/jetson/ROS2-Visualization/recordings");

        run_all_script_ = this->declare_parameter<std::string>(
            "run_all_script",
            "/home/jetson/run_all.sh");

        logs_dir_ = recordings_dir_ + "/controller_logs";

        status_pub_ = this->create_publisher<std_msgs::msg::String>("/ui/status", 10);

        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/start_system",
            std::bind(&SystemController::startCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/stop_system",
            std::bind(&SystemController::stopCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        list_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/list_recordings",
            std::bind(&SystemController::listCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        run_all_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/run_all",
            std::bind(&SystemController::runAllCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        publishStatus();

        RCLCPP_INFO(this->get_logger(), "System controller started WITH map_builder and run_all service");
    }

    ~SystemController() override
    {
        stopAllProcesses();
    }

private:
    std::string rosSetup()
    {
        return
            "source /opt/ros/humble/setup.bash && "
            "source /home/jetson/ROS2-Visualization/install/setup.bash && ";
    }

    std::string getTimestamp()
    {
        std::time_t now = std::time(nullptr);
        std::tm *tm = std::localtime(&now);

        char buffer[64];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", tm);

        return std::string(buffer);
    }

    void publishStatus()
    {
        std_msgs::msg::String msg;
        msg.data = state_;
        status_pub_->publish(msg);
    }

    pid_t startCommandWithLog(const std::string & command, const std::string & log_file)
    {
        std::string full_cmd =
            "mkdir -p \"" + logs_dir_ + "\" && "
            "(" + command + ") > \"" + log_file + "\" 2>&1";

        RCLCPP_INFO(this->get_logger(), "Starting command:");
        RCLCPP_INFO(this->get_logger(), "%s", command.c_str());
        RCLCPP_INFO(this->get_logger(), "Log: %s", log_file.c_str());

        pid_t pid = fork();

        if (pid == 0) {
            setpgid(0, 0);
            execlp("bash", "bash", "-lc", full_cmd.c_str(), nullptr);
            perror("exec failed");
            _exit(1);
        }

        if (pid < 0) {
            throw std::runtime_error("fork() failed");
        }

        setpgid(pid, pid);
        return pid;
    }

    pid_t startCommandNoLog(const std::string & command)
    {
        RCLCPP_INFO(this->get_logger(), "Starting command without log redirect:");
        RCLCPP_INFO(this->get_logger(), "%s", command.c_str());

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

    bool isAlive(pid_t pid)
    {
        return pid > 0 && kill(pid, 0) == 0;
    }

    bool waitForExit(pid_t pid, int loops)
    {
        for (int i = 0; i < loops; ++i) {
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

        RCLCPP_INFO(this->get_logger(), "Stopping %s pid=%d", name.c_str(), pid);

        kill(-pid, SIGINT);

        int wait_loops = (name == "rosbag_record") ? 200 : 50;

        if (waitForExit(pid, wait_loops)) {
            pid = -1;
            return;
        }

        if (name == "rosbag_record") {
            RCLCPP_WARN(this->get_logger(), "rosbag_record did not exit after SIGINT");
        }

        kill(-pid, SIGTERM);

        if (waitForExit(pid, 50)) {
            pid = -1;
            return;
        }

        kill(-pid, SIGKILL);
        waitpid(pid, nullptr, 0);

        pid = -1;
    }

    void stopAllProcesses()
    {
        stopProcess(run_all_pid_, "run_all");
        stopProcess(record_pid_, "rosbag_record");
        stopProcess(map_builder_pid_, "map_builder");
        stopProcess(camera_pid_, "camera");
        stopProcess(pointcloud_pid_, "pointcloud");
    }

    void reindexLastBag()
    {
        if (last_session_path_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No last session path to reindex");
            return;
        }

        if (!fs::exists(last_session_path_)) {
            RCLCPP_WARN(this->get_logger(), "Bag folder does not exist: %s", last_session_path_.c_str());
            return;
        }

        std::string cmd =
            "source /opt/ros/humble/setup.bash && "
            "ros2 bag reindex \"" + last_session_path_ + "\"";

        RCLCPP_INFO(this->get_logger(), "Reindexing bag:");
        RCLCPP_INFO(this->get_logger(), "%s", cmd.c_str());

        int ret = std::system(("bash -lc '" + cmd + "'").c_str());

        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "Bag reindex finished successfully");
        } else {
            RCLCPP_WARN(this->get_logger(), "Bag reindex failed");
        }
    }

    void waitForTopics()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for output topics...");

        bool lidar_ready = false;
        bool camera_ready = false;
        bool map_ready = false;

        for (int i = 0; i < 100; ++i) {
            if (this->count_publishers("/lidar") > 0) {
                lidar_ready = true;
            }

            if (this->count_publishers("/camera_image/compressed") > 0) {
                camera_ready = true;
            }

            if (this->count_publishers("/map_points") > 0) {
                map_ready = true;
            }

            if (lidar_ready && camera_ready && map_ready) {
                RCLCPP_INFO(this->get_logger(), "Topics are ready");
                return;
            }

            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        if (!lidar_ready) {
            throw std::runtime_error("/lidar publisher not found");
        }

        if (!camera_ready) {
            throw std::runtime_error("/camera_image/compressed publisher not found");
        }

        if (!map_ready) {
            throw std::runtime_error("/map_points publisher not found");
        }
    }

    void runAllCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "START button pressed - launching run_all.sh");

        try {
            fs::create_directories(logs_dir_);

            if (!fs::exists(run_all_script_)) {
                throw std::runtime_error("run_all.sh not found: " + run_all_script_);
            }

            const std::string t = getTimestamp();

            std::string script_dir = fs::path(run_all_script_).parent_path().string();
            std::string script_name = fs::path(run_all_script_).filename().string();

            std::string cmd =
                "cd \"" + script_dir + "\" && "
                "bash \"" + script_name + "\"";

            run_all_pid_ = startCommandWithLog(
                cmd,
                logs_dir_ + "/run_all_" + t + ".log"
            );

            state_ = "run_all_started";
            publishStatus();

            response->success = true;
            response->message = "Started run_all.sh";

        } catch (const std::exception & e) {
            state_ = "run_all_error";
            publishStatus();

            response->success = false;
            response->message = e.what();

            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void startCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "START RECORD pressed");

        if (state_ == "recording") {
            response->success = false;
            response->message = "Already recording";
            return;
        }

        try {
            fs::create_directories(recordings_dir_);
            fs::create_directories(logs_dir_);

            const std::string t = getTimestamp();
            const std::string ros = rosSetup();

            pointcloud_pid_ = startCommandWithLog(
                ros + "ros2 run pointcloud_publisher pointcloud_publisher_node",
                logs_dir_ + "/pointcloud_" + t + ".log"
            );

            sleep(1);

            if (!isAlive(pointcloud_pid_)) {
                throw std::runtime_error("pointcloud publisher failed");
            }

            map_builder_pid_ = startCommandWithLog(
                ros +
                "ros2 run pointcloud_publisher map_builder_node "
                "--ros-args "
                "-p input_cloud_topic:=/cloud_registered "
                "-p input_imu_topic:=\"" + imu_input_topic_ + "\" "
                "-p output_map_topic:=/map_points",
                logs_dir_ + "/map_builder_" + t + ".log"
            );

            sleep(1);

            if (!isAlive(map_builder_pid_)) {
                throw std::runtime_error("map_builder failed");
            }

            camera_pid_ = startCommandWithLog(
                ros +
                "ros2 run pointcloud_publisher camera_publisher_node "
                "--ros-args -p input_topic:=\"" + camera_input_topic_ + "\"",
                logs_dir_ + "/camera_" + t + ".log"
            );

            sleep(1);

            if (!isAlive(camera_pid_)) {
                throw std::runtime_error("camera publisher failed");
            }

            waitForTopics();

            std::string session_name = "session_" + t;
            last_session_path_ = recordings_dir_ + "/" + session_name;

            record_pid_ = startCommandNoLog(
                "gnome-terminal -- bash -lc '"
                "source /opt/ros/humble/setup.bash && "
                "source /home/jetson/ROS2-Visualization/install/setup.bash && "
                "mkdir -p /home/jetson/ROS2-Visualization/recordings && "
                "cd /home/jetson/ROS2-Visualization/recordings && "
                "ros2 bag record -o " + session_name + " "
                "/lidar_points "
                "/camera_image/compressed "
                "/map_points; "
                "exec bash'"
            );

            sleep(2);

            if (!isAlive(record_pid_)) {
                throw std::runtime_error("rosbag record terminal failed");
            }

            state_ = "recording";
            publishStatus();

            response->success = true;
            response->message = "Started recording with map_builder";

        } catch (const std::exception & e) {
            stopAllProcesses();

            state_ = "error";
            publishStatus();

            response->success = false;
            response->message = e.what();

            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void stopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "STOP RECORD pressed");

        stopProcess(record_pid_, "rosbag_record");
        stopProcess(map_builder_pid_, "map_builder");
        stopProcess(camera_pid_, "camera");
        stopProcess(pointcloud_pid_, "pointcloud");

        sleep(2);

        reindexLastBag();

        state_ = "stopped";
        publishStatus();

        response->success = true;
        response->message = "Stopped recording and reindexed";
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
    std::string camera_input_topic_;
    std::string imu_input_topic_;
    std::string recordings_dir_;
    std::string logs_dir_;
    std::string last_session_path_;
    std::string run_all_script_;

    pid_t pointcloud_pid_;
    pid_t camera_pid_;
    pid_t map_builder_pid_;
    pid_t record_pid_;
    pid_t run_all_pid_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr list_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_all_service_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SystemController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
