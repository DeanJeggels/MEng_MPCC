// path_recorder_rtabmap.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <iomanip>



struct PathPoint {
    double x, y, timestamp;
    double left_bound = 0.0;   // Can be set later
    double right_bound = 0.0;  // Can be set later
};

class PathRecorder : public rclcpp::Node {
public:

std::string robot_name_ = "mpcc";
    PathRecorder() : Node("path_recorder") {
        // Parameters
        this->declare_parameter("output_file", "recorded_path.csv");
        this->declare_parameter("min_distance_threshold", 0.1);  // Minimum distance between recorded points
        this->declare_parameter("recording_rate", 10.0);  // Hz
        
        output_file_ = this->get_parameter("output_file").as_string();
        min_distance_threshold_ = this->get_parameter("min_distance_threshold").as_double();
        
        // Subscriber to RTAB-Map localization pose
        using std::placeholders::_1;
        localization_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization_pose", 10,
            std::bind(&PathRecorder::localizationPoseCallback, this, std::placeholders::_1));

                    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states", 10,
            std::bind(&PathRecorder::odomCallback, this, _1));
        
        // Services for start/stop recording
        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_recording",
            std::bind(&PathRecorder::startRecording, this, std::placeholders::_1, std::placeholders::_2));
        
        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_recording", 
            std::bind(&PathRecorder::stopRecording, this, std::placeholders::_1, std::placeholders::_2));
        
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_path",
            std::bind(&PathRecorder::savePath, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "RTAB-Map Path Recorder Node Started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /localization_pose");
        RCLCPP_INFO(this->get_logger(), "Output file: %s", output_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Min distance threshold: %.3f m", min_distance_threshold_);
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Services available:");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /start_recording std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /stop_recording std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /save_path std_srvs/srv/Trigger");
    }

private:
    void localizationPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (!is_recording_) return;
        
        // Extract x, y from RTAB-Map localization pose
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;
        double current_time = this->now().seconds();
        
        // Check if we should record this point (distance threshold)
        if (!recorded_points_.empty()) {
            const auto& last_point = recorded_points_.back();
            double distance = std::sqrt(std::pow(current_x - last_point.x, 2) + 
                                      std::pow(current_y - last_point.y, 2));
            
            if (distance < min_distance_threshold_) {
                return;  // Skip this point, too close to last recorded point
            }
        }
        
        // Record the point
        PathPoint point;
        point.x = current_x;
        point.y = current_y;
        point.timestamp = current_time;
        recorded_points_.push_back(point);
        
        points_recorded_++;
        
        // Log progress periodically
        if (points_recorded_ % 50 == 0) {
            double total_distance = calculateTotalDistance();
            RCLCPP_INFO(this->get_logger(), 
                       "Recorded %zu points, Total distance: %.2f m", 
                       points_recorded_, total_distance);
        }
    }

    void odomCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        if (!is_recording_) return;

        auto it = std::find(msg->name.begin(), msg->name.end(), robot_name_);
        if (it == msg->name.end()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                              "Robot model '%s' not found in model_states", 
                              robot_name_.c_str());
            return;
        }

        size_t robot_index = std::distance(msg->name.begin(), it);
        const auto& pose = msg->pose[robot_index];
        double current_x = pose.position.x;
        double current_y = pose.position.y;
        double current_time = this->now().seconds();

        
        // Check if we should record this point (distance threshold)
        if (!recorded_points_.empty()) {
            const auto& last_point = recorded_points_.back();
            double distance = std::sqrt(std::pow(current_x - last_point.x, 2) + 
                                      std::pow(current_y - last_point.y, 2));
            
            if (distance < min_distance_threshold_) {
                return;  // Skip this point, too close to last recorded point
            }
        }
        
        // Record the point
        PathPoint point;
        point.x = current_x;
        point.y = current_y;
        point.timestamp = current_time;
        recorded_points_.push_back(point);
        
        points_recorded_++;
        
        // Log progress periodically
        if (points_recorded_ % 50 == 0) {
            double total_distance = calculateTotalDistance();
            RCLCPP_INFO(this->get_logger(), 
                       "Recorded %zu points, Total distance: %.2f m", 
                       points_recorded_, total_distance);
        }
    }
    
    void startRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (is_recording_) {
            response->success = false;
            response->message = "Already recording!";
            return;
        }
        
        is_recording_ = true;
        recorded_points_.clear();
        points_recorded_ = 0;
        start_time_ = this->now().seconds();
        
        response->success = true;
        response->message = "Started recording RTAB-Map localization path";
        
        RCLCPP_INFO(this->get_logger(), "🔴 STARTED RECORDING RTAB-MAP PATH");
        RCLCPP_INFO(this->get_logger(), "Drive your robot now. Localization poses will be recorded automatically.");
        RCLCPP_INFO(this->get_logger(), "Make sure RTAB-Map is publishing to /localization_pose");
    }
    
    void stopRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!is_recording_) {
            response->success = false;
            response->message = "Not currently recording!";
            return;
        }
        
        is_recording_ = false;
        double end_time = this->now().seconds();
        double recording_duration = end_time - start_time_;
        double total_distance = calculateTotalDistance();
        
        response->success = true;
        response->message = std::string("Stopped recording. Points: ") + 
                           std::to_string(points_recorded_) + 
                           ", Distance: " + std::to_string(total_distance) + "m";
        
        RCLCPP_INFO(this->get_logger(), "⏹️ STOPPED RECORDING RTAB-MAP PATH");
        RCLCPP_INFO(this->get_logger(), "Recording summary:");
        RCLCPP_INFO(this->get_logger(), "  Points recorded: %zu", points_recorded_);
        RCLCPP_INFO(this->get_logger(), "  Total distance: %.2f m", total_distance);
        RCLCPP_INFO(this->get_logger(), "  Recording time: %.1f s", recording_duration);
        RCLCPP_INFO(this->get_logger(), "  Average speed: %.2f m/s", total_distance / recording_duration);
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Use 'ros2 service call /save_path std_srvs/srv/Trigger' to save to CSV");
    }
    
    void savePath(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (recorded_points_.empty()) {
            response->success = false;
            response->message = "No points recorded yet!";
            return;
        }
        
        std::ofstream file(output_file_);
        if (!file.is_open()) {
            response->success = false;
            response->message = "Failed to open output file: " + output_file_;
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", output_file_.c_str());
            return;
        }
        
        // Write header compatible with your MPCC controller
        file << "x,y,left_bound,right_bound\n";
        
        // Write all recorded points
        for (const auto& point : recorded_points_) {
            file << std::fixed << std::setprecision(6) 
                 << point.x << ","
                 << point.y << ","
                 << point.left_bound << ","  // Default 0.0, you can modify later
                 << point.right_bound << "\n";
        }
        
        file.close();
        
        double total_distance = calculateTotalDistance();
        
        response->success = true;
        response->message = "RTAB-Map path saved to " + output_file_ + 
                           " (" + std::to_string(points_recorded_) + " points, " +
                           std::to_string(total_distance) + "m)";
        
        RCLCPP_INFO(this->get_logger(), "💾 RTAB-MAP PATH SAVED TO: %s", output_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "File contains %zu points covering %.2f meters", 
                   points_recorded_, total_distance);
    }
    
    double calculateTotalDistance() const {
        if (recorded_points_.size() < 2) return 0.0;
        
        double total = 0.0;
        for (size_t i = 1; i < recorded_points_.size(); ++i) {
            double dx = recorded_points_[i].x - recorded_points_[i-1].x;
            double dy = recorded_points_[i].y - recorded_points_[i-1].y;
            total += std::sqrt(dx*dx + dy*dy);
        }
        return total;
    }
    
    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_pose_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
     rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    
    std::string output_file_;
    double min_distance_threshold_;
    
    bool is_recording_ = false;
    std::vector<PathPoint> recorded_points_;
    size_t points_recorded_ = 0;
    double start_time_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathRecorder>());
    rclcpp::shutdown();
    return 0;
}
