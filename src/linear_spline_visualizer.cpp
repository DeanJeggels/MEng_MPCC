// linear_spline_visualizer.cpp
#include "mpcc/linear_spline_visualizer.hpp"
#include <fstream>

using namespace std::chrono_literals;

LinearSplineVisualizer::LinearSplineVisualizer() : Node("linear_spline_visualizer") {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("spline_path", 10);
    original_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("original_path", 10);
    
    timer_ = this->create_wall_timer(1s, std::bind(&LinearSplineVisualizer::timer_callback, this));
    ref_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/reference_position", 10);
    s_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "current_s", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
        auto point = spline_.getPointAtProgress(msg->data);
        auto ref_msg = std_msgs::msg::Float64MultiArray();
        
        auto derivatives = spline_.getDerivativesAtProgress(msg->data);
        ref_msg.data = {point.x, point.y, derivatives.dx_ds, derivatives.dy_ds};

        ref_pub_->publish(ref_msg);
    });
    
    load_and_publish();
}

void LinearSplineVisualizer::load_and_publish() {
    if (!spline_.loadFromCSV("/home/deanjeggs/new_ros2_ws/src/mpcc/racetracks/aut_centerline.csv")) {
        RCLCPP_ERROR(get_logger(), "Failed to load trajectory file");
        return;
    }

    // Also publish original csv trajectory
    nav_msgs::msg::Path original_path_msg;
    original_path_msg.header.stamp = now();
    original_path_msg.header.frame_id = "map";

    for (const auto& point : spline_.getTrajectoryPoints()) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.orientation.w = 1.0;
        original_path_msg.poses.push_back(pose);
    }

    original_path_pub_->publish(original_path_msg);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";
    
    const double max_s = spline_.getMaxProgress();
    for (double s = 0; s <= max_s; s += resolution_) {
        auto point = spline_.getPointAtProgress(s);
        
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }
    
    path_pub_->publish(path_msg);

    RCLCPP_INFO(get_logger(), "Published spline with %zu points", path_msg.poses.size());
}

void LinearSplineVisualizer::timer_callback() {
    load_and_publish();
}

// Main function to make this an executable
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinearSplineVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
