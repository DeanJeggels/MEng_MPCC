#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class ContouringErrorNode : public rclcpp::Node {
public:
    ContouringErrorNode() : Node("contouring_error_node") {
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&ContouringErrorNode::odomCallback, this, std::placeholders::_1));
            
        ref_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/reference_position", 10,
            std::bind(&ContouringErrorNode::referenceCallback, this, std::placeholders::_1));

        // Publisher for errors
        error_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/contouring_errors", 10);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        actual_x_ = msg->pose.pose.position.x;
        actual_y_ = msg->pose.pose.position.y;
        calculateErrors();
    }

    void referenceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 4) {
            xd_ = msg->data[0];
            yd_ = msg->data[1];
            dx_ds_ = msg->data[2];
            dy_ds_ = msg->data[3];
        }
    }

    void calculateErrors() {
        const double contouring_error = computeContouringError();
        const double lag_error = computeLagError();
        std_msgs::msg::Float64MultiArray error_msg;
        error_msg.data = {contouring_error, lag_error};
        error_pub_->publish(error_msg);
    }

    double computeContouringError() const {
        // ε_contour = sin(phi(s_))(x_-x_d(s_))-cos(phi(s_))(y_-y_d(s_))
        return std::sin(std::atan2(dy_ds_, dx_ds_)) * (actual_x_ - xd_) - std::cos(std::atan2(dy_ds_, dx_ds_)) * (actual_y_ - yd_);
        
    }

    double computeLagError() const {
        // ε_lag = -cos(phi(s_))(x_-x_d(s_))-sin(phi(s_))(y_-y_d(s_))
        return -std::cos(std::atan2(dy_ds_, dx_ds_)) * (actual_x_ - xd_) - std::sin(std::atan2(dy_ds_, dx_ds_)) * (actual_y_ - yd_);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub_;
    
    // State variables
    double actual_x_ = 0.0, actual_y_ = 0.0;
    double xd_ = 0.0, yd_ = 0.0;
    double dx_ds_ = 1.0, dy_ds_ = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ContouringErrorNode>());
    rclcpp::shutdown();
    return 0;
}
