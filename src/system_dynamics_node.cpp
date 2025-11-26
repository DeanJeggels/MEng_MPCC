// system_dynamics_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include "mpcc/linear_spline.hpp"
#include "std_msgs/msg/float64.hpp"

class SystemDynamicsNode : public rclcpp::Node {
public:
    SystemDynamicsNode() : Node("system_dynamics_node"), v_max_(1.2), omega_max_(M_PI / 2), update_rate_(50.0) {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&SystemDynamicsNode::odom_callback, this, std::placeholders::_1));
    
        dynamics_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/dynamics_estimation", 10);

        s_pub_ = this->create_publisher<std_msgs::msg::Float64>("current_s", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
            std::bind(&SystemDynamicsNode::compute_dynamics, this));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dynamics_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr s_pub_;

    double v_max_, omega_max_, update_rate_;
    double x_, y_, psi_, v_, omega_, s_;
    double v_s_=0.1;

    //get state values [x, y, psi, s] and linear velocity

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        psi_ = msg->pose.pose.orientation.z;
        v_ = msg->twist.twist.linear.x;
        omega_ = msg->twist.twist.angular.z;
    }

    double clamp(double value, double min_value, double max_value) {
        return std::max(min_value, std::min(value, max_value));
    }

    void compute_dynamics() {
        double A[4][4] = {{0, 0, -v_ * sin(psi_), 0},
                          {0, 0, v_ * cos(psi_), 0},
                          {0, 0, 0, 0},
                          {0, 0, 0, 0}};
        
        double B[4][3] = {{cos(psi_), 0, 0},
                          {sin(psi_), 0, 0},
                          {0, 1, 0},
                          {0, 0, 1}};
        
        // Compute state derivative \dot{ξ} = Aξ + Bu
        double x_dot = A[0][2] * psi_ + B[0][0] * v_; //x_dot = -v_ * sin(psi_) * psi_ + cos(psi_)*v_
        double y_dot = A[1][2] * psi_ + B[1][0] * v_; //y_dot = vcos(psi_) * psi_ + sin(psi_)*v_
        double psi_dot = B[2][1] * omega_; // psi_dot = omega_
        double s_dot = B[3][2] * v_s_; // s_dot = v_s_

        v_ = clamp(v_, -v_max_, v_max_);
        omega_ = clamp(omega_, -omega_max_, omega_max_);
        s_+= v_s_;

        std_msgs::msg::Float64MultiArray msg;
        msg.data = {x_, y_, psi_, s_, x_dot, y_dot, psi_dot, s_dot};
        dynamics_pub_->publish(msg);

        auto s_msg = std_msgs::msg::Float64();
        s_msg.data = s_;
        s_pub_->publish(s_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SystemDynamicsNode>());
    rclcpp::shutdown();
    return 0;
}
