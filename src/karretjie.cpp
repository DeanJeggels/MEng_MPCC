#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <mpc/NLMPC.hpp>
#include <mpc/Utils.hpp>

constexpr int Tnx = 4; // Number of states [x, y, psi, s]
constexpr int Tnu = 3; // Number of inputs [v, w, v_s]
constexpr int Tny = 4; // Number of outputs (same as states)
constexpr int Tph = 15; // Predicion horizon
constexpr int Tch = 15; // Control horizon 
constexpr int Tineq = 6*(Tph+1); // inequality constraints 
constexpr int Teq = 0; // equality constraints

mpc::mat<Tnx, Tnx> A(Tnx,Tnx);
mpc::mat<Tnx, Tnu> B(Tnx,Tnu);
mpc::mat<Tnx,Tnx> Ad(Tnx,Tnx);
mpc::mat<Tnx,Tnu> Bd(Tnx,Tnu);


class Karretjie : public rclcpp::Node {
public:
    Karretjie() : Node("karretjie"), update_rate_(10.0), dt_(1.0 / update_rate_) {
        // Initial state [x, y, psi, s]
        state_.setZero();
        
        RCLCPP_INFO(this->get_logger(), "Initializing Karretjie node with:");
        RCLCPP_INFO(this->get_logger(), "  Update rate: %.1f Hz", update_rate_);
        RCLCPP_INFO(this->get_logger(), "  Initial state: x=%.2f, y=%.2f, psi=%.2f, s=%.2f", 
                   state_[0], state_[1], state_[2], state_[3]);

        cmd_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mpcc/result_cmd", 10,
            std::bind(&Karretjie::cmd_callback, this, std::placeholders::_1)
        );
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/karretjie/odom", 10);

        state_ << 0.0, 0.0, 0.0, 0.0;
        RCLCPP_INFO(this->get_logger(), "Reset initial state to: x=%.2f, y=%.2f", state_[0], state_[1]);

    }

private:

    mpc::cvec<Tnx> state_, next_state;

    void cmd_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "\n--- New Command Received ---");
        RCLCPP_INFO(this->get_logger(), "Received control inputs: v=%.3f, omega=%.3f, v_s=%.3f",
                   msg->twist.twist.linear.x, 
                   msg->twist.twist.angular.z,
                   msg->twist.twist.linear.y);

        Eigen::Vector3d u(msg->twist.twist.linear.x, 
                         msg->twist.twist.angular.z,
                         msg->twist.twist.linear.y);

        dt_ = msg->twist.twist.linear.z;
        
        RCLCPP_INFO(this->get_logger(), "Previous state: x=%.3f, y=%.3f, psi=%.3f, s=%.3f",
                    state_[0], state_[1], state_[2], state_[3]);

        stateEquation(next_state, state_, u, -1);
        state_ = next_state;

        RCLCPP_INFO(this->get_logger(), "New state: x=%.3f, y=%.3f, psi=%.3f, s=%.3f",
                    state_[0], state_[1], state_[2], state_[3]);

        // Publish as Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = state_(0);
        odom_msg.pose.pose.position.y = state_(1);
        odom_msg.pose.pose.position.z = state_(3);


        // Convert yaw (psi) to quaternion
        double psi = state_(2);
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = psi;
        odom_msg.pose.pose.orientation.w = cos(psi / 2.0);

        odom_pub_->publish(odom_msg);
    }

    void stateEquation(mpc::cvec<Tnx>& dx, const mpc::cvec<Tnx>& x, const mpc::cvec<Tnu>& u, const unsigned int&) {

        double psi = x(2);
        double v = u(0);
        double omega = u(1);
        double v_s = u(2);

        RCLCPP_INFO(this->get_logger(), "State equation calculation:");
        RCLCPP_INFO(this->get_logger(), "Current state: x=%.3f, y=%.3f, psi=%.3f, s=%.3f",
                    x[0], x[1], x[2], x[3]);
        RCLCPP_INFO(this->get_logger(), "Control inputs: v=%.3f, omega=%.3f, v_s=%.3f",
                    u[0], u[1], u[2]);

    
        dx(0) = x(0) + v * std::cos(psi) * dt_;
        dx(1) = x(1) + v * std::sin(psi)* dt_;
        dx(2) = x(2) + omega* dt_;
        dx(3) = x(3) + v_s* dt_;


        // dx= Ad * x + Bd * u;

        RCLCPP_INFO(this->get_logger(), "Calculated state derivative: dx=%.3f, dy=%.3f, dpsi=%.3f, ds=%.3f",
        dx[0], dx[1], dx[2], dx[3]);

        // print out dx
        // std::cout << "dx: " << dx << std::endl;

    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    double v_max_, omega_max_, update_rate_, dt_;
    double v_s_max;
    double x_min_, x_max_, y_min_, y_max_, s_max_;
    double x_, y_, psi_, v_, omega_;
    double v_s_ = 0.0;
    double s_ = 0.0;
    double xd_, yd_, dx_ds_, dy_ds_;

};





int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Karretjie>());
    rclcpp::shutdown();
    return 0;
}
