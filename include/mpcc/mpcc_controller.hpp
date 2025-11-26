#ifndef VOYAGER_MPCC_CONTROLLER_HPP
#define VOYAGER_MPCC_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "mpcc/linear_spline.hpp"
#include "mpcc/optimization_problem.hpp"

namespace voyager_mpcc {

class MPCCController : public rclcpp::Node {
public:
    // Constructor
    MPCCController();

private:
    // Callback for odometry data
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Main control loop
    void controlLoop();
    
    // Load parameters from ROS parameter server
    void loadParameters();
    
    // ROS publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Trajectory and controller objects
    LinearSpline spline_;
    std::unique_ptr<OptimizationProblem> optimizer_;
    MPCCParameters mpcc_params_;
    
    // Current state
    double current_x_;
    double current_y_;
    double current_psi_;
    double current_s_;
    bool state_initialized_;
    
    // Parameters
    std::string trajectory_file_;
    double control_frequency_;
    int prediction_horizon_;
};

} // namespace voyager_mpcc

#endif // VOYAGER_MPCC_CONTROLLER_HPP