// include/mpcc/linear_spline_visualizer.hpp
#ifndef MPCC_LINEAR_SPLINE_VISUALIZER_HPP
#define MPCC_LINEAR_SPLINE_VISUALIZER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mpcc/linear_spline.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

class LinearSplineVisualizer : public rclcpp::Node {
public:
    LinearSplineVisualizer();

private:
    void load_and_publish();
    void timer_callback();
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    voyager_mpcc::LinearSpline spline_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr s_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr original_path_pub_;
    double resolution_ = 0.1; 
};

#endif // MPCC_LINEAR_SPLINE_VISUALIZER_HPP
