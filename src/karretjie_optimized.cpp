// karretjie_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <mpc/Utils.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


constexpr int Tnx = 4;  // states: [x, y, psi, s]
constexpr int Tnu = 3;  // inputs: [v, omega, v_s]

class Karretjie : public rclcpp::Node {
public:
  Karretjie()
  : Node("karretjie"), update_rate_(10.0)
  {
    state_.setZero();
    state_ << 0.0, 0.0, 0.0, 0.0;

    cmd_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mpcc/result_cmd_karretjie", 10,
      std::bind(&Karretjie::cmdCallback, this, std::placeholders::_1)
    );
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/karretjie/odom", 10
    );
  }

private:
  void cmdCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Read control input
    mpc::cvec<Tnu> u;
    u << msg->twist.twist.linear.x,
         msg->twist.twist.angular.z,
         msg->twist.twist.linear.y;
         
    dt_ = 1.0 / update_rate_;

    RCLCPP_INFO(this->get_logger(), "u=[%.3f, %.3f, %.3f]",
                u(0), u(1), u(2));
    // Advance state
    mpc::cvec<Tnx> next;
    propagateState(next, state_, u, dt_);
    state_ = next;

    RCLCPP_INFO(this->get_logger(), "state=[%.3f, %.3f, %.3f, %.3f]",
                state_(0), state_(1), state_(2), state_(3));

    // Publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.header.frame_id   = "odom";
    odom.child_frame_id    = "base_link";
    odom.pose.pose.position.x = state_(0);
    odom.pose.pose.position.y = state_(1);
    odom.pose.pose.position.z = state_(3);

    // Convert yaw angle to quaternion using tf2
    tf2::Quaternion q;
    q.setRPY(0, 0, state_(2));  // roll=0, pitch=0, yaw=state_(2)
    odom.pose.pose.orientation = tf2::toMsg(q);
    //Cout the odom message
    RCLCPP_INFO(this->get_logger(), "Publishing odom: position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w);
    odom_pub_->publish(odom);
  }

  void propagateState(mpc::cvec<Tnx>& x_next,
                    const mpc::cvec<Tnx>& x,
                    const mpc::cvec<Tnu>& u,
                    double dt)
{
    double psi = x(2);
    x_next(0) = x(0) + u(0) * std::cos(psi) * dt;
    x_next(1) = x(1) + u(0) * std::sin(psi) * dt;
    x_next(2) = x(2) + u(1) * dt;
    x_next(3) = x(3) + u(2) * dt;

}


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  mpc::cvec<Tnx> state_;
  double update_rate_, dt_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Karretjie>());
  rclcpp::shutdown();
  return 0;
}
