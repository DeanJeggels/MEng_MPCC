#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <algorithm>
#include <vector>
#include <chrono>
#include <cmath>

class ActualPathPublisher : public rclcpp::Node
{
public:
  ActualPathPublisher()
  : Node("actual_path_publisher"), robot_name_("mpcc")
  {
    model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/model_states", 10,
      std::bind(&ActualPathPublisher::odomCallback, this, std::placeholders::_1));

    actual_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpcc/actual_path", 10);
  }

private:
  void odomCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    auto it = std::find(msg->name.begin(), msg->name.end(), robot_name_);
    if (it == msg->name.end()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Robot model '%s' not found in model_states", 
                          robot_name_.c_str());
      return;
    }

    size_t robot_index = std::distance(msg->name.begin(), it);
    auto& pose = msg->pose[robot_index];

    double x = pose.position.x;
    double y = pose.position.y;
    double qw = pose.orientation.w;
    double qz = pose.orientation.z;
    double yaw = 2.0 * std::atan2(qz, qw);

    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = this->now();
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.orientation.z = std::sin(yaw / 2.0);
    ps.pose.orientation.w = std::cos(yaw / 2.0);

    actual_path_.push_back(ps);

    nav_msgs::msg::Path path_msg;
    path_msg.header = ps.header;
    path_msg.poses = actual_path_;

    actual_pub_->publish(path_msg);
  }

  std::string robot_name_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_pub_;
  std::vector<geometry_msgs::msg::PoseStamped> actual_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActualPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
