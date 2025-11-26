// mpcc_bag_manager.cpp
#include <atomic>
#include <csignal>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

// rosbag2 writer API (ROS 2 Humble)
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"

class MPCCBagManager : public rclcpp::Node
{
public:
  MPCCBagManager() : Node("mpcc_bag_manager")
  {
    // Recording parameters
    declare_parameter<std::string>("bag_name", "mpcc_paths_bag");
    declare_parameter<std::string>("storage_id", "sqlite3");
    declare_parameter<std::string>("serialization_format", "cdr");
    declare_parameter<std::vector<std::string>>(
      "topics", {"/mpcc_obst/actual_path", "/mpcc_obst/predicted_path"});

    // Playback parameters
    declare_parameter<std::string>("play_bag_uri", "mpcc_paths_bag");
    declare_parameter<double>("play_rate", 1.0);
    declare_parameter<bool>("play_loop", false);
    declare_parameter<double>("play_start_offset", 0.0); // seconds
    declare_parameter<std::vector<std::string>>(
      "play_remaps", std::vector<std::string>{}); // e.g. "/from:=/to"

    // Services
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "start_bag",
      std::bind(&MPCCBagManager::startBag, this, std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "stop_bag",
      std::bind(&MPCCBagManager::stopBag, this, std::placeholders::_1, std::placeholders::_2));

    play_srv_ = create_service<std_srvs::srv::Trigger>(
      "play_bag",
      std::bind(&MPCCBagManager::playBag, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "MPCC Bag Manager ready. Services: /start_bag, /stop_bag, /play_bag");
  }

  ~MPCCBagManager() override
  {
    stopPlaybackProcess();
    std::lock_guard<std::mutex> lk(rec_mtx_);
    subs_.clear();
    writer_.reset();
  }

private:
  struct SubHandle {
    std::string topic;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub;
  };

  // ---------- Recording ----------
  void startBag(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::lock_guard<std::mutex> lk(rec_mtx_);
    if (recording_) {
      res->success = false;
      res->message = "Already recording";
      return;
    }

    get_parameter("bag_name", bag_name_);
    get_parameter("storage_id", storage_id_);
    get_parameter("serialization_format", serialization_format_);
    get_parameter("topics", topics_);

    try {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_name_;
      storage_options.storage_id = storage_id_;

      rosbag2_cpp::ConverterOptions converter_options;
      converter_options.input_serialization_format = serialization_format_;
      converter_options.output_serialization_format = serialization_format_;

      writer_ = std::make_unique<rosbag2_cpp::Writer>();
      writer_->open(storage_options, converter_options);

      subs_.clear();
      for (const auto & t : topics_) {
        // Ensure topic is registered in the bag
        rosbag2_storage::TopicMetadata md;
        md.name = t;
        md.type = "nav_msgs/msg/Path";
        md.serialization_format = serialization_format_;
        writer_->create_topic(md);

        // Subscriber that serializes and writes messages with proper topic + type
        SubHandle h;
        h.topic = t;
        h.sub = this->create_subscription<nav_msgs::msg::Path>(
          t, rclcpp::SystemDefaultsQoS(),
          [this, t](const nav_msgs::msg::Path::SharedPtr msg)
          {
            rclcpp::SerializedMessage sm;
            path_ser_.serialize_message(msg.get(), &sm);

            // Timestamp: prefer header stamp; fallback to now()
            rcutils_time_point_value_t ts_ns =
              static_cast<rcutils_time_point_value_t>(msg->header.stamp.sec) * 1000000000ll +
              static_cast<rcutils_time_point_value_t>(msg->header.stamp.nanosec);
            if (ts_ns == 0) {
              ts_ns = static_cast<rcutils_time_point_value_t>(this->get_clock()->now().nanoseconds());
            }
            rclcpp::Time stamp_ts(ts_ns);

            // IMPORTANT: use the overload that includes topic_name + type
            writer_->write(
              sm,                     // payload
              t,                      // topic_name
              "nav_msgs/msg/Path",    // type
              stamp_ts                // timestamp
            );
          });

        subs_.push_back(std::move(h));
      }

      recording_ = true;
      res->success = true;
      std::ostringstream oss;
      oss << "Recording started to '" << bag_name_ << "' with topics:";
      for (auto & t : topics_) oss << " " << t;
      res->message = oss.str();
      RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    } catch (const std::exception & e) {
      res->success = false;
      res->message = std::string("Failed to start recording: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    }
  }

  void stopBag(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::lock_guard<std::mutex> lk(rec_mtx_);
    if (!recording_) {
      res->success = false;
      res->message = "Not recording";
      return;
    }

    try {
      subs_.clear();
      writer_.reset();
      recording_ = false;
      res->success = true;
      res->message = "Recording stopped";
      RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    } catch (const std::exception & e) {
      res->success = false;
      res->message = std::string("Failed to stop recording: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    }
  }

  // ---------- Playback ----------
  void playBag(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::lock_guard<std::mutex> lk(play_mtx_);

    if (play_pid_ > 0) {
      stopPlaybackProcess();
    }

    std::string bag_uri;
    double rate;
    bool loop;
    double start_offset;
    std::vector<std::string> remaps;

    get_parameter("play_bag_uri", bag_uri);
    get_parameter("play_rate", rate);
    get_parameter("play_loop", loop);
    get_parameter("play_start_offset", start_offset);
    get_parameter("play_remaps", remaps);

    std::ostringstream cmd;
    cmd << "ros2 bag play " << escapeArg(bag_uri)
        << " -r " << rate;
    if (loop) cmd << " -l";
    if (start_offset > 0.0) {
      cmd << " --start-offset " << start_offset;
    }
    for (const auto & remap : remaps) {
      cmd << " --remap " << escapeArg(remap);
    }

    std::string full_cmd = cmd.str() + " & echo $!";
    FILE* pipe = popen(full_cmd.c_str(), "r");
    if (!pipe) {
      res->success = false;
      res->message = "Failed to start rosbag playback (popen)";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }

    char buffer[128];
    std::string pid_str;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      pid_str += buffer;
    }
    pclose(pipe);

    try {
      play_pid_ = static_cast<pid_t>(std::stol(pid_str));
    } catch (...) {
      play_pid_ = -1;
    }

    if (play_pid_ <= 0) {
      res->success = false;
      res->message = "Failed to obtain playback process PID";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }

    res->success = true;
    res->message = std::string("Playback started (PID ")
                 + std::to_string(play_pid_) + "): " + cmd.str();
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void stopPlaybackProcess()
  {
    if (play_pid_ > 0) {
      ::kill(play_pid_, SIGINT);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      ::kill(play_pid_, SIGTERM);
      play_pid_ = -1;
      RCLCPP_INFO(get_logger(), "Playback stopped");
    }
  }

  static std::string escapeArg(const std::string & s)
  {
    std::ostringstream oss;
    oss << "'";
    for (char c : s) {
      if (c == '\'') {
        oss << "'\\''";
      } else {
        oss << c;
      }
    }
    oss << "'";
    return oss.str();
  }

private:
  // Recording state
  std::mutex rec_mtx_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  bool recording_{false};
  std::string bag_name_, storage_id_, serialization_format_;
  std::vector<std::string> topics_;
  std::vector<SubHandle> subs_;
  rclcpp::Serialization<nav_msgs::msg::Path> path_ser_;

  // Playback state
  std::mutex play_mtx_;
  std::atomic<pid_t> play_pid_{-1};

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr play_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCCBagManager>());
  rclcpp::shutdown();
  return 0;
}
