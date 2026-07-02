// Copyright 2025 Marcel Weinmann
#pragma once
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rslcpp/utilities.hpp>
#include <rslcpp_dynamic_job/backend.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#if ROS_DISTRO_HUMBLE
#define GET_TIMESTAMP(msg) (msg->time_stamp)
#else
#define GET_TIMESTAMP(msg) (msg->send_timestamp)
#endif

#if ROS_DISTRO_HUMBLE
#define get_qos(metadata) (rclcpp::QoS(1).best_effort().durability_volatile().keep_last(1))
#else
#include <rosbag2_storage/qos.hpp>
#define get_qos(metadata)                                              \
  ((metadata).offered_qos_profiles.empty()                             \
     ? rclcpp::QoS(1).best_effort().durability_volatile().keep_last(1) \
     : rosbag2_storage::from_rclcpp_qos_vector((metadata).offered_qos_profiles).at(0))
#endif
namespace rslcpp::helper_nodes
{
class BagPlayer : public rclcpp::Node
{
public:
  explicit BagPlayer(rclcpp::NodeOptions options) : Node("BagPlayer", options)
  {
    // declare node parameter
    this->declare_parameter("bag_file_path", "");
    this->declare_parameter("pub_intervall_us", 100);
    this->declare_parameter("pub_progress", false);
    this->declare_parameter("topics", std::vector<std::string>());
    this->declare_parameter("start_offset", 0.0);
    pub_progress_enabled_ = this->get_parameter("pub_progress").as_bool();
    topics_filter_ = this->get_parameter("topics").as_string_array();
    topics_filter_.erase(
      std::remove(topics_filter_.begin(), topics_filter_.end(), ""), topics_filter_.end());
    topics_.reserve(topics_filter_.size());
    topics_.insert(topics_filter_.begin(), topics_filter_.end());
    start_offset_s_ = std::max(0.0, this->get_parameter("start_offset").as_double());

    std::string bag_path = this->get_parameter("bag_file_path").as_string();
    std::filesystem::path filePath(bag_path);

    // create a reader and all publisher for the bag
    if (std::filesystem::exists(filePath)) {
      valid_bag_storage_ = true;
      create_reader(bag_path);
      apply_topics_filter();
      seek_start_offset();
      init_publisher();

      // read the first message and store initial time for the job handler
      if (rosbag_->has_next()) {
        last_msg_ = rosbag_->read_next();
        initial_time_ = get_last_message_timestamp();
        dynamic_job::set_initial_time(initial_time_);
      }

      // set up the progress publisher if enabled
      if (pub_progress_enabled_) {
        bag_duration_s_ = get_bag_duration()->seconds();
        pub_progress_ = this->create_publisher<std_msgs::msg::Float32>("/rslcpp/progress", 1);
      }
    } else {
      std::cout << "RSLCPP | BagPlayer | The specified rosbag file does not exist." << std::endl;
      std::cout << bag_path << std::endl;
    }

    // create timer to publish the bag
    timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::microseconds(this->get_parameter("pub_intervall_us").as_int()),
      std::bind(&BagPlayer::timer_callback, this));
  }
  /**
   * Returns the timestamp of the first message in the bag
   *
   * \return the first timestamp in the bag
   */
  const rclcpp::Time get_initial_bag_time(void)
  {
    return valid_bag_storage_ ? initial_time_ : rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  /**
   * Returns the duration of the entire ros bag
   *
   * \return a const reference to the duration of the ros bag
   */
  const std::shared_ptr<rclcpp::Duration> get_bag_duration(void)
  {
    auto metadata = rosbag_->get_metadata();
    return std::make_shared<rclcpp::Duration>(metadata.duration);
  }

private:
  /**
   * creates the reader object for the specified ros bag
   */
  void create_reader(std::string bag_path)
  {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    rosbag_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    rosbag_->open(storage_options);
  }
  /**
   * initializes all ros publishers and stores them into a map
   */
  void init_publisher(void)
  {
    // get all topics and types
    std::vector<rosbag2_storage::TopicMetadata> metadata_vec = rosbag_->get_all_topics_and_types();
    pub_vec_.reserve(topics_.empty() ? metadata_vec.size() : topics_.size());

    // create a generic publisher for each topic
    for (const rosbag2_storage::TopicMetadata & metadata : metadata_vec) {
      if (!should_play_topic(metadata.name)) {
        continue;
      }
      pub_vec_[metadata.name] =
        this->create_generic_publisher(metadata.name, metadata.type, get_qos(metadata));
    }
  }
  /**
   * Restricts the reader to the requested playback topics.
   */
  void apply_topics_filter(void)
  {
    if (topics_filter_.empty()) {
      return;
    }

    rosbag2_storage::StorageFilter storage_filter;
    storage_filter.topics = topics_filter_;
    rosbag_->set_filter(storage_filter);
  }
  /**
   * Seeks the reader to bag start + configured offset.
   */
  void seek_start_offset(void)
  {
    const auto metadata = rosbag_->get_metadata();
    const auto bag_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      metadata.starting_time.time_since_epoch());
    const auto start_offset_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
        start_offset_s_));
    const auto seek_timestamp = bag_start_ns + start_offset_ns;
    initial_time_ = rclcpp::Time(
      static_cast<rcutils_time_point_value_t>(seek_timestamp.count()), RCL_ROS_TIME);
    if (start_offset_s_ <= 0.0) {
      return;
    }

    rosbag_->seek(static_cast<rcutils_time_point_value_t>(seek_timestamp.count()));
  }
  /**
   * Checks whether the topic is included in the configured playback set.
   */
  bool should_play_topic(const std::string & topic) const
  {
    return topics_.empty() || topics_.find(topic) != topics_.end();
  }
  /**
   * Returns the duration of the entire ros bag
   *
   * \return the timestamp of the next message in the bag
   */
  const rclcpp::Time get_last_message_timestamp(void)
  {
    if (last_msg_ != nullptr) {
      int32_t sec = static_cast<int32_t>(GET_TIMESTAMP(last_msg_) / 1e9);
      uint32_t nanosec =
        static_cast<uint32_t>(GET_TIMESTAMP(last_msg_) % static_cast<uint32_t>(1e9));
      return rclcpp::Time(sec, nanosec, RCL_ROS_TIME);
    } else {
      throw std::bad_weak_ptr();
    }
  }
  /**
   * Publish the last message in the buffer via IPC
   */
  void publish_last_message(void)
  {
    rclcpp::SerializedMessage serialized_msg(*last_msg_->serialized_data);
    const auto publisher = pub_vec_.find(last_msg_->topic_name);
    if (publisher != pub_vec_.end()) {
      publisher->second->publish(serialized_msg);
    }
    last_msg_ = nullptr;
  }
  /**
   * Publish all messages with a bag timestamp smaller than the sim clock
   */
  void timer_callback(void)
  {
    // only check for messages if a valid bag was found
    if (!valid_bag_storage_) {
      return;
    }

    // create a lambda that publishes the last message if it is in the current intervall
    rclcpp::Time last_timestamp;
    const auto publish_message = [this, &last_timestamp] {
      last_timestamp = get_last_message_timestamp();
      if (last_timestamp < this->get_clock()->now()) {
        publish_last_message();
      }
    };

    // if there is a message in the buffer publish it
    if (last_msg_ != nullptr) {
      publish_message();
    }

    // publish all remaining messages in the intervall
    while (rosbag_->has_next() && last_timestamp < this->get_clock()->now()) {
      last_msg_ = rosbag_->read_next();
      publish_message();
    }

    // publish the current time progress in relation to the bag length
    if (pub_progress_enabled_) {
      publish_progress();
    }

    // publish the information that all messages in the bag have been published
    if (!rosbag_->has_next()) {
      auto message = std_msgs::msg::Bool();
      message.data = true;
      pub_status_->publish(message);
    }
  }
  /**
   * Publish the elapsed time in relation to the bag length as a value in [0, 1]
   */
  void publish_progress(void)
  {
    const double elapsed = (this->get_clock()->now() - initial_time_).seconds();

    auto message = std_msgs::msg::Float32();
    message.data = bag_duration_s_ > 0.0
                     ? static_cast<float>(std::clamp(elapsed / bag_duration_s_, 0.0, 1.0))
                     : 0.0f;
    pub_progress_->publish(message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_ =
    this->create_publisher<std_msgs::msg::Bool>("/rslcpp/shutdown", 1);
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_progress_{};

  rclcpp::TimerBase::SharedPtr timer_{};
  std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr> pub_vec_{};
  std::unique_ptr<rosbag2_cpp::Reader> rosbag_{};
  rosbag2_storage::SerializedBagMessageSharedPtr last_msg_{};

  rclcpp::Time initial_time_{0, 0, RCL_ROS_TIME};
  std::vector<std::string> topics_filter_{};
  std::unordered_set<std::string> topics_{};
  double start_offset_s_{0.0};
  double bag_duration_s_{0.0};
  bool pub_progress_enabled_{false};
  bool valid_bag_storage_{false};
};
}  // namespace rslcpp::helper_nodes
