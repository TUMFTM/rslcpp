// Copyright 2025 Marcel Weinmann
#pragma once
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rslcpp/utilities.hpp>
#include <rslcpp_dynamic_job/backend.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
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

    std::string bag_path = this->get_parameter("bag_file_path").as_string();
    std::filesystem::path filePath(bag_path);

    // create a reader and all publisher for the bag
    if (std::filesystem::exists(filePath)) {
      valid_bag_storage_ = true;
      create_reader(bag_path);
      init_publisher();

      // read the first message and store initial time for the job handler
      if (rosbag_->has_next()) {
        last_msg_ = rosbag_->read_next();
        initial_time_ = get_last_message_timestamp();
        dynamic_job::set_initial_time(initial_time_);
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

    // create a generic publisher for each topic
    for (const rosbag2_storage::TopicMetadata & metadata : metadata_vec) {
      pub_vec_[metadata.name] =
        this->create_generic_publisher(metadata.name, metadata.type, get_qos(metadata));
    }
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
    pub_vec_[last_msg_->topic_name]->publish(serialized_msg);
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

    // publish the information that all messages in the bag have been published
    if (!rosbag_->has_next()) {
      auto message = std_msgs::msg::Bool();
      message.data = true;
      pub_status_->publish(message);
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_ =
    this->create_publisher<std_msgs::msg::Bool>("/rslcpp/shutdown", 1);

  rclcpp::TimerBase::SharedPtr timer_{};
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr> pub_vec_{};
  std::unique_ptr<rosbag2_cpp::Reader> rosbag_{};
  rosbag2_storage::SerializedBagMessageSharedPtr last_msg_{};

  rclcpp::Time initial_time_;
  bool valid_bag_storage_{false};
};
}  // namespace rslcpp::helper_nodes
