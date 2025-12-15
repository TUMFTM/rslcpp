// Copyright 2025 Simon Sagmeister
#pragma once
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sstream>
#include <string>
#include <vector>
namespace rslcpp::helper_nodes
{
class BagRecorder : public rclcpp::Node
{
public:
  explicit BagRecorder(rclcpp::NodeOptions options) : Node("BagRecorder", options)
  {
    // declare node parameter
    this->declare_parameter("log_dir", "rslcpp/logs");
    this->declare_parameter<std::vector<std::string>>("ignore_topic_list", {""});
    this->declare_parameter<std::vector<std::string>>("record_topic_list", {""});

    // store the list of topics to be ignored
    ignore_list_ = this->get_parameter("ignore_topic_list").as_string_array();
    ignore_list_.push_back("/rosout");
    // store the list of topics to be recorded. If list is empty all topics are recorded.
    record_list_ = this->get_parameter("record_topic_list").as_string_array();

    // initialize the subscriber and the bag writer
    create_writer();
    update_subscriber();

    this->declare_parameter("topic_scan_period_s", 0.1);
    timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::microseconds(
        static_cast<int>(1e6 * this->get_parameter("topic_scan_period_s").as_double())),
      std::bind(&BagRecorder::update_subscriber, this));
  }

private:
  /**
   * creates the writer object for the specified ros bag
   */
  void create_writer()
  {
    std::filesystem::path log_dir = this->get_parameter("log_dir").as_string();
    const rosbag2_storage::StorageOptions storage_options(
      {(log_dir / log_dir.filename()).string(), "mcap"});
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(), rmw_get_serialization_format()});

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage_options, converter_options);
  }
  /**
   * initializes all ros subscriber and stores them into a map
   */
  void update_subscriber(void)
  {
    if (all_topics_discovered) return;
    // if record list is empty get all topics and types. Keep searching.
    if (record_list_.front() == "") {
      record_topic_names_and_types = this->get_topic_names_and_types();
      create_subscriber();
      return;
    }
    // if record list is not empty search until all topics are discovered.
    std::map<std::string, std::vector<std::string>> all_topic_names_and_types =
      this->get_topic_names_and_types();
    for (const auto & topic : record_list_) {
      if (all_topic_names_and_types.find(topic) == all_topic_names_and_types.end()) {
        RCLCPP_WARN_STREAM(
          this->get_logger(), "BagRecorder: Topic " << topic << " not yet discovered.");
        continue;
      }
      record_topic_names_and_types.insert_or_assign(topic, all_topic_names_and_types.at(topic));
    }
    create_subscriber();
    if (record_topic_names_and_types.size() == record_list_.size()) all_topics_discovered = true;
  }
  void create_subscriber(void)
  {
    // create a generic subscriber for each topic that should be loged
    for (const auto & [topic, type] : record_topic_names_and_types) {
      if (std::find(ignore_list_.begin(), ignore_list_.end(), topic) != ignore_list_.end())
        continue;
      if (sub_map_.find(topic) != sub_map_.end()) continue;
      auto publishers_info = this->get_publishers_info_by_topic(topic);
      if (!publishers_info.empty()) {
        sub_map_[topic] = this->create_generic_subscription(
          topic, type.front(), rclcpp::QoS(publishers_info[0].qos_profile()),
          [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> message) {
            this->on_subscription(topic, type.front(), message);
          });
      }
    }
  }
  void on_subscription(
    std::string topic, std::string msg_type, std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    writer_->write(msg, topic, msg_type, this->now());
  }

private:
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> sub_map_{};
  std::vector<std::string> ignore_list_{};
  std::vector<std::string> record_list_{};

  std::unique_ptr<rosbag2_cpp::Writer> writer_{};

  rclcpp::TimerBase::SharedPtr timer_{};
  std::map<std::string, std::vector<std::string>> record_topic_names_and_types{};
  bool all_topics_discovered{false};
};
}  // namespace rslcpp::helper_nodes
