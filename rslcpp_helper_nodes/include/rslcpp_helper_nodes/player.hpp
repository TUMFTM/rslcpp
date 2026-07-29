// Copyright 2025 Marcel Weinmann
#pragma once
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
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
    parse_qos_overrides();

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
        this->create_generic_publisher(metadata.name, metadata.type, resolve_qos(metadata));
    }

    // warn about QoS overrides that do not match any of the played topics
    for (const auto & override_entry : qos_overrides_) {
      if (pub_vec_.find(override_entry.first) == pub_vec_.end()) {
        RCLCPP_WARN_STREAM(
          this->get_logger(), "BagPlayer: The QoS override for topic '"
                                << override_entry.first
                                << "' does not match any of the played topics.");
      }
    }
  }
  /**
   * Reads the per topic QoS overrides from the node parameters.
   *
   * The parameters use the same layout as the QoS profile override file of
   * `ros2 bag play --qos-profile-overrides-path`:
   *
   *   qos_overrides:
   *     /some/topic:
   *       reliability: reliable
   *       durability: transient_local
   *       history: keep_last
   *       depth: 10
   *
   * The additional 'publisher' level of the rclcpp QoS overriding feature is accepted as well,
   * i.e. 'qos_overrides./some/topic.publisher.reliability' is equivalent to the layout above.
   */
  void parse_qos_overrides(void)
  {
    const std::string prefix = "qos_overrides.";
    const std::string publisher_level = "publisher.";
    const std::string subscription_level = "subscription.";
    for (const auto & [name, value] :
         this->get_node_parameters_interface()->get_parameter_overrides()) {
      if (name.rfind(prefix, 0) != 0) {
        continue;
      }

      // topic names cannot contain a '.', therefore the first '.' separates topic and policy
      const std::string entry = name.substr(prefix.size());
      const std::size_t separator = entry.find('.');
      if (separator == std::string::npos || separator == 0 || separator + 1 == entry.size()) {
        throw rclcpp::exceptions::InvalidParametersException(
          "BagPlayer: '" + name +
          "' is not a valid QoS override, expected 'qos_overrides.<topic>.<policy>'.");
      }
      std::string policy = entry.substr(separator + 1);

      // the player has no subscriptions, so overrides for them are none of its business
      if (policy.rfind(subscription_level, 0) == 0) {
        continue;
      }
      if (policy.rfind(publisher_level, 0) == 0) {
        policy = policy.substr(publisher_level.size());
      }
      if (qos_policy_names().find(policy) == qos_policy_names().end()) {
        throw rclcpp::exceptions::InvalidParametersException(
          "BagPlayer: '" + policy + "' is not a supported QoS policy (parameter '" + name + "').");
      }

      // declaring the parameter makes the override visible for parameter introspection
      if (!this->has_parameter(name)) {
        this->declare_parameter(name, value);
      }
      qos_overrides_[entry.substr(0, separator)].emplace(policy, this->get_parameter(name));
    }
  }
  /**
   * Returns all QoS policies that can be overridden for a topic
   *
   * \return a const reference to the set of supported QoS policy names
   */
  static const std::unordered_set<std::string> & qos_policy_names(void)
  {
    static const std::unordered_set<std::string> names{
      "history",
      "depth",
      "reliability",
      "durability",
      "liveliness",
      "avoid_ros_namespace_conventions",
      "deadline",
      "deadline.sec",
      "deadline.nsec",
      "lifespan",
      "lifespan.sec",
      "lifespan.nsec",
      "liveliness_lease_duration",
      "liveliness_lease_duration.sec",
      "liveliness_lease_duration.nsec"};
    return names;
  }
  /**
   * Applies the configured QoS overrides on top of the QoS profile recorded in the bag
   *
   * \return the QoS profile the publisher of the topic should offer
   */
  rclcpp::QoS resolve_qos(const rosbag2_storage::TopicMetadata & metadata)
  {
    rclcpp::QoS qos = get_qos(metadata);
    const auto override_entry = qos_overrides_.find(metadata.name);
    if (override_entry == qos_overrides_.end()) {
      return qos;
    }
    const std::map<std::string, rclcpp::Parameter> & policies = override_entry->second;

    // the depth is applied before the history so that an explicit 'keep_all' is not overwritten
    const auto depth = policies.find("depth");
    if (depth != policies.end()) {
      qos.keep_last(static_cast<size_t>(std::max<int64_t>(0, depth->second.as_int())));
    }
    const auto history = policies.find("history");
    if (history != policies.end()) {
      qos.history(parse_qos_policy<rmw_qos_history_policy_t>(
        history->second, {{"system_default", RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT},
                          {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
                          {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL}}));
    }
    const auto reliability = policies.find("reliability");
    if (reliability != policies.end()) {
      qos.reliability(parse_qos_policy<rmw_qos_reliability_policy_t>(
        reliability->second, {{"system_default", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT},
                              {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
                              {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}}));
    }
    const auto durability = policies.find("durability");
    if (durability != policies.end()) {
      qos.durability(parse_qos_policy<rmw_qos_durability_policy_t>(
        durability->second, {{"system_default", RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT},
                             {"transient_local", RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
                             {"volatile", RMW_QOS_POLICY_DURABILITY_VOLATILE}}));
    }
    const auto liveliness = policies.find("liveliness");
    if (liveliness != policies.end()) {
      qos.liveliness(parse_qos_policy<rmw_qos_liveliness_policy_t>(
        liveliness->second, {{"system_default", RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT},
                             {"automatic", RMW_QOS_POLICY_LIVELINESS_AUTOMATIC},
                             {"manual_by_topic", RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC}}));
    }
    const auto deadline = parse_qos_duration(policies, "deadline");
    if (deadline.has_value()) {
      qos.deadline(deadline.value());
    }
    const auto lifespan = parse_qos_duration(policies, "lifespan");
    if (lifespan.has_value()) {
      qos.lifespan(lifespan.value());
    }
    const auto lease_duration = parse_qos_duration(policies, "liveliness_lease_duration");
    if (lease_duration.has_value()) {
      qos.liveliness_lease_duration(lease_duration.value());
    }
    const auto avoid_conventions = policies.find("avoid_ros_namespace_conventions");
    if (avoid_conventions != policies.end()) {
      qos.avoid_ros_namespace_conventions(avoid_conventions->second.as_bool());
    }

    RCLCPP_INFO_STREAM(
      this->get_logger(), "BagPlayer: Overriding the recorded QoS profile of topic '"
                            << metadata.name << "' with " << policies.size()
                            << " policy setting(s).");
    return qos;
  }
  /**
   * Converts the string value of a QoS policy parameter into its rmw policy value
   *
   * \return the rmw policy value matching the parameter value
   */
  template <typename PolicyT>
  static PolicyT parse_qos_policy(
    const rclcpp::Parameter & parameter, const std::map<std::string, PolicyT> & supported_values)
  {
    const auto value = supported_values.find(parameter.as_string());
    if (value != supported_values.end()) {
      return value->second;
    }

    std::string options;
    for (const auto & supported_value : supported_values) {
      options += (options.empty() ? "" : ", ") + supported_value.first;
    }
    throw rclcpp::exceptions::InvalidParameterValueException(
      "BagPlayer: '" + parameter.as_string() + "' is not a valid value for the parameter '" +
      parameter.get_name() + "', supported values are: " + options + ".");
  }
  /**
   * Reads a duration policy that is either given in seconds or as a 'sec' / 'nsec' pair
   *
   * \return the configured duration or std::nullopt if the policy is not overridden
   */
  static std::optional<rclcpp::Duration> parse_qos_duration(
    const std::map<std::string, rclcpp::Parameter> & policies, const std::string & policy)
  {
    const auto seconds = policies.find(policy);
    if (seconds != policies.end()) {
      return rclcpp::Duration::from_seconds(
        seconds->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
          ? static_cast<double>(seconds->second.as_int())
          : seconds->second.as_double());
    }

    const auto sec = policies.find(policy + ".sec");
    const auto nsec = policies.find(policy + ".nsec");
    if (sec == policies.end() && nsec == policies.end()) {
      return std::nullopt;
    }
    return rclcpp::Duration(
      static_cast<int32_t>(sec != policies.end() ? sec->second.as_int() : 0),
      static_cast<uint32_t>(nsec != policies.end() ? nsec->second.as_int() : 0));
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
  std::unordered_map<std::string, std::map<std::string, rclcpp::Parameter>> qos_overrides_{};
  double start_offset_s_{0.0};
  double bag_duration_s_{0.0};
  bool pub_progress_enabled_{false};
  bool valid_bag_storage_{false};
};
}  // namespace rslcpp::helper_nodes
