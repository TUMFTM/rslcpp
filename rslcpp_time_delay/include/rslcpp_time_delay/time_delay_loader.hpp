// Copyright 2025 Simon Sagmeister
#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rslcpp_time_delay_backend/delay_backend.hpp>
#include <sstream>
#include <string>
namespace rslcpp::time_delay
{
// Function to parse the CSV file and configure the delay backend_

class TimeDelayLoader : public rclcpp::Node
{
public:
  explicit TimeDelayLoader(rclcpp::NodeOptions options) : Node("TimeDelayLoader", options)
  {
    std::string default_file_path =
      ament_index_cpp::get_package_share_directory("rslcpp_time_delay") +
      "/config/ExampleDelayConfig.csv";
    std::string file_path = this->declare_parameter("delay_config_file_path", default_file_path);

    std::cout << "Loading delay configuration from: " << file_path << std::endl;
    parse_delay_config(file_path);

    double global_skew = this->declare_parameter("global_measurement_skew", 1.0);
    backend_.set_global_measurement_skew(global_skew);
  }

private:
  rslcpp::time_delay::DelayBackend & backend_ = rslcpp::time_delay::DelayBackend::getInstance();

private:
  void parse_delay_config(const std::string & file_path)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      std::cerr << "Error: Unable to open file: " << file_path << std::endl;
      return;
    }

    std::string line;
    // Skip the header line
    std::getline(file, line);

    while (std::getline(file, line)) {
      std::istringstream line_stream(line);
      std::string topic_name, delay_type, fixed_delay_str, measured_skew_str;

      // Parse the CSV line
      if (
        !std::getline(line_stream, topic_name, ',') ||
        !std::getline(line_stream, delay_type, ',') ||
        !std::getline(line_stream, fixed_delay_str, ',') ||
        !std::getline(line_stream, measured_skew_str, ',')) {
        std::cerr << "Error: Malformed line in CSV: " << line << std::endl;
        continue;
      }

      // Trim whitespace from the parsed strings
      auto trim = [](std::string & str) {
        str.erase(0, str.find_first_not_of(" \t"));
        str.erase(str.find_last_not_of(" \t") + 1);
      };
      trim(topic_name);
      trim(delay_type);
      trim(fixed_delay_str);
      trim(measured_skew_str);

      // Process the delay type and values
      try {
        if (delay_type == "FIXED") {
          if (!fixed_delay_str.empty()) {
            double fixed_delay = std::stod(fixed_delay_str);  // Convert to double
            backend_.set_fixed_topic_delay(topic_name, fixed_delay * 1e6);
          } else {
            std::cerr << "Error: Missing fixed delay value for topic: " << topic_name << std::endl;
          }
        } else if (delay_type == "MEASURED") {
          if (!measured_skew_str.empty()) {
            double measured_skew = std::stod(measured_skew_str);  // Convert to double
            backend_.set_measured_topic_delay_skew(topic_name, measured_skew);
          } else {
            std::cerr << "Error: Missing measured skew value for topic: " << topic_name
                      << std::endl;
          }
        } else {
          std::cerr << "Error: Unknown delay type: " << delay_type << " for topic: " << topic_name
                    << std::endl;
        }
      } catch (const std::exception & e) {
        std::cerr << "Error: Failed to process line: " << line << " - " << e.what() << std::endl;
      }
    }

    file.close();
  }
};
}  // namespace rslcpp::time_delay
