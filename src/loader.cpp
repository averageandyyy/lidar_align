#include "lidar_align/loader.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lidar_align/transform.h"

namespace lidar_align {
namespace {

rclcpp::Logger getLogger() { return rclcpp::get_logger("lidar_align.loader"); }

Timestamp stampToMicroseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<Timestamp>(stamp.sec) * 1000000ll +
         static_cast<Timestamp>(stamp.nanosec / 1000u);
}

template <typename MessageT>
bool deserializeBagMessage(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message,
    MessageT* message) {
  try {
    rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
    rclcpp::Serialization<MessageT> serialization;
    serialization.deserialize_message(&serialized_message, message);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(getLogger(), "Failed to deserialize topic '%s': %s",
                 bag_message->topic_name.c_str(), e.what());
    return false;
  }
}

}  // namespace

Loader::Loader(const Config& config) : config_(config) {}

Loader::Config Loader::getConfig(const std::shared_ptr<rclcpp::Node>& node) {
  Loader::Config config;
  config.use_n_scans = node->declare_parameter<int>("use_n_scans", config.use_n_scans);
  return config;
}

void Loader::parsePointcloudMsg(const sensor_msgs::msg::PointCloud2& msg,
                                LoaderPointcloud* pointcloud) const {
  bool has_timing = false;
  bool has_intensity = false;
  for (const sensor_msgs::msg::PointField& field : msg.fields) {
    if (field.name == "time_offset_us") {
      has_timing = true;
    } else if (field.name == "intensity") {
      has_intensity = true;
    }
  }

  if (has_timing) {
    pcl::fromROSMsg(msg, *pointcloud);
  } else if (has_intensity) {
    Pointcloud raw_pointcloud;
    pcl::fromROSMsg(msg, raw_pointcloud);

    for (const Point& raw_point : raw_pointcloud) {
      PointAllFields point;
      point.x = raw_point.x;
      point.y = raw_point.y;
      point.z = raw_point.z;
      point.intensity = static_cast<uint16_t>(std::max(0.0f, raw_point.intensity));

      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(raw_point.intensity)) {
        continue;
      }

      pointcloud->push_back(point);
    }
    pointcloud->header = raw_pointcloud.header;
  } else {
    pcl::PointCloud<pcl::PointXYZ> raw_pointcloud;
    pcl::fromROSMsg(msg, raw_pointcloud);

    for (const pcl::PointXYZ& raw_point : raw_pointcloud) {
      PointAllFields point;
      point.x = raw_point.x;
      point.y = raw_point.y;
      point.z = raw_point.z;

      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }

      pointcloud->push_back(point);
    }
    pointcloud->header = raw_pointcloud.header;
  }

  pointcloud->header.stamp = static_cast<std::uint64_t>(stampToMicroseconds(msg.header.stamp));
  pointcloud->header.frame_id = msg.header.frame_id;
}

bool Loader::loadPointcloudFromROSBag(const std::string& bag_path,
                                      const Scan::Config& scan_config,
                                      Lidar* lidar) const {
  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag_path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(getLogger(), "Opening rosbag2 input failed: %s", e.what());
    return false;
  }

  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    topic_types[topic.name] = topic.type;
  }

  size_t scan_num = 0;
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    const auto topic_it = topic_types.find(bag_message->topic_name);
    if (topic_it == topic_types.end() ||
        topic_it->second != "sensor_msgs/msg/PointCloud2") {
      continue;
    }

    std::cout << " Loading scan: \e[1m" << scan_num++
              << "\e[0m from rosbag2" << '\r' << std::flush;

    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    if (!deserializeBagMessage(bag_message, &pointcloud_msg)) {
      return false;
    }

    LoaderPointcloud pointcloud;
    parsePointcloudMsg(pointcloud_msg, &pointcloud);
    lidar->addPointcloud(pointcloud, scan_config);

    if (static_cast<int>(lidar->getNumberOfScans()) >= config_.use_n_scans) {
      break;
    }
  }
  std::cout << std::endl;

  if (lidar->getTotalPoints() == 0) {
    RCLCPP_ERROR(getLogger(),
                 "No points were loaded. Verify that the bag contains populated "
                 "sensor_msgs/msg/PointCloud2 messages.");
    return false;
  }

  return true;
}

bool Loader::loadTformFromROSBag(const std::string& bag_path, Odom* odom) const {
  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag_path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(getLogger(), "Opening rosbag2 input failed: %s", e.what());
    return false;
  }

  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    topic_types[topic.name] = topic.type;
  }

  size_t tform_num = 0;
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    const auto topic_it = topic_types.find(bag_message->topic_name);
    if (topic_it == topic_types.end() ||
        topic_it->second != "geometry_msgs/msg/TransformStamped") {
      continue;
    }

    std::cout << " Loading transform: \e[1m" << tform_num++
              << "\e[0m from rosbag2" << '\r' << std::flush;

    geometry_msgs::msg::TransformStamped transform_msg;
    if (!deserializeBagMessage(bag_message, &transform_msg)) {
      return false;
    }

    const Timestamp stamp = stampToMicroseconds(transform_msg.header.stamp);

    const Transform T(
        Transform::Translation(
            static_cast<float>(transform_msg.transform.translation.x),
            static_cast<float>(transform_msg.transform.translation.y),
            static_cast<float>(transform_msg.transform.translation.z)),
        Transform::Rotation(static_cast<float>(transform_msg.transform.rotation.w),
                            static_cast<float>(transform_msg.transform.rotation.x),
                            static_cast<float>(transform_msg.transform.rotation.y),
                            static_cast<float>(transform_msg.transform.rotation.z)));
    odom->addTransformData(stamp, T);
  }
  std::cout << std::endl;

  if (odom->size() < 2) {
    RCLCPP_ERROR(getLogger(),
                 "Fewer than two odometry transforms were found in the bag.");
    return false;
  }

  return true;
}

bool Loader::loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom) const {
  std::ifstream file(csv_path, std::ifstream::in);
  if (!file.is_open()) {
    RCLCPP_ERROR(getLogger(), "Could not open CSV file: %s", csv_path.c_str());
    return false;
  }

  size_t tform_num = 0;
  while (file.peek() != EOF) {
    std::cout << " Loading transform: \e[1m" << tform_num++
              << "\e[0m from csv file" << '\r' << std::flush;

    Timestamp stamp = 0;
    Transform T;

    if (getNextCSVTransform(file, &stamp, &T)) {
      odom->addTransformData(stamp, T);
    }
  }
  std::cout << std::endl;

  if (odom->size() < 2) {
    RCLCPP_ERROR(getLogger(),
                 "Fewer than two odometry transforms were loaded from the CSV file.");
    return false;
  }

  return true;
}

bool Loader::getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                 Transform* T) {
  std::string line;
  std::getline(str, line);

  if (line.empty() || line[0] == '#') {
    return false;
  }

  std::stringstream line_stream(line);
  std::string cell;

  std::vector<std::string> data;
  while (std::getline(line_stream, cell, ',')) {
    data.push_back(cell);
  }

  if (data.size() < 9) {
    return false;
  }

  constexpr size_t TIME = 0;
  constexpr size_t X = 2;
  constexpr size_t Y = 3;
  constexpr size_t Z = 4;
  constexpr size_t RW = 5;
  constexpr size_t RX = 6;
  constexpr size_t RY = 7;
  constexpr size_t RZ = 8;

  *stamp = std::stoll(data[TIME]) / 1000ll;
  *T = Transform(
      Transform::Translation(static_cast<float>(std::stod(data[X])),
                             static_cast<float>(std::stod(data[Y])),
                             static_cast<float>(std::stod(data[Z]))),
      Transform::Rotation(static_cast<float>(std::stod(data[RW])),
                          static_cast<float>(std::stod(data[RX])),
                          static_cast<float>(std::stod(data[RY])),
                          static_cast<float>(std::stod(data[RZ]))));

  return true;
}

}  // namespace lidar_align
