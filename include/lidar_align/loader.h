#ifndef LIDAR_ALIGN_LOADER_H_
#define LIDAR_ALIGN_LOADER_H_

#include <istream>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lidar_align/sensors.h"

namespace lidar_align {

class Loader {
 public:
  struct Config {
    int use_n_scans = std::numeric_limits<int>::max();
  };

  explicit Loader(const Config& config);

  void parsePointcloudMsg(const sensor_msgs::msg::PointCloud2& msg,
                          LoaderPointcloud* pointcloud) const;

  bool loadPointcloudFromROSBag(const std::string& bag_path,
                                const Scan::Config& scan_config,
                                Lidar* lidar) const;

  bool loadTformFromROSBag(const std::string& bag_path, Odom* odom) const;

  bool loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom) const;

  static Config getConfig(const std::shared_ptr<rclcpp::Node>& node);

 private:
  static bool getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                  Transform* T);

  Config config_;
};

}  // namespace lidar_align

POINT_CLOUD_REGISTER_POINT_STRUCT(
    lidar_align::PointAllFields,
    (float, x, x)(float, y, y)(float, z, z)(int32_t, time_offset_us,
                                            time_offset_us)(
        uint16_t, reflectivity, reflectivity)(uint16_t, intensity,
                                              intensity)(uint8_t, ring, ring))

#endif  // LIDAR_ALIGN_LOADER_H_
