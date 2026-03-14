#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "lidar_align/aligner.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"

using namespace lidar_align;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("lidar_align");

  Loader loader(Loader::getConfig(node));

  Lidar lidar;
  Odom odom;

  const std::string input_bag_path =
      node->declare_parameter<std::string>("input_bag_path", "");
  RCLCPP_INFO(node->get_logger(), "Loading pointcloud data...: %s",
              input_bag_path.c_str());
  if (input_bag_path.empty()) {
    RCLCPP_FATAL(node->get_logger(),
                 "Could not find input_bag_path parameter, exiting.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  if (!loader.loadPointcloudFromROSBag(input_bag_path, Scan::getConfig(node),
                                       &lidar)) {
    RCLCPP_FATAL(node->get_logger(),
                 "Error loading pointclouds from ROS 2 bag.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const bool transforms_from_csv =
      node->declare_parameter<bool>("transforms_from_csv", false);
  RCLCPP_INFO(node->get_logger(), "Loading transformation data...");
  if (transforms_from_csv) {
    const std::string input_csv_path =
        node->declare_parameter<std::string>("input_csv_path", "");
    if (input_csv_path.empty()) {
      RCLCPP_FATAL(node->get_logger(),
                   "Could not find input_csv_path parameter, exiting.");
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
    if (!loader.loadTformFromMaplabCSV(input_csv_path, &odom)) {
      RCLCPP_FATAL(node->get_logger(), "Error loading transforms from CSV.");
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
  } else if (!loader.loadTformFromROSBag(input_bag_path, &odom)) {
    RCLCPP_FATAL(node->get_logger(),
                 "Error loading transforms from ROS 2 bag.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  if (lidar.getNumberOfScans() == 0) {
    RCLCPP_FATAL(node->get_logger(), "No lidar data loaded, exiting.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(node->get_logger(), "Interpolating transformation data...");
  lidar.setOdomOdomTransforms(odom);

  Aligner aligner(Aligner::getConfig(node));
  aligner.lidarOdomTransform(&lidar, &odom);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
