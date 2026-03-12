#include "lidar_align/aligner.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace lidar_align {

Aligner::Aligner(const Config& config) : config_(config) {}

Aligner::Config Aligner::getConfig(const std::shared_ptr<rclcpp::Node>& node) {
  Aligner::Config config;
  config.local = node->declare_parameter<bool>("local", config.local);
  config.inital_guess =
      node->declare_parameter<std::vector<double>>("inital_guess",
                                                   config.inital_guess);
  config.max_time_offset =
      node->declare_parameter<double>("max_time_offset", config.max_time_offset);
  config.angular_range =
      node->declare_parameter<double>("angular_range", config.angular_range);
  config.translation_range = node->declare_parameter<double>(
      "translation_range", config.translation_range);
  config.max_evals = node->declare_parameter<double>("max_evals", config.max_evals);
  config.xtol = node->declare_parameter<double>("xtol", config.xtol);
  config.knn_batch_size =
      node->declare_parameter<int>("knn_batch_size", config.knn_batch_size);
  config.knn_k = node->declare_parameter<int>("knn_k", config.knn_k);
  config.global_knn_max_dist = node->declare_parameter<double>(
      "global_knn_max_dist", config.global_knn_max_dist);
  config.local_knn_max_dist = node->declare_parameter<double>(
      "local_knn_max_dist", config.local_knn_max_dist);
  config.time_cal = node->declare_parameter<bool>("time_cal", config.time_cal);
  config.output_pointcloud_path = node->declare_parameter<std::string>(
      "output_pointcloud_path", config.output_pointcloud_path);
  config.output_calibration_path = node->declare_parameter<std::string>(
      "output_calibration_path", config.output_calibration_path);

  return config;
}

float Aligner::kNNError(const pcl::KdTreeFLANN<Point>& kdtree,
                        const Pointcloud& pointcloud, const size_t k,
                        const float max_dist, const size_t start_idx,
                        const size_t end_idx) {
  std::vector<int> kdtree_idx(k);
  std::vector<float> kdtree_dist(k);

  float error = 0.0f;
  for (size_t idx = start_idx; idx < std::min(pointcloud.size(), end_idx);
       ++idx) {
    if (kdtree.nearestKSearch(pointcloud[idx], static_cast<int>(k), kdtree_idx,
                              kdtree_dist) <= 0) {
      continue;
    }
    for (const float distance : kdtree_dist) {
      error += std::min(distance, max_dist);
    }
  }
  return error;
}

float Aligner::lidarOdomKNNError(const Pointcloud& base_pointcloud,
                                 const Pointcloud& combined_pointcloud) const {
  if (!rclcpp::ok()) {
    throw std::runtime_error("ROS 2 node died, exiting");
  }

  Pointcloud::ConstPtr combined_pointcloud_ptr(
      &combined_pointcloud, [](const Pointcloud*) {});

  pcl::KdTreeFLANN<Point> kdtree;
  kdtree.setInputCloud(combined_pointcloud_ptr);

  const float max_dist =
      config_.local ? config_.local_knn_max_dist : config_.global_knn_max_dist;

  size_t k = static_cast<size_t>(config_.knn_k);
  if (&base_pointcloud == &combined_pointcloud) {
    ++k;
  }

  std::vector<std::future<float>> errors;
  for (size_t start_idx = 0; start_idx < base_pointcloud.size();
       start_idx += static_cast<size_t>(config_.knn_batch_size)) {
    const size_t end_idx =
        start_idx + std::min(base_pointcloud.size() - start_idx,
                             static_cast<size_t>(config_.knn_batch_size));
    errors.emplace_back(std::async(std::launch::async, Aligner::kNNError,
                                   kdtree, base_pointcloud, k, max_dist,
                                   start_idx, end_idx));
  }

  float total_error = 0.0f;
  for (std::future<float>& error : errors) {
    total_error += error.get();
  }

  return total_error;
}

float Aligner::lidarOdomKNNError(const Lidar& lidar) const {
  Pointcloud pointcloud;
  lidar.getCombinedPointcloud(&pointcloud);
  return lidarOdomKNNError(pointcloud, pointcloud);
}

double Aligner::LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& /*grad*/, void* f_data) {
  auto* data = static_cast<OptData*>(f_data);

  if (x.size() > 6) {
    data->lidar->setOdomOdomTransforms(*(data->odom), x[6]);
  }

  Transform::Vector6 vector;
  vector.setZero();

  const size_t offset = x.size() == 3 ? 3 : 0;
  for (size_t i = offset; i < 6; ++i) {
    vector[i] = static_cast<float>(x[i - offset]);
  }

  data->lidar->setOdomLidarTransform(Transform::exp(vector));

  const double error = data->aligner->lidarOdomKNNError(*(data->lidar));

  static int iteration = 0;
  std::cout << std::fixed << std::setprecision(2);
  if (x.size() > 3) {
    std::cout << " \e[1mx:\e[0m " << std::setw(6) << vector[0];
    std::cout << " \e[1my:\e[0m " << std::setw(6) << vector[1];
    std::cout << " \e[1mz:\e[0m " << std::setw(6) << vector[2];
  }
  std::cout << " \e[1mrx:\e[0m " << std::setw(6) << vector[3];
  std::cout << " \e[1mry:\e[0m " << std::setw(6) << vector[4];
  std::cout << " \e[1mrz:\e[0m " << std::setw(6) << vector[5];
  if (x.size() > 6) {
    std::cout << " \e[1mtime:\e[0m " << std::setw(6) << x[6];
  }
  std::cout << " \e[1mError:\e[0m " << std::setw(10) << error;
  std::cout << " \e[1mIteration:\e[0m " << iteration++ << '\r' << std::flush;

  return error;
}

void Aligner::optimize(const std::vector<double>& lb,
                       const std::vector<double>& ub, OptData* opt_data,
                       std::vector<double>* x) {
  nlopt::opt opt(config_.local ? nlopt::LN_BOBYQA : nlopt::GN_DIRECT_L,
                 x->size());

  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_maxeval(static_cast<int>(config_.max_evals));
  opt.set_xtol_abs(config_.xtol);
  opt.set_min_objective(LidarOdomMinimizer, opt_data);

  double minf = 0.0;
  nlopt::result result = opt.optimize(*x, minf);
  (void)result;

  std::vector<double> grad;
  LidarOdomMinimizer(*x, grad, opt_data);
}

std::string Aligner::generateCalibrationString(const Transform& T,
                                               const double time_offset) const {
  const Transform::Vector6 T_log = T.log();
  std::stringstream ss;

  ss << "Active Transformation Vector (x,y,z,rx,ry,rz) from the Pose Sensor "
        "Frame to the Lidar Frame:"
     << std::endl
     << "[";
  ss << T_log[0] << ", ";
  ss << T_log[1] << ", ";
  ss << T_log[2] << ", ";
  ss << T_log[3] << ", ";
  ss << T_log[4] << ", ";
  ss << T_log[5] << "]" << std::endl << std::endl;

  ss << "Active Transformation Matrix from the Pose Sensor Frame to the "
        "Lidar Frame:"
     << std::endl;
  ss << T.matrix() << std::endl << std::endl;

  ss << "Active Translation Vector (x,y,z) from the Pose Sensor Frame to "
        "the Lidar Frame:"
     << std::endl
     << "[";
  ss << T.translation().x() << ", ";
  ss << T.translation().y() << ", ";
  ss << T.translation().z() << "]" << std::endl << std::endl;

  ss << "Active Hamiltonian Quaternion (w,x,y,z) from the Pose Sensor "
        "Frame to the Lidar Frame:"
     << std::endl
     << "[";
  ss << T.rotation().w() << ", ";
  ss << T.rotation().x() << ", ";
  ss << T.rotation().y() << ", ";
  ss << T.rotation().z() << "]" << std::endl << std::endl;

  if (config_.time_cal) {
    ss << "Time offset that must be added to lidar timestamps in seconds:"
       << std::endl
       << time_offset << std::endl
       << std::endl;
  }

  ss << "ROS 2 static TF publisher:" << std::endl;
  ss << "ros2 run tf2_ros static_transform_publisher ";
  ss << "--x " << T.translation().x() << ' ';
  ss << "--y " << T.translation().y() << ' ';
  ss << "--z " << T.translation().z() << ' ';
  ss << "--qx " << T.rotation().x() << ' ';
  ss << "--qy " << T.rotation().y() << ' ';
  ss << "--qz " << T.rotation().z() << ' ';
  ss << "--qw " << T.rotation().w() << ' ';
  ss << "--frame-id POSE_FRAME --child-frame-id LIDAR_FRAME" << std::endl;

  return ss.str();
}

void Aligner::lidarOdomTransform(Lidar* lidar, Odom* odom) {
  OptData opt_data;
  opt_data.lidar = lidar;
  opt_data.odom = odom;
  opt_data.aligner = this;

  size_t num_params = 6;
  if (config_.time_cal) {
    ++num_params;
  }

  std::vector<double> x(num_params, 0.0);

  if (!config_.local) {
    RCLCPP_INFO(rclcpp::get_logger("lidar_align"),
                "Performing global optimization...");

    constexpr double kPi = 3.14159265358979323846;
    const std::vector<double> lb = {-kPi, -kPi, -kPi};
    const std::vector<double> ub = {kPi, kPi, kPi};

    std::vector<double> global_x(3, 0.0);
    optimize(lb, ub, &opt_data, &global_x);
    config_.local = true;

    x[3] = global_x[0];
    x[4] = global_x[1];
    x[5] = global_x[2];
  } else {
    x = config_.inital_guess;
    if (x.size() != num_params) {
      x.resize(num_params, 0.0);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("lidar_align"),
              "Performing local optimization...");

  std::vector<double> lb = {
      -config_.translation_range, -config_.translation_range,
      -config_.translation_range, -config_.angular_range,
      -config_.angular_range,     -config_.angular_range};
  std::vector<double> ub = {
      config_.translation_range, config_.translation_range,
      config_.translation_range, config_.angular_range,
      config_.angular_range,     config_.angular_range};
  for (size_t i = 0; i < 6; ++i) {
    lb[i] += x[i];
    ub[i] += x[i];
  }
  if (config_.time_cal) {
    ub.push_back(config_.max_time_offset);
    lb.push_back(-config_.max_time_offset);
  }

  optimize(lb, ub, &opt_data, &x);

  if (!config_.output_pointcloud_path.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("lidar_align"),
                "Saving aligned pointcloud...");
    lidar->saveCombinedPointcloud(config_.output_pointcloud_path);
  }

  const double time_offset = config_.time_cal ? x.back() : 0.0;
  const std::string output_calibration =
      generateCalibrationString(lidar->getOdomLidarTransform(), time_offset);
  if (!config_.output_calibration_path.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("lidar_align"),
                "Saving calibration file...");

    std::ofstream file(config_.output_calibration_path, std::ofstream::out);
    file << output_calibration;
  }
  RCLCPP_INFO(rclcpp::get_logger("lidar_align"), "Final calibration:");
  std::cout << output_calibration;
}

}  // namespace lidar_align
