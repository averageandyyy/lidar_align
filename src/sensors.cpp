#include "lidar_align/sensors.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace lidar_align {

OdomTformData::OdomTformData(Timestamp timestamp_us, Transform T_o0_ot)
    : T_o0_ot_(T_o0_ot), timestamp_us_(timestamp_us) {}

const Transform& OdomTformData::getTransform() const { return T_o0_ot_; }

const Timestamp& OdomTformData::getTimestamp() const { return timestamp_us_; }

void Odom::addTransformData(const Timestamp& timestamp_us, const Transform& T) {
  data_.emplace_back(timestamp_us, T);
}

Transform Odom::getOdomTransform(const Timestamp timestamp_us,
                                 const size_t start_idx,
                                 size_t* match_idx) const {
  if (data_.empty()) {
    throw std::runtime_error("Attempted to interpolate an empty odometry buffer");
  }
  if (data_.size() == 1) {
    if (match_idx != nullptr) {
      *match_idx = 0;
    }
    return data_.front().getTransform();
  }

  size_t idx = start_idx;
  while ((idx < (data_.size() - 1)) &&
         (timestamp_us > data_[idx].getTimestamp())) {
    ++idx;
  }
  if (idx > 0) {
    --idx;
  }

  if (match_idx != nullptr) {
    *match_idx = idx;
  }

  const double denominator = static_cast<double>(data_[idx + 1].getTimestamp() -
                                                 data_[idx].getTimestamp());
  const double t_diff_ratio =
      denominator == 0.0
          ? 0.0
          : static_cast<double>(timestamp_us - data_[idx].getTimestamp()) /
                denominator;

  const Transform::Vector6 diff_vector =
      (data_[idx].getTransform().inverse() * data_[idx + 1].getTransform()).log();
  const Transform out =
      data_[idx].getTransform() * Transform::exp(t_diff_ratio * diff_vector);

  return out;
}

Scan::Scan(const LoaderPointcloud& in, const Config& config)
    : timestamp_us_(static_cast<Timestamp>(in.header.stamp)),
      odom_transform_set_(false),
      motion_compensation_enabled_(config.motion_compensation) {
  std::default_random_engine generator(
      static_cast<unsigned int>(in.header.stamp & 0xffffffffu));
  std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

  for (const PointAllFields& point : in) {
    if ((static_cast<float>(point.intensity) > config.min_return_intensity) &&
        distribution(generator) < config.keep_points_ratio) {
      const float sq_dist = point.x * point.x + point.y * point.y + point.z * point.z;
      if (std::isfinite(sq_dist) &&
          (sq_dist > (config.min_point_distance * config.min_point_distance)) &&
          (sq_dist < (config.max_point_distance * config.max_point_distance))) {
        Point store_point;
        store_point.x = point.x;
        store_point.y = point.y;
        store_point.z = point.z;
        store_point.intensity = static_cast<float>(point.time_offset_us);

        if (config.estimate_point_times) {
          const double timing_factor = 19098593.171 / config.lidar_rpm;
          const double angle = std::atan2(point.x, point.y);

          if (std::abs(angle) > 3.0) {
            continue;
          }
          store_point.intensity = static_cast<float>(angle * timing_factor);
          if (!config.clockwise_lidar) {
            store_point.intensity *= -1.0f;
          }
        }
        raw_points_.push_back(store_point);
      }
    }
  }
  raw_points_.header = in.header;
}

Scan::Config Scan::getConfig(const std::shared_ptr<rclcpp::Node>& node) {
  Scan::Config config;
  config.min_point_distance =
      node->declare_parameter<double>("min_point_distance", config.min_point_distance);
  config.max_point_distance =
      node->declare_parameter<double>("max_point_distance", config.max_point_distance);
  config.keep_points_ratio =
      node->declare_parameter<double>("keep_points_ratio", config.keep_points_ratio);
  config.min_return_intensity = node->declare_parameter<double>(
      "min_return_intensity", config.min_return_intensity);

  config.estimate_point_times = node->declare_parameter<bool>(
      "estimate_point_times", config.estimate_point_times);
  config.clockwise_lidar =
      node->declare_parameter<bool>("clockwise_lidar", config.clockwise_lidar);
  config.motion_compensation = node->declare_parameter<bool>(
      "motion_compensation", config.motion_compensation);
  config.lidar_rpm = node->declare_parameter<double>("lidar_rpm", config.lidar_rpm);

  return config;
}

void Scan::setOdomTransform(const Odom& odom, const double time_offset,
                            const size_t start_idx, size_t* match_idx) {
  T_o0_ot_.clear();

  for (const Point& point : raw_points_) {
    const Timestamp point_offset_us =
        motion_compensation_enabled_ ? static_cast<Timestamp>(point.intensity) : 0ll;
    const Timestamp point_ts_us =
        timestamp_us_ + static_cast<Timestamp>(1000000.0 * time_offset) +
        point_offset_us;

    T_o0_ot_.push_back(odom.getOdomTransform(point_ts_us, start_idx, match_idx));
  }
  odom_transform_set_ = true;
}

const Transform& Scan::getOdomTransform() const {
  if (!odom_transform_set_) {
    throw std::runtime_error(
        "Attempted to get odom transform before it was set");
  }
  return T_o0_ot_.front();
}

void Scan::getTimeAlignedPointcloud(const Transform& T_o_l,
                                    Pointcloud* pointcloud) const {
  if (!odom_transform_set_) {
    throw std::runtime_error(
        "Attempted to get a time aligned pointcloud before odom transforms were set");
  }

  for (size_t i = 0; i < raw_points_.size(); ++i) {
    const Transform T_o_lt = T_o0_ot_[i] * T_o_l;
    const Transform::Translation transformed =
        T_o_lt.rotation() * Transform::Translation(raw_points_[i].x,
                                                   raw_points_[i].y,
                                                   raw_points_[i].z) +
        T_o_lt.translation();

    Point out;
    out.x = transformed.x();
    out.y = transformed.y();
    out.z = transformed.z();
    out.intensity = raw_points_[i].intensity;
    pointcloud->push_back(out);
  }
}

const Pointcloud& Scan::getRawPointcloud() const { return raw_points_; }

Lidar::Lidar(const LidarId& lidar_id) : lidar_id_(lidar_id) {}

size_t Lidar::getNumberOfScans() const { return scans_.size(); }

size_t Lidar::getTotalPoints() const {
  size_t num_points = 0;
  for (const Scan& scan : scans_) {
    num_points += scan.getRawPointcloud().size();
  }
  return num_points;
}

const LidarId& Lidar::getId() const { return lidar_id_; }

void Lidar::addPointcloud(const LoaderPointcloud& pointcloud,
                          const Scan::Config& config) {
  scans_.emplace_back(pointcloud, config);
}

void Lidar::getCombinedPointcloud(Pointcloud* pointcloud) const {
  for (const Scan& scan : scans_) {
    scan.getTimeAlignedPointcloud(getOdomLidarTransform(), pointcloud);
  }
}

void Lidar::saveCombinedPointcloud(const std::string& file_path) const {
  Pointcloud combined;
  getCombinedPointcloud(&combined);
  pcl::PLYWriter writer;
  writer.write(file_path, combined, true);
}

void Lidar::setOdomOdomTransforms(const Odom& odom, const double time_offset) {
  size_t idx = 0;
  for (Scan& scan : scans_) {
    scan.setOdomTransform(odom, time_offset, idx, &idx);
  }
}

void Lidar::setOdomLidarTransform(const Transform& T_o_l) { T_o_l_ = T_o_l; }

const Transform& Lidar::getOdomLidarTransform() const { return T_o_l_; }

}  // namespace lidar_align
