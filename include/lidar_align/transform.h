#ifndef LIDAR_ALIGN_TRANSFORM_H_
#define LIDAR_ALIGN_TRANSFORM_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <ostream>

namespace lidar_align {

class Transform {
 public:
  struct Vector6 {
    std::array<float, 6> data{};

    void setZero() { data.fill(0.0f); }

    float& operator[](const std::size_t idx) { return data[idx]; }
    const float& operator[](const std::size_t idx) const { return data[idx]; }
  };

  class Translation {
   public:
    Translation() = default;
    Translation(const float x, const float y, const float z)
        : x_(x), y_(y), z_(z) {}

    float x() const { return x_; }
    float y() const { return y_; }
    float z() const { return z_; }

    float norm() const { return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_); }

    Translation operator+(const Translation& rhs) const {
      return Translation(x_ + rhs.x_, y_ + rhs.y_, z_ + rhs.z_);
    }

    Translation operator-(const Translation& rhs) const {
      return Translation(x_ - rhs.x_, y_ - rhs.y_, z_ - rhs.z_);
    }

    Translation operator-() const { return Translation(-x_, -y_, -z_); }

    Translation operator*(const float scalar) const {
      return Translation(x_ * scalar, y_ * scalar, z_ * scalar);
    }

    Translation operator/(const float scalar) const {
      return Translation(x_ / scalar, y_ / scalar, z_ / scalar);
    }

   private:
    float x_ = 0.0f;
    float y_ = 0.0f;
    float z_ = 0.0f;
  };

  class Rotation {
   public:
    Rotation() = default;
    Rotation(const float w, const float x, const float y, const float z)
        : w_(w), x_(x), y_(y), z_(z) {
      normalize();
    }

    static Rotation Identity() { return Rotation(1.0f, 0.0f, 0.0f, 0.0f); }

    float w() const { return w_; }
    float x() const { return x_; }
    float y() const { return y_; }
    float z() const { return z_; }

    Rotation inverse() const { return Rotation(w_, -x_, -y_, -z_); }

    Rotation operator*(const Rotation& rhs) const {
      return Rotation(w_ * rhs.w_ - x_ * rhs.x_ - y_ * rhs.y_ - z_ * rhs.z_,
                      w_ * rhs.x_ + x_ * rhs.w_ + y_ * rhs.z_ - z_ * rhs.y_,
                      w_ * rhs.y_ - x_ * rhs.z_ + y_ * rhs.w_ + z_ * rhs.x_,
                      w_ * rhs.z_ + x_ * rhs.y_ - y_ * rhs.x_ + z_ * rhs.w_);
    }

    Translation operator*(const Translation& rhs) const {
      const Translation qv(x_, y_, z_);
      const Translation uv = cross(qv, rhs);
      const Translation uuv = cross(qv, uv);
      return rhs + uv * (2.0f * w_) + uuv * 2.0f;
    }

    static Rotation fromRotationVector(const Translation& rotation_vector) {
      constexpr float kEpsilon = 1e-8f;
      const float angle = rotation_vector.norm();
      if (angle < kEpsilon) {
        return Identity();
      }

      const Translation axis = rotation_vector / angle;
      const float half_angle = angle * 0.5f;
      const float sin_half = std::sin(half_angle);
      return Rotation(std::cos(half_angle), axis.x() * sin_half,
                      axis.y() * sin_half, axis.z() * sin_half);
    }

    Translation toRotationVector() const {
      constexpr float kEpsilon = 1e-8f;
      Rotation normalized = *this;
      normalized.normalize();

      const float sin_half =
          std::sqrt(normalized.x_ * normalized.x_ + normalized.y_ * normalized.y_ +
                    normalized.z_ * normalized.z_);
      if (sin_half < kEpsilon) {
        return Translation();
      }

      const float clamped_w = std::clamp(normalized.w_, -1.0f, 1.0f);
      const float angle = 2.0f * std::atan2(sin_half, clamped_w);
      const Translation axis(normalized.x_ / sin_half, normalized.y_ / sin_half,
                             normalized.z_ / sin_half);
      return axis * angle;
    }

    std::array<std::array<float, 3>, 3> matrix() const {
      Rotation normalized = *this;
      normalized.normalize();

      const float xx = normalized.x_ * normalized.x_;
      const float yy = normalized.y_ * normalized.y_;
      const float zz = normalized.z_ * normalized.z_;
      const float xy = normalized.x_ * normalized.y_;
      const float xz = normalized.x_ * normalized.z_;
      const float yz = normalized.y_ * normalized.z_;
      const float wx = normalized.w_ * normalized.x_;
      const float wy = normalized.w_ * normalized.y_;
      const float wz = normalized.w_ * normalized.z_;

      return {{{1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz),
                2.0f * (xz + wy)},
               {2.0f * (xy + wz), 1.0f - 2.0f * (xx + zz),
                2.0f * (yz - wx)},
               {2.0f * (xz - wy), 2.0f * (yz + wx),
                1.0f - 2.0f * (xx + yy)}}};
    }

   private:
    static Translation cross(const Translation& lhs, const Translation& rhs) {
      return Translation(lhs.y() * rhs.z() - lhs.z() * rhs.y(),
                         lhs.z() * rhs.x() - lhs.x() * rhs.z(),
                         lhs.x() * rhs.y() - lhs.y() * rhs.x());
    }

    void normalize() {
      const float magnitude =
          std::sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
      if (magnitude <= 0.0f) {
        w_ = 1.0f;
        x_ = 0.0f;
        y_ = 0.0f;
        z_ = 0.0f;
        return;
      }
      w_ /= magnitude;
      x_ /= magnitude;
      y_ /= magnitude;
      z_ /= magnitude;
    }

    float w_ = 1.0f;
    float x_ = 0.0f;
    float y_ = 0.0f;
    float z_ = 0.0f;
  };

  struct Matrix {
    float data[4][4]{};

    float* operator[](const std::size_t row) { return data[row]; }
    const float* operator[](const std::size_t row) const { return data[row]; }
  };

  Transform() {
    rotation_ = Rotation::Identity();
    translation_ = Translation();
  }

  Transform(const Translation& translation, const Rotation& rotation)
      : translation_(translation), rotation_(rotation) {}

  const Rotation& rotation() const { return rotation_; }

  const Translation& translation() const { return translation_; }

  Matrix matrix() const {
    Matrix out{};
    const auto rotation_matrix = rotation_.matrix();

    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t col = 0; col < 3; ++col) {
        out[row][col] = rotation_matrix[row][col];
      }
    }
    out[0][3] = translation_.x();
    out[1][3] = translation_.y();
    out[2][3] = translation_.z();
    out[3][3] = 1.0f;
    return out;
  }

  Transform inverse() const {
    const Rotation rotation_inverted = rotation_.inverse();
    return Transform(rotation_inverted * (-translation_), rotation_inverted);
  }

  Transform operator*(const Transform& rhs) const {
    return Transform(translation_ + rotation_ * rhs.translation(),
                     rotation_ * rhs.rotation());
  }

  static Transform exp(const Vector6& vector) {
    return Transform(Translation(vector[0], vector[1], vector[2]),
                     Rotation::fromRotationVector(
                         Translation(vector[3], vector[4], vector[5])));
  }

  Vector6 log() const {
    Vector6 out;
    out.setZero();
    out[0] = translation_.x();
    out[1] = translation_.y();
    out[2] = translation_.z();

    const Translation rotation_vector = rotation_.toRotationVector();
    out[3] = rotation_vector.x();
    out[4] = rotation_vector.y();
    out[5] = rotation_vector.z();
    return out;
  }

 private:
  Rotation rotation_;
  Translation translation_;
};

inline Transform::Vector6 operator*(const double scalar,
                                    const Transform::Vector6& vector) {
  Transform::Vector6 out;
  for (std::size_t i = 0; i < out.data.size(); ++i) {
    out[i] = static_cast<float>(scalar * vector[i]);
  }
  return out;
}

inline std::ostream& operator<<(std::ostream& os, const Transform::Matrix& matrix) {
  os << std::fixed << std::setprecision(6);
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      os << std::setw(11) << matrix[row][col];
      if (col + 1 < 4) {
        os << ' ';
      }
    }
    if (row + 1 < 4) {
      os << '\n';
    }
  }
  return os;
}

}  // namespace lidar_align

#endif  // LIDAR_ALIGN_TRANSFORM_H_
