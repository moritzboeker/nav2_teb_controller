#pragma once

#include <angles/angles.h>

#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ostream>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav2_teb_controller/math_utils.hpp"

namespace nav2_teb_controller {

/**
 * @brief Modern SE(2) Pose: Eigen3 fixed-size, header-only, numerically stable.
 */
class PoseSE2 {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructors
  PoseSE2() = default;
  PoseSE2(const PoseSE2 &) = default;
  PoseSE2(PoseSE2 &&) = default;
  PoseSE2(double x, double y, double th) : _position{x, y}, _theta{th} {}
  PoseSE2(const Eigen::Vector2d &position, double th) : _position{position}, _theta{th} {}
  // From messages
  explicit PoseSE2(const geometry_msgs::msg::Pose2D &pose)
      : _position{pose.x, pose.y}, _theta{pose.theta} {}
  explicit PoseSE2(const geometry_msgs::msg::Pose &pose)
      : _position{pose.position.x, pose.position.y}, _theta{tf2::getYaw(pose.orientation)} {}
  explicit PoseSE2(const geometry_msgs::msg::PoseStamped &pose) : PoseSE2(pose.pose) {}

  // Destructor
  ~PoseSE2() = default;

  // Accessors
  Eigen::Vector2d &position() { return _position; }
  const Eigen::Vector2d &position() const { return _position; }
  double &x() { return _position.x(); }
  const double &x() const { return _position.x(); }
  double &y() { return _position.y(); }
  const double &y() const { return _position.y(); }
  double &theta() { return _theta; }
  const double &theta() const { return _theta; }

  // Conversions
  geometry_msgs::msg::Pose toPoseMsg() const {
    geometry_msgs::msg::Pose msg;
    msg.position.x = x();
    msg.position.y = y();
    tf2::Quaternion q;
    q.setRPY(0, 0, _theta);
    msg.orientation = tf2::toMsg(q);
    return msg;
  }

  geometry_msgs::msg::Pose2D toPose2D() const {
    geometry_msgs::msg::Pose2D msg;
    msg.x = x();
    msg.y = y();
    msg.theta = theta();
    return msg;
  }

  // Utilities
  Eigen::Vector2d orientationUnitVec() const {
    return Eigen::Vector2d(std::cos(_theta), std::sin(_theta));
  }

  void setZero() {
    _position.setZero();
    _theta = 0.0;
  }

  void scale(double factor) {
    _position *= factor;
    _theta = angles::normalize_angle(_theta * factor);
  }

  void plus(const double
                *pose_as_array)  // needed for g2o vertex_pose, expects array[3]: {dx, dy, dtheta}
  {
    _position.coeffRef(0) += pose_as_array[0];
    _position.coeffRef(1) += pose_as_array[1];
    _theta = angles::normalize_angle(_theta + pose_as_array[2]);
  }

  static PoseSE2 average(const PoseSE2 &p1, const PoseSE2 &p2) {
    return PoseSE2((p1._position + p2._position) / 2.0, average_angle(p1._theta, p2._theta));
  }

  void rotateGlobal(double angle, bool adjust_theta = true) {
    _position = Eigen::Rotation2Dd(angle) * _position;
    if (adjust_theta)
      _theta = angles::normalize_angle(_theta + angle);
  }

  // Operators: =, +=, +, -=, -, scalar*p, p*scalar, <<,
  // Assignment
  PoseSE2 &operator=(const PoseSE2 &rhs) = default;
  PoseSE2 &operator=(PoseSE2 &&rhs) = default;

  // Compound assignment
  PoseSE2 &operator+=(const PoseSE2 &rhs) {
    _position += rhs._position;
    _theta = angles::normalize_angle(_theta + rhs._theta);
    return *this;
  }

  PoseSE2 &operator-=(const PoseSE2 &rhs) {
    _position -= rhs._position;
    _theta = angles::normalize_angle(_theta - rhs._theta);
    return *this;
  }

  PoseSE2 &operator*=(double scalar) {
    _position *= scalar;
    _theta = angles::normalize_angle(_theta * scalar);
    return *this;
  }

  friend PoseSE2 operator+(PoseSE2 lhs, const PoseSE2 &rhs) { return lhs += rhs; }
  friend PoseSE2 operator-(PoseSE2 lhs, const PoseSE2 &rhs) { return lhs -= rhs; }
  friend PoseSE2 operator*(PoseSE2 pose, double scalar) { return pose *= scalar; }
  friend PoseSE2 operator*(double scalar, PoseSE2 pose) { return pose *= scalar; }

  // Stream
  friend std::ostream &operator<<(std::ostream &os, const PoseSE2 &pose) {
    return os << "PoseSE2(" << pose.x() << ", " << pose.y() << ", " << pose.theta() << ")";
  }

private:
  Eigen::Vector2d _position{0.0, 0.0};
  double _theta = 0.0;
};

inline std::vector<nav2_teb_controller::PoseSE2> posesFromPath(const nav_msgs::msg::Path &path,
                                                               size_t max_poses = 20) {
  std::vector<nav2_teb_controller::PoseSE2> poses;
  const size_t n = std::min(path.poses.size(), max_poses);
  poses.reserve(n);
  for (size_t i = 0; i < n; ++i)
    poses.emplace_back(path.poses[i]);
  return poses;
}

}  // namespace nav2_teb_controller
