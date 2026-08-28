// Copyright (c) 2025 Masazumi Imai
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "traj_gen/common.hpp"

#include <cmath>
#include <stdexcept>

namespace traj_gen
{

std::vector<VectorStateConstraint> createBoundaryConditions(
  const VectorStateConstraint & start_constraint, const VectorStateConstraint & end_constraint)
{
  std::vector<VectorStateConstraint> constraints;
  constraints.reserve(2);
  constraints.push_back(start_constraint);
  constraints.push_back(end_constraint);
  return constraints;
}

void addConstraint(
  std::vector<VectorStateConstraint> & constraints, const VectorStateConstraint & new_constraint)
{
  constraints.push_back(new_constraint);
}

void addConstraint(
  std::vector<VectorStateConstraint> & constraints, double time,
  const std::optional<Eigen::VectorXd> & position, const std::optional<Eigen::VectorXd> & velocity,
  const std::optional<Eigen::VectorXd> & acceleration)
{
  VectorStateConstraint new_constraint;
  new_constraint.time = time;
  new_constraint.position = position;
  new_constraint.velocity = velocity;
  new_constraint.acceleration = acceleration;
  constraints.push_back(new_constraint);
}

std::vector<AngularStateConstraint> createBoundaryConditions(
  const AngularStateConstraint & start_constraint, const AngularStateConstraint & end_constraint)
{
  std::vector<AngularStateConstraint> constraints;
  constraints.reserve(2);
  constraints.push_back(start_constraint);
  constraints.push_back(end_constraint);
  return constraints;
}

void addConstraint(
  std::vector<AngularStateConstraint> & constraints, const AngularStateConstraint & new_constraint)
{
  constraints.push_back(new_constraint);
}

void addConstraint(
  std::vector<AngularStateConstraint> & constraints, double time,
  const std::optional<Eigen::Quaterniond> & orientation,
  const std::optional<Eigen::Vector3d> & angular_velocity,
  const std::optional<Eigen::Vector3d> & angular_acceleration)
{
  AngularStateConstraint new_constraint;
  new_constraint.time = time;
  new_constraint.orientation = orientation;
  new_constraint.angular_velocity = angular_velocity;
  new_constraint.angular_acceleration = angular_acceleration;
  constraints.push_back(new_constraint);
}

Eigen::Quaterniond expMap(const Eigen::Vector3d & rotation_vector)
{
  if (!rotation_vector.allFinite()) {
    throw std::invalid_argument("Rotation vector must contain only finite values.");
  }

  const double theta = rotation_vector.stableNorm();
  if (!std::isfinite(theta)) {
    throw std::invalid_argument("Rotation vector norm must be finite.");
  }

  const double half_theta = 0.5 * theta;
  Eigen::Quaterniond quaternion;
  if (theta < 1e-3) {
    const double theta_squared = theta * theta;
    const double theta_fourth = theta_squared * theta_squared;
    const double vector_scale = 0.5 - theta_squared / 48.0 + theta_fourth / 3840.0;
    quaternion = Eigen::Quaterniond(
      1.0 - theta_squared / 8.0 + theta_fourth / 384.0, vector_scale * rotation_vector.x(),
      vector_scale * rotation_vector.y(), vector_scale * rotation_vector.z());
  } else {
    const Eigen::Vector3d axis = rotation_vector / theta;
    const double sin_half_theta = std::sin(half_theta);
    quaternion = Eigen::Quaterniond(
      std::cos(half_theta), sin_half_theta * axis.x(), sin_half_theta * axis.y(),
      sin_half_theta * axis.z());
  }
  quaternion.normalize();
  return quaternion;
}

Eigen::Vector3d logMap(const Eigen::Quaterniond & q)
{
  if (!q.coeffs().allFinite()) {
    throw std::invalid_argument("Quaternion must contain only finite values.");
  }
  const double scale = q.coeffs().cwiseAbs().maxCoeff();
  if (scale == 0.0) {
    throw std::invalid_argument("Quaternion must be non-zero.");
  }

  const Eigen::Vector4d scaled = q.coeffs() / scale;
  Eigen::Quaterniond normalized(scaled / scaled.norm());
  Eigen::Index dominant_axis;
  normalized.vec().cwiseAbs().maxCoeff(&dominant_axis);
  if (normalized.w() < 0.0 || (normalized.w() == 0.0 && normalized.vec()(dominant_axis) < 0.0)) {
    normalized.coeffs() *= -1.0;
  }

  const double sin_half_theta = normalized.vec().stableNorm();
  if (sin_half_theta < 1e-3) {
    const double squared = sin_half_theta * sin_half_theta;
    const double vector_scale = 2.0 + squared / 3.0 + 3.0 * squared * squared / 20.0;
    return vector_scale * normalized.vec();
  }

  const double theta = 2.0 * std::atan2(sin_half_theta, normalized.w());
  return (theta / sin_half_theta) * normalized.vec();
}

}  // namespace traj_gen
