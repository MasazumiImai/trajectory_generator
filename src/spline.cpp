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

#include "traj_gen/spline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traj_gen
{
namespace
{

void validateQueryTime(double time)
{
  if (!std::isfinite(time)) {
    throw std::invalid_argument("Query time must be finite.");
  }
}

Eigen::Quaterniond normalizedQuaternion(const Eigen::Quaterniond & quaternion)
{
  if (!quaternion.coeffs().allFinite()) {
    throw std::invalid_argument("Quaternion must contain only finite values.");
  }
  const double scale = quaternion.coeffs().cwiseAbs().maxCoeff();
  if (scale == 0.0) {
    throw std::invalid_argument("Quaternion must be non-zero.");
  }

  const Eigen::Vector4d scaled = quaternion.coeffs() / scale;
  return Eigen::Quaterniond(scaled / scaled.norm());
}

}  // namespace

VectorSpline::VectorSpline(const std::vector<VectorStateConstraint> & constraints, int dof)
: kDof_(dof)
{
  auto map = constraintsToMap(constraints, kDof_);
  start_time_ = map.begin()->first;
  end_time_ = map.rbegin()->first;
  start_position_ = map.begin()->second.at(0);
  end_position_ = map.rbegin()->second.at(0);
  coefficients_ = solveSplineCoefficients(map, kDof_);
}

Eigen::VectorXd VectorSpline::getPosition(double time)
{
  validateQueryTime(time);
  if (time < start_time_) {
    return start_position_;
  }
  if (time > end_time_) {
    return end_position_;
  }

  Eigen::VectorXd position(kDof_);
  for (int i = 0; i < kDof_; ++i) {
    double pos_i = 0.0;
    for (int j = 0; j < coefficients_.rows(); ++j) {
      pos_i += coefficients_(j, i) * std::pow(time, j);
    }
    position(i) = pos_i;
  }
  if (!position.allFinite()) {
    throw std::runtime_error("Spline position evaluation produced a non-finite value.");
  }
  return position;
}

Eigen::VectorXd VectorSpline::getVelocity(double time)
{
  validateQueryTime(time);
  if (time < start_time_ || time > end_time_) {
    return Eigen::VectorXd::Zero(kDof_);
  }

  Eigen::VectorXd velocity(kDof_);
  for (int i = 0; i < kDof_; ++i) {
    double vel_i = 0.0;
    for (int j = 1; j < coefficients_.rows(); ++j) {
      vel_i += coefficients_(j, i) * j * std::pow(time, j - 1);
    }
    velocity(i) = vel_i;
  }
  if (!velocity.allFinite()) {
    throw std::runtime_error("Spline velocity evaluation produced a non-finite value.");
  }
  return velocity;
}

std::map<double, std::map<int, Eigen::VectorXd>> VectorSpline::constraintsToMap(
  const std::vector<VectorStateConstraint> & constraints, int dof)
{
  if (dof <= 0) {
    throw std::invalid_argument("Degrees of freedom must be positive.");
  }
  if (constraints.empty()) {
    throw std::invalid_argument("At least one waypoint is required.");
  }

  const auto validate_value = [dof](const Eigen::VectorXd & value) {
    if (value.size() != static_cast<Eigen::Index>(dof)) {
      throw std::invalid_argument("Waypoint vector size must match degrees of freedom.");
    }
    if (!value.allFinite()) {
      throw std::invalid_argument("Waypoint vectors must contain only finite values.");
    }
  };

  std::map<double, std::map<int, Eigen::VectorXd>> constraints_map;
  for (const auto & constraint : constraints) {
    if (!std::isfinite(constraint.time)) {
      throw std::invalid_argument("Waypoint time must be finite.");
    }
    if (!constraint.position) {
      throw std::invalid_argument("Every waypoint must specify position.");
    }
    if (constraint.acceleration && !constraint.velocity) {
      throw std::invalid_argument("Acceleration requires velocity at the same waypoint.");
    }

    validate_value(*constraint.position);
    if (constraint.velocity) {
      validate_value(*constraint.velocity);
    }
    if (constraint.acceleration) {
      validate_value(*constraint.acceleration);
    }

    auto [waypoint, inserted] = constraints_map.try_emplace(constraint.time);
    if (!inserted) {
      throw std::invalid_argument("Waypoint times must be unique.");
    }
    waypoint->second.emplace(0, *constraint.position);
    if (constraint.velocity) {
      waypoint->second.emplace(1, *constraint.velocity);
    }
    if (constraint.acceleration) {
      waypoint->second.emplace(2, *constraint.acceleration);
    }
  }
  return constraints_map;
}

Eigen::MatrixXd VectorSpline::solveSplineCoefficients(
  const std::map<double, std::map<int, Eigen::VectorXd>> & constraints_map, int dof)
{
  int num_constraints = 0;
  for (const auto & [time, state_map] : constraints_map) {
    num_constraints += state_map.size();
  }

  // Basis matrix
  Eigen::MatrixXd T = Eigen::MatrixXd::Zero(num_constraints, num_constraints);
  // Spline value vector
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(num_constraints, dof);

  int row = 0;
  for (const auto & [time, state_map] : constraints_map) {
    for (const auto & [derivative_order, value] : state_map) {
      X.row(row) = value.transpose();
      for (int col = 0; col < num_constraints; ++col) {
        if (derivative_order == 0) {  // position
          T(row, col) = std::pow(time, col);
        } else if (derivative_order == 1) {  // velocity
          if (col >= 1) T(row, col) = col * std::pow(time, col - 1);
        } else if (derivative_order == 2) {  // acceleration
          if (col >= 2) T(row, col) = col * (col - 1) * std::pow(time, col - 2);
        }
      }
      ++row;
    }
  }
  if (!T.allFinite()) {
    throw std::runtime_error("Spline basis matrix contains non-finite values.");
  }

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> decomposition(T);
  if (decomposition.rank() != num_constraints) {
    throw std::runtime_error("Spline constraints are numerically rank deficient.");
  }

  const Eigen::MatrixXd coefficients = decomposition.solve(X);  // coeffs = T^-1 * X
  if (!coefficients.allFinite()) {
    throw std::runtime_error("Spline coefficient solve produced non-finite values.");
  }

  const Eigen::MatrixXd residual = T * coefficients - X;
  constexpr double kResidualTolerance = 1e-9;
  const Eigen::ArrayXXd allowed_residual = kResidualTolerance * (1.0 + X.array().abs());
  if (!residual.allFinite() || (residual.array().abs() > allowed_residual).any()) {
    throw std::runtime_error("Spline constraints could not be satisfied accurately.");
  }
  return coefficients;  // num_constraints x dof
}

OrientationSpline::OrientationSpline(const std::vector<AngularStateConstraint> & constraints)
: vector_spline_(buildVectorConstraints(constraints), 3)
{
  const auto [start, end] = std::minmax_element(
    constraints.begin(), constraints.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.time < rhs.time; });
  start_orientation_ = normalizedQuaternion(*start->orientation);
  end_orientation_ = normalizedQuaternion(*end->orientation);
  start_time_ = start->time;
  end_time_ = end->time;
}

Eigen::Quaterniond OrientationSpline::getOrientation(double time)
{
  validateQueryTime(time);
  if (time < start_time_) {
    return start_orientation_;
  }
  if (time > end_time_) {
    return end_orientation_;
  }

  Eigen::Vector3d delta_omega = vector_spline_.getPosition(time);
  Eigen::Quaterniond delta_q = expMap(delta_omega);
  return delta_q * start_orientation_;
}

Eigen::Vector3d OrientationSpline::getAngularVelocity(double time)
{
  validateQueryTime(time);
  if (time < start_time_ || time > end_time_) {
    return Eigen::Vector3d::Zero();
  }
  return vector_spline_.getVelocity(time);
}

std::vector<VectorStateConstraint> OrientationSpline::buildVectorConstraints(
  const std::vector<AngularStateConstraint> & constraints)
{
  if (constraints.empty()) {
    throw std::invalid_argument("At least one orientation waypoint is required.");
  }

  for (const auto & constraint : constraints) {
    if (!std::isfinite(constraint.time)) {
      throw std::invalid_argument("Orientation waypoint time must be finite.");
    }
    if (!constraint.orientation) {
      throw std::invalid_argument("Every orientation waypoint must specify orientation.");
    }
    if (constraint.angular_acceleration && !constraint.angular_velocity) {
      throw std::invalid_argument(
        "Angular acceleration requires angular "
        "velocity at the same waypoint.");
    }
    normalizedQuaternion(*constraint.orientation);
    if (constraint.angular_velocity && !constraint.angular_velocity->allFinite()) {
      throw std::invalid_argument("Angular velocity must contain only finite values.");
    }
    if (constraint.angular_acceleration && !constraint.angular_acceleration->allFinite()) {
      throw std::invalid_argument("Angular acceleration must contain only finite values.");
    }
  }

  const auto start = std::min_element(
    constraints.begin(), constraints.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.time < rhs.time; });
  const Eigen::Quaterniond start_orientation = normalizedQuaternion(*start->orientation);

  std::vector<VectorStateConstraint> vector_constraints;
  vector_constraints.reserve(constraints.size());
  for (const auto & ac : constraints) {
    VectorStateConstraint vc;
    vc.time = ac.time;
    // Calculate the difference quaternion and convert it to a rotation vector
    // using a logarithmic mapping.
    const Eigen::Quaterniond orientation = normalizedQuaternion(*ac.orientation);
    const Eigen::Quaterniond delta_q = orientation * start_orientation.conjugate();
    vc.position = logMap(delta_q);
    if (ac.angular_velocity) {
      vc.velocity = *ac.angular_velocity;
    }
    if (ac.angular_acceleration) {
      vc.acceleration = *ac.angular_acceleration;
    }
    vector_constraints.push_back(vc);
  }
  return vector_constraints;
}

}  // namespace traj_gen
