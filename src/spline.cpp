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

#include "trajectory_generator/spline.hpp"

#define DEBUG false

namespace trajectory_generator
{

VectorSpline::VectorSpline(const std::vector<VectorStateConstraint> & constraints, int dof)
: kDof_(dof)
{
  auto map = constraintsToMap(constraints);
  coefficients_ = solveSplineCoefficients(map, kDof_);
}

Eigen::VectorXd VectorSpline::getPosition(double time)
{
  Eigen::VectorXd position(kDof_);
  for (int i = 0; i < kDof_; ++i) {
    double pos_i = 0.0;
    for (int j = 0; j < coefficients_.rows(); ++j) {
      pos_i += coefficients_(j, i) * std::pow(time, j);
    }
    position(i) = pos_i;
  }
  return position;
}

Eigen::VectorXd VectorSpline::getVelocity(double time)
{
  Eigen::VectorXd velocity(kDof_);
  for (int i = 0; i < kDof_; ++i) {
    double vel_i = 0.0;
    for (int j = 1; j < coefficients_.rows(); ++j) {
      vel_i += coefficients_(j, i) * j * std::pow(time, j - 1);
    }
    velocity(i) = vel_i;
  }
  return velocity;
}

std::map<double, std::map<int, Eigen::VectorXd>> VectorSpline::constraintsToMap(
  const std::vector<VectorStateConstraint> & constraints)
{
  std::map<double, std::map<int, Eigen::VectorXd>> constraints_map;
  for (const auto & c : constraints) {
    if (c.position) {
      constraints_map[c.time][0] = *c.position;
    }
    if (c.velocity) {
      constraints_map[c.time][1] = *c.velocity;
    }
    if (c.acceleration) {
      constraints_map[c.time][2] = *c.acceleration;
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
      X.row(row) = value;
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
  const Eigen::MatrixXd coefficients = T.colPivHouseholderQr().solve(X);  // coeffs = T^-1 * X
  return coefficients;  // num_constraints x dof
}

OrientationSpline::OrientationSpline(const std::vector<AngularStateConstraint> & constraints)
: vector_spline_(buildVectorConstraints(constraints), 3),
  start_orientation_(*constraints.front().orientation)
{
}

Eigen::Quaterniond OrientationSpline::getOrientation(double time)
{
  Eigen::Vector3d delta_omega = vector_spline_.getPosition(time);
  Eigen::Quaterniond delta_q = expMap(delta_omega);
  return delta_q * start_orientation_;
}

Eigen::Vector3d OrientationSpline::getAngularVelocity(double time)
{
  return vector_spline_.getVelocity(time);
}

std::vector<VectorStateConstraint> OrientationSpline::buildVectorConstraints(
  const std::vector<AngularStateConstraint> & constraints)
{
  if (constraints.empty() || !constraints.front().orientation) {
    throw std::invalid_argument("Start orientation is required.");
  }
  const Eigen::Quaterniond start_orientation = *constraints.front().orientation;

  std::vector<VectorStateConstraint> vector_constraints;
  for (const auto & ac : constraints) {
    VectorStateConstraint vc;
    vc.time = ac.time;
    if (ac.orientation) {
      // Calculate the difference quaternion and convert it to a rotation vector using a logarithmic mapping.
      Eigen::Quaterniond delta_q = (*ac.orientation) * start_orientation.inverse();
      vc.position = logMap(delta_q);
    }
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

}  // namespace trajectory_generator
