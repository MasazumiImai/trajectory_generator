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

#ifndef TRAJECTORY_GENERATOR__SPLINE_HPP_
#define TRAJECTORY_GENERATOR__SPLINE_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "trajectory_generator/trajectory_generator.hpp"
#include "trajectory_generator/visibility_control.hpp"

namespace trajectory_generator
{

/**
 * @brief Class for generating and managing spline trajectories of N-dimensional vectors (joint angles, positions, etc.).
 */
class VectorSpline
{
public:
  TRAJECTORY_GENERATOR_PUBLIC
  explicit VectorSpline(const std::vector<VectorStateConstraint> & constraints, int dof);
  virtual ~VectorSpline() = default;

  /** @brief Get position vector at specified time. */
  Eigen::VectorXd getPosition(double time);

  /** @brief Get velocity vector at specified time. */
  Eigen::VectorXd getVelocity(double time);

private:
  static std::map<double, std::map<int, Eigen::VectorXd>> constraintsToMap(
    const std::vector<VectorStateConstraint> & constraints);

  static Eigen::MatrixXd solveSplineCoefficients(
    const std::map<double, std::map<int, Eigen::VectorXd>> & constraints_map, int dof);

  const int kDof_;  // Degree of freedom for trajectory (number of order)
  Eigen::MatrixXd coefficients_;  // Spline coefficients
};

/**
 * @brief Class for generating and managing spline trajectories of orientation (quaternion).
 */
class OrientationSpline
{
public:
  TRAJECTORY_GENERATOR_PUBLIC
  explicit OrientationSpline(const std::vector<AngularStateConstraint> & constraints);
  virtual ~OrientationSpline() = default;

  /** @brief Get orientation (quaternion) at specified time. */
  Eigen::Quaterniond getOrientation(double time);

  /** @brief Get angular velocity vector at specified time. */
  Eigen::Vector3d getAngularVelocity(double time);

private:
  static std::vector<VectorStateConstraint> buildVectorConstraints(
    const std::vector<AngularStateConstraint> & constraints);

  VectorSpline vector_spline_;  // Use VectorSpline internally to interpolate rotation vectors.
  Eigen::Quaterniond start_orientation_;  // Reference starting orientation
};

}  // namespace trajectory_generator

#endif  // TRAJECTORY_GENERATOR__SPLINE_HPP_
