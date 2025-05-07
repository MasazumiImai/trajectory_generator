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

#ifndef TRAJECTORY_GENERATOR__SPLINE_TRAJECTORY_HPP_
#define TRAJECTORY_GENERATOR__SPLINE_TRAJECTORY_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>

#include "trajectory_generator/trajectory_generator.hpp"
#include "trajectory_generator/visibility_control.hpp"

namespace trajectory_generator
{

class SplineTrajectory
{
public:
  TRAJECTORY_GENERATOR_PUBLIC
  explicit SplineTrajectory();

  virtual ~SplineTrajectory();

  static Eigen::Vector3d calculatePositionAtCertainTime(
    const double & time, const Eigen::MatrixXd & coefficients);
  static Eigen::Quaterniond calculateOrientationAtCertainTime(
    const double & time, const Eigen::MatrixXd & coefficients);

  static Eigen::Matrix3Xd calculateTrajectory(
    const std::vector<LinearStateConstraint> & constraints, const int & step_time);
  static Eigen::Matrix4Xd calculateTrajectory(
    const std::vector<AngularStateConstraint> & constraints, const int & step_time);

  static Eigen::Matrix3Xd calculateVelocityTrajectory(
    const std::vector<LinearStateConstraint> & constraints, const int & step_time);
  static Eigen::Matrix3Xd calculateVelocityTrajectory(
    const std::vector<AngularStateConstraint> & constraints, const int & step_time);

private:
  static Eigen::MatrixXd calculateCoefficients(
    const std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> &
      constraints_map);

  static Eigen::MatrixXd getSplineValueVector(
    const std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> &
      constraints_map);

  static Eigen::MatrixXd getBasisMatrix(
    const std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> &
      constraints_map);
};

}  // namespace trajectory_generator

#endif  // TRAJECTORY_GENERATOR__SPLINE_TRAJECTORY_HPP_
