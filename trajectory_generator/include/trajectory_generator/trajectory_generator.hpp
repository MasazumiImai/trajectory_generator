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

#ifndef TRAJECTORY_GENERATOR__TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_GENERATOR__TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "trajectory_generator/visibility_control.hpp"

namespace trajectory_generator
{

// struct BoundaryCondition
// {
//   double time;
//   std::optional<Eigen::Vector3d> position;
//   std::optional<Eigen::Vector3d> orientation;
//   std::optional<Eigen::Vector3d> twist;
//   std::optional<Eigen::Vector3d> accel;
// };

struct LinearStateConstraint
{
  double time;  // [s]
  std::optional<Eigen::Vector3d> position;  // [m]
  std::optional<Eigen::Vector3d> linear_velocity;  // [m/s]
  std::optional<Eigen::Vector3d> linear_acceleration;  // [m/s^2]
};

struct AngularStateConstraint
{
  double time;
  std::optional<Eigen::Quaterniond> orientation;
  std::optional<Eigen::Vector3d> angular_velocity;
  std::optional<Eigen::Vector3d> angular_acceleration;
};

class TrajectoryGenerator
{
public:
  TRAJECTORY_GENERATOR_PUBLIC
  explicit TrajectoryGenerator();

  virtual ~TrajectoryGenerator();

  /**
   * @brief Add constraint
   *
   * @param constraints
   * @param time  // [ms]
   * @param position  // [m]
   * @param linear_velocity  // [m/s]
   * @param linear_acceleration  // [m/s^2]
   */
  static void addConstraint(
    std::vector<LinearStateConstraint> & constraints, const int & time,
    const Eigen::Vector3d & position,
    const std::optional<Eigen::Vector3d> & linear_velocity = std::nullopt,
    const std::optional<Eigen::Vector3d> & linear_acceleration = std::nullopt);
  static void addConstraint(
    std::vector<AngularStateConstraint> & constraints, const int & time,
    const Eigen::Quaterniond & orientation,
    const std::optional<Eigen::Vector3d> & angular_velocity = std::nullopt,
    const std::optional<Eigen::Vector3d> & angular_acceleration = std::nullopt);

  /**
   * @brief Create a constraint object
   *
   * @param time  // [ms]
   * @param position  // [m]
   * @param linear_velocity  // [m/s]
   * @param linear_acceleration  // [m/s^2]
   * @return std::vector<LinearStateConstraint>
   */
  static std::vector<LinearStateConstraint> createConstraint(
    const int & time, const Eigen::Vector3d & position,
    const std::optional<Eigen::Vector3d> & linear_velocity = std::nullopt,
    const std::optional<Eigen::Vector3d> & linear_acceleration = std::nullopt);
  static std::vector<AngularStateConstraint> createConstraint(
    const int & time, const Eigen::Quaterniond & orientation,
    const std::optional<Eigen::Vector3d> & angular_velocity = std::nullopt,
    const std::optional<Eigen::Vector3d> & angular_acceleration = std::nullopt);

  static std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> getConstraintsMap(
    const std::vector<LinearStateConstraint> & constraints);
  static std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> getConstraintsMap(
    const std::vector<AngularStateConstraint> & constraints);

  static int getNumberOfConstraints(
    const std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> &
      constraints_map);

  static bool isStartFromZero(const double & start_time);

  static int calculateMotionDuration(const double & start_time, const double & final_time);

  static Eigen::Quaterniond expMap(const Eigen::Vector3d & omega);

  static Eigen::Vector3d logMap(const Eigen::Quaterniond & q_in);

private:
};

}  // namespace trajectory_generator

#endif  // TRAJECTORY_GENERATOR__TRAJECTORY_GENERATOR_HPP_
