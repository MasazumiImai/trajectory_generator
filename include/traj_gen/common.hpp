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

#ifndef TRAJ_GEN__COMMON_HPP_
#define TRAJ_GEN__COMMON_HPP_

#include <Eigen/Dense>
#include <limits>
#include <optional>
#include <vector>

#include "traj_gen/visibility_control.h"

namespace traj_gen
{

/**
 * @brief Constraints for N-dimensional vector spaces (joint space, work space position, etc.).
 */
struct VectorStateConstraint
{
  double time{std::numeric_limits<double>::quiet_NaN()};  // [s]
  std::optional<Eigen::VectorXd> position;
  std::optional<Eigen::VectorXd> velocity;
  std::optional<Eigen::VectorXd> acceleration;
};

/**
 * @brief Create a list of constraints with boundary conditions (start and end points) for vectors.
 */
TRAJ_GEN_PUBLIC
std::vector<VectorStateConstraint> createBoundaryConditions(
  const VectorStateConstraint & start_constraint, const VectorStateConstraint & end_constraint);

/**
 * @brief Add a new constraint to the existing constraint list.
 */
TRAJ_GEN_PUBLIC
void addConstraint(
  std::vector<VectorStateConstraint> & constraints, const VectorStateConstraint & new_constraint);

/**
 * @brief Add a new constraint with separated position, velocity, and acceleration.
 */
TRAJ_GEN_PUBLIC
void addConstraint(
  std::vector<VectorStateConstraint> & constraints, double time,
  const std::optional<Eigen::VectorXd> & position, const std::optional<Eigen::VectorXd> & velocity,
  const std::optional<Eigen::VectorXd> & acceleration);

/**
 * @brief Constraints for orientation (quaternion).
 */
struct AngularStateConstraint
{
  double time{std::numeric_limits<double>::quiet_NaN()};  // [s]
  std::optional<Eigen::Quaterniond> orientation;
  std::optional<Eigen::Vector3d> angular_velocity;  // Spatial/world frame [rad/s]
  std::optional<Eigen::Vector3d> angular_acceleration;  // Spatial/world frame [rad/s^2]
};

/**
 * @brief Create a list of constraints with boundary conditions (start and end points) for orientation.
 */
TRAJ_GEN_PUBLIC
std::vector<AngularStateConstraint> createBoundaryConditions(
  const AngularStateConstraint & start_constraint, const AngularStateConstraint & end_constraint);

/**
 * @brief Add a new constraint to the existing constraint list.
 */
TRAJ_GEN_PUBLIC
void addConstraint(
  std::vector<AngularStateConstraint> & constraints, const AngularStateConstraint & new_constraint);

/**
 * @brief Add a new constraint with separated orientation, velocity, and acceleration.
 */
TRAJ_GEN_PUBLIC
void addConstraint(
  std::vector<AngularStateConstraint> & constraints, double time,
  const std::optional<Eigen::Quaterniond> & orientation,
  const std::optional<Eigen::Vector3d> & angular_velocity,
  const std::optional<Eigen::Vector3d> & angular_acceleration);

// --- Mathematical utility functions for quaternion calculations ---

TRAJ_GEN_PUBLIC
Eigen::Quaterniond expMap(const Eigen::Vector3d & omega);

/** @brief Return the principal rotation vector (norm at most pi) for a
 * quaternion. */
TRAJ_GEN_PUBLIC
Eigen::Vector3d logMap(const Eigen::Quaterniond & q);

}  // namespace traj_gen

#endif  // TRAJ_GEN__COMMON_HPP_
