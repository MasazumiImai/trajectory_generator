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
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "trajectory_generator/visibility_control.hpp"

namespace trajectory_generator
{

/**
 * @brief Constraints for N-dimensional vector spaces (joint space, work space position, etc.).
 */
struct VectorStateConstraint
{
  double time;  // [s]
  std::optional<Eigen::VectorXd> position;
  std::optional<Eigen::VectorXd> velocity;
  std::optional<Eigen::VectorXd> acceleration;
};

/**
 * @brief Create a list of constraints with boundary conditions (start and end points) for vectors.
 */
TRAJECTORY_GENERATOR_PUBLIC
std::vector<VectorStateConstraint> createBoundaryConditions(
  const VectorStateConstraint & start_constraint, const VectorStateConstraint & end_constraint);

/**
 * @brief Add a new constraint to the existing constraint list.
 */
TRAJECTORY_GENERATOR_PUBLIC
void addConstraint(
  std::vector<VectorStateConstraint> & constraints, const VectorStateConstraint & new_constraint);

/**
 * @brief Constraints for orientation (quaternion).
 */
struct AngularStateConstraint
{
  double time;  // [s]
  std::optional<Eigen::Quaterniond> orientation;
  std::optional<Eigen::Vector3d> angular_velocity;
  std::optional<Eigen::Vector3d> angular_acceleration;
};

/**
 * @brief Create a list of constraints with boundary conditions (start and end points) for orientation.
 */
TRAJECTORY_GENERATOR_PUBLIC
std::vector<AngularStateConstraint> createBoundaryConditions(
  const AngularStateConstraint & start_constraint, const AngularStateConstraint & end_constraint);

/**
 * @brief Add a new constraint to the existing constraint list.
 */
TRAJECTORY_GENERATOR_PUBLIC
void addConstraint(
  std::vector<AngularStateConstraint> & constraints, const AngularStateConstraint & new_constraint);

// --- Mathematical utility functions for quaternion calculations ---

TRAJECTORY_GENERATOR_PUBLIC
Eigen::Quaterniond expMap(const Eigen::Vector3d & omega);

TRAJECTORY_GENERATOR_PUBLIC
Eigen::Vector3d logMap(const Eigen::Quaterniond & q);

}  // namespace trajectory_generator

#endif  // TRAJECTORY_GENERATOR__TRAJECTORY_GENERATOR_HPP_
