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

#include "trajectory_generator/trajectory_generator.hpp"

#define DEBUG false

namespace trajectory_generator
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

Eigen::Quaterniond expMap(const Eigen::Vector3d & omega)
{
  const double theta = omega.norm();
  if (theta < 1e-9) {
    return Eigen::Quaterniond::Identity();
  }

  const Eigen::Vector3d axis = omega.normalized();
  const double angle = theta;

  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));
  return q;
}

Eigen::Vector3d logMap(const Eigen::Quaterniond & q)
{
  Eigen::AngleAxisd angle_axis(q);
  Eigen::Vector3d omega = angle_axis.angle() * angle_axis.axis();
  return omega;
}

}  // namespace trajectory_generator
