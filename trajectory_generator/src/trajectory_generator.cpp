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

#define DEBUG_ENABLED false

namespace trajectory_generator
{

TrajectoryGenerator::TrajectoryGenerator()
{
  std::cout << "TrajectoryGenerator class is established." << std::endl;
}

TrajectoryGenerator::~TrajectoryGenerator()
{
  std::cout << "TrajectoryGenerator class is destructed." << std::endl;
}

void TrajectoryGenerator::addConstraint(
  std::vector<LinearStateConstraint> & constraints, const int & time,
  const Eigen::Vector3d & position, const std::optional<Eigen::Vector3d> & linear_velocity,
  const std::optional<Eigen::Vector3d> & linear_acceleration)
{
  auto new_constraint = createConstraint(time, position, linear_velocity, linear_acceleration);

  constraints.push_back(new_constraint.at(0));
}

void TrajectoryGenerator::addConstraint(
  std::vector<AngularStateConstraint> & constraints, const int & time,
  const Eigen::Quaterniond & orientation, const std::optional<Eigen::Vector3d> & angular_velocity,
  const std::optional<Eigen::Vector3d> & angular_acceleration)
{
  auto new_constraint = createConstraint(time, orientation, angular_velocity, angular_acceleration);

  constraints.push_back(new_constraint.at(0));
}

std::vector<LinearStateConstraint> TrajectoryGenerator::createConstraint(
  const int & time, const Eigen::Vector3d & position,
  const std::optional<Eigen::Vector3d> & linear_velocity,
  const std::optional<Eigen::Vector3d> & linear_acceleration)
{
  std::vector<LinearStateConstraint> constraint(1);
  constraint.at(0).time = (double)time * 0.001;  // [ms]
  constraint.at(0).position = position;
  if (linear_velocity) {
    constraint.at(0).linear_velocity = linear_velocity;
  }
  if (linear_acceleration) {
    constraint.at(0).linear_acceleration = linear_acceleration;
  }
  return constraint;
}

std::vector<AngularStateConstraint> TrajectoryGenerator::createConstraint(
  const int & time, const Eigen::Quaterniond & orientation,
  const std::optional<Eigen::Vector3d> & angular_velocity,
  const std::optional<Eigen::Vector3d> & angular_acceleration)
{
  std::vector<AngularStateConstraint> constraint(1);
  constraint.at(0).time = (double)time * 0.001;  // [ms]
  constraint.at(0).orientation = orientation;
  if (angular_velocity) {
    constraint.at(0).angular_velocity = angular_velocity;
  }
  if (angular_acceleration) {
    constraint.at(0).angular_acceleration = angular_acceleration;
  }
  return constraint;
}

std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>>
TrajectoryGenerator::getConstraintsMap(const std::vector<LinearStateConstraint> & constraints)
{
  std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> constraints_map;

  for (const LinearStateConstraint & condition : constraints) {
    if (condition.position) {
      constraints_map[condition.time]["x"] = condition.position.value();
    }
    if (condition.linear_velocity) {
      constraints_map[condition.time]["dx"] = condition.linear_velocity.value();
    }
    if (condition.linear_acceleration) {
      constraints_map[condition.time]["ddx"] = condition.linear_acceleration.value();
    }
  }
  return constraints_map;
}

std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>>
TrajectoryGenerator::getConstraintsMap(const std::vector<AngularStateConstraint> & constraints)
{
  std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> constraints_map;

  for (const AngularStateConstraint & condition : constraints) {
    if (condition.orientation) {
      Eigen::Vector3d theta = logMap(*condition.orientation);
      constraints_map[condition.time]["x"] = theta;
    }
    if (condition.angular_velocity) {
      constraints_map[condition.time]["dx"] = condition.angular_velocity.value();
    }
    if (condition.angular_acceleration) {
      constraints_map[condition.time]["ddx"] = condition.angular_acceleration.value();
    }
  }
  return constraints_map;
}

int TrajectoryGenerator::getNumberOfConstraints(
  const std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> & constraints_map)
{
  int num_of_constraints = 0;
  for (const auto & [time, state_map] : constraints_map) {
    num_of_constraints += state_map.size();
  }
  return num_of_constraints;
}

bool TrajectoryGenerator::isStartFromZero(const double & start_time)
{
  return start_time == 0.0;
}

int TrajectoryGenerator::calculateMotionDuration(
  const double & start_time, const double & final_time)
{
  double motion_duration = final_time - start_time;  // [s]
  int motion_duration_ms = (int)(motion_duration * 1000.0);  // [ms]
  return motion_duration_ms;
}

Eigen::Quaterniond TrajectoryGenerator::expMap(const Eigen::Vector3d & omega)
{
  double theta = omega.norm();
  Eigen::Quaterniond q;

  if (theta < 1e-10) {
    q.w() = 1.0;
    q.vec() = 0.5 * omega;
  } else {
    Eigen::Vector3d axis = omega / theta;
    double half_theta = 0.5 * theta;
    q.w() = std::cos(half_theta);
    q.vec() = axis * std::sin(half_theta);
  }

  return q.normalized();
}

Eigen::Vector3d TrajectoryGenerator::logMap(const Eigen::Quaterniond & q_in)
{
  Eigen::Quaterniond q = q_in.normalized();
  double q_w = q.w();
  Eigen::Vector3d q_v = q.vec();

  double sin_theta = q_v.norm();

  if (sin_theta < 1e-10) {
    return 2.0 * q_v;
  } else {
    double theta = 2.0 * std::atan2(sin_theta, q_w);
    Eigen::Vector3d axis = q_v / sin_theta;
    return theta * axis;
  }
}

}  // namespace trajectory_generator
