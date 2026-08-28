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

#include <traj_gen/traj_gen.hpp>

int main()
{
  traj_gen::VectorStateConstraint vector_waypoint;
  vector_waypoint.time = 0.0;
  vector_waypoint.position = Eigen::VectorXd::Zero(1);
  vector_waypoint.velocity = Eigen::VectorXd::Zero(1);
  vector_waypoint.acceleration = Eigen::VectorXd::Zero(1);
  const traj_gen::VectorSpline vector_spline({vector_waypoint}, 1);
  const traj_gen::VectorTrajectoryBase & vector_trajectory = vector_spline;

  traj_gen::AngularStateConstraint angular_waypoint;
  angular_waypoint.time = 0.0;
  angular_waypoint.orientation = Eigen::Quaterniond::Identity();
  angular_waypoint.angular_velocity = Eigen::Vector3d::Zero();
  angular_waypoint.angular_acceleration = Eigen::Vector3d::Zero();
  const traj_gen::OrientationSpline orientation_spline({angular_waypoint});
  const traj_gen::OrientationTrajectoryBase & orientation_trajectory = orientation_spline;

  const bool valid = vector_trajectory.getPosition(0.0).isZero() &&
    vector_trajectory.getVelocity(0.0).isZero() &&
    vector_trajectory.getAcceleration(0.0).isZero() &&
    orientation_trajectory.getOrientation(0.0).isApprox(Eigen::Quaterniond::Identity()) &&
    orientation_trajectory.getAngularVelocity(0.0).isZero() &&
    orientation_trajectory.getAngularAcceleration(0.0).isZero() &&
    traj_gen::expMap(Eigen::Vector3d::Zero()).isApprox(Eigen::Quaterniond::Identity());
  return valid ? 0 : 1;
}
