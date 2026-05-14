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

#ifndef TRAJ_GEN__BASE_HPP_
#define TRAJ_GEN__BASE_HPP_

#include <Eigen/Dense>

#include "traj_gen/visibility_control.h"

namespace traj_gen
{

class TRAJ_GEN_PUBLIC VectorTrajectoryBase
{
public:
  virtual ~VectorTrajectoryBase() = default;

  virtual Eigen::VectorXd getPosition(double time) = 0;

  virtual Eigen::VectorXd getVelocity(double time) = 0;
};

class TRAJ_GEN_PUBLIC OrientationTrajectoryBase
{
public:
  virtual ~OrientationTrajectoryBase() = default;

  virtual Eigen::Quaterniond getOrientation(double time) = 0;

  virtual Eigen::Vector3d getAngularVelocity(double time) = 0;
};

}  // namespace traj_gen

#endif  // TRAJ_GEN__BASE_HPP_
