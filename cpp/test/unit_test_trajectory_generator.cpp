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

#include <gtest/gtest.h>

#include "trajectory_generator/spline.hpp"
#include "trajectory_generator/trajectory_generator.hpp"

namespace trajectory_generator
{

TEST(TrajectoryGenerator, splineCoefficientsTestCase)
{
  // For linear
  // Constraints
  int kStartTime = 0;  // [ms]
  Eigen::Vector3d kStartPosition(0.0, 0.0, 0.0);  // [m]
  Eigen::Vector3d kStartLinearVelocity(-0.5, 0.0, 0.5);  // [m/s]
  Eigen::Vector3d kStartLinearAcceleration(0.0, 0.0, 0.0);  // [m/s^2]
  int kFinalTime = 1000;  // [ms]
  Eigen::Vector3d kFinalLinearPosition(1.0, 0.0, 0.0);
  Eigen::Vector3d kFinalLinearVelocity(-0.5, 0.0, -0.5);
  Eigen::Vector3d kFinalLinearAcceleration(0.0, 0.0, 0.0);

  auto constraints = TrajectoryGenerator::addBoundaryConditions(
    kStartTime, kStartPosition, kStartLinearVelocity, kStartLinearAcceleration, kFinalTime,
    kFinalLinearPosition, kFinalLinearVelocity, kFinalLinearAcceleration);

  int kMidTime = 500;  // [ms]
  Eigen::Vector3d kMidLinearPosition(0.5, 0.0, 0.5);
  Eigen::Vector3d kMidLinearVelocity(0.5, 0.0, 0.5);
  TrajectoryGenerator::addConstraint(constraints, kMidTime, kMidLinearPosition, kMidLinearVelocity);

  auto constraints_map = TrajectoryGenerator::getConstraintsMap(constraints);

  // Position trajectory
  const int kStepTime = 100;  // [ms]
  const Eigen::Matrix3Xd trajectory = Spline::calculateTrajectory(constraints, kStepTime);
  std::cout << "trajectory = " << std::endl << trajectory.transpose() << std::endl;

  // Check results
  const double kThreshold = 1e-5;
  const Eigen::Vector3d result_start_position = trajectory.col(0);
  EXPECT_LT((kStartPosition - result_start_position).norm(), kThreshold);
  const Eigen::Vector3d result_mid_position = trajectory.col(trajectory.cols() / 2);
  EXPECT_LT((kMidLinearPosition - result_mid_position).norm(), kThreshold);
  const Eigen::Vector3d result_final_position = trajectory.col(trajectory.cols() - 1);
  EXPECT_LT((kFinalLinearPosition - result_final_position).norm(), kThreshold);

  // Linear velocity trajectory
  const Eigen::Matrix3Xd velocity_trajectory =
    Spline::calculateVelocityTrajectory(constraints, kStepTime);
  std::cout << "velocity_trajectory = " << std::endl
            << velocity_trajectory.transpose() << std::endl;

  // For angular
  // Constraints
  Eigen::Vector3d kStartOrientationRPY(0.0, M_PI_2, M_PI);
  Eigen::Matrix3d kStartRotationMatrix;
  kStartRotationMatrix =
    Eigen::AngleAxisd(kStartOrientationRPY[2], Eigen::Vector3d::UnitZ()) *  // yaw
    Eigen::AngleAxisd(kStartOrientationRPY[1], Eigen::Vector3d::UnitY()) *  // pitch
    Eigen::AngleAxisd(kStartOrientationRPY[0], Eigen::Vector3d::UnitX());  // roll
  Eigen::Quaterniond kStartOrientation(kStartRotationMatrix);  // [m]
  std::cout << "kStartOrientation = " << std::endl << kStartOrientation << std::endl;
  Eigen::Vector3d kStartAngularVelocity(0.0, 0.0, 0.0);  // [m/s]
  Eigen::Vector3d kStartAngularAcceleration(0.0, 0.0, 0.0);  // [m/s^2]

  Eigen::Vector3d kFinalOrientationRPY(0.0, 0.0, M_PI);
  Eigen::Matrix3d kFinalRotationMatrix;
  kFinalRotationMatrix = Eigen::AngleAxisd(
                           kFinalOrientationRPY[2], Eigen::Vector3d::UnitZ()) *  // yaw
    Eigen::AngleAxisd(kFinalOrientationRPY[1], Eigen::Vector3d::UnitY()) *  // pitch
    Eigen::AngleAxisd(kFinalOrientationRPY[0],
                      Eigen::Vector3d::UnitX());  // roll
  Eigen::Quaterniond kFinalOrientation(kFinalRotationMatrix);
  std::cout << "kFinalOrientation = " << std::endl << kFinalOrientation << std::endl;
  Eigen::Vector3d kFinalAngularVelocity(0.0, 0.0, 0.0);
  Eigen::Vector3d kFinalAngularAcceleration(0.0, 0.0, 0.0);

  auto angular_constraints = TrajectoryGenerator::addBoundaryConditions(
    kStartTime, kStartOrientation, kStartAngularVelocity, kStartAngularAcceleration, kFinalTime,
    kFinalOrientation, kFinalAngularVelocity, kFinalAngularAcceleration);

  // Orientation trajectory
  const Eigen::Matrix4Xd angular_trajectory =
    Spline::calculateTrajectory(angular_constraints, kStepTime);
  std::cout << "angular_trajectory = " << std::endl << angular_trajectory.transpose() << std::endl;

  // Check results
  Eigen::Vector4d result_start_orientation_eigen = angular_trajectory.col(0);
  Eigen::Quaterniond result_start_orientation;
  result_start_orientation.x() = result_start_orientation_eigen(0);
  result_start_orientation.y() = result_start_orientation_eigen(1);
  result_start_orientation.z() = result_start_orientation_eigen(2);
  result_start_orientation.w() = result_start_orientation_eigen(3);
  EXPECT_NEAR(std::abs(kStartOrientation.dot(result_start_orientation)), 1.0, kThreshold);

  Eigen::Vector4d result_final_orientation_eigen =
    angular_trajectory.col(angular_trajectory.cols() - 1);
  Eigen::Quaterniond result_final_orientation;
  result_final_orientation.x() = result_final_orientation_eigen(0);
  result_final_orientation.y() = result_final_orientation_eigen(1);
  result_final_orientation.z() = result_final_orientation_eigen(2);
  result_final_orientation.w() = result_final_orientation_eigen(3);
  EXPECT_NEAR(std::abs(kFinalOrientation.dot(result_final_orientation)), 1.0, kThreshold);

  // Angular velocity trajectory
  const Eigen::Matrix3Xd angular_velocity_trajectory =
    Spline::calculateVelocityTrajectory(angular_constraints, kStepTime);
  std::cout << "angular_velocity_trajectory = " << std::endl
            << angular_velocity_trajectory.transpose() << std::endl;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace trajectory_generator
