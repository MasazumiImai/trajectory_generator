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

TEST(TrajectoryGenerator, VectorSplineTaskSpace)
{
  const double start_time = 0.0;
  const double mid_time = 0.5;
  const double end_time = 1.0;
  const double threshold = 1e-5;

  VectorStateConstraint start_constraint;
  start_constraint.time = start_time;
  start_constraint.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  start_constraint.velocity = Eigen::Vector3d(-0.5, 0.0, 0.5);
  start_constraint.acceleration = Eigen::Vector3d::Zero();

  VectorStateConstraint end_constraint;
  end_constraint.time = end_time;
  end_constraint.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  end_constraint.velocity = Eigen::Vector3d(-0.5, 0.0, -0.5);
  end_constraint.acceleration = Eigen::Vector3d::Zero();

  VectorStateConstraint mid_constraint;
  mid_constraint.time = mid_time;
  mid_constraint.position = Eigen::Vector3d(0.5, 0.0, 0.5);
  mid_constraint.velocity = Eigen::Vector3d(0.5, 0.0, 0.5);

  // std::vector<VectorStateConstraint> constraints;
  // constraints.push_back(start_constraint);
  // constraints.push_back(mid_constraint);
  // constraints.push_back(end_constraint);
  std::vector<VectorStateConstraint> constraints =
    createBoundaryConditions(start_constraint, end_constraint);
  addConstraint(constraints, mid_constraint);

  VectorSpline spline(constraints, 3);

  std::cout << "\n--- [VectorSpline Task Space Trajectory] ---" << std::endl;
  std::cout << "Time[s], Pos.x, Pos.y, Pos.z, Vel.x, Vel.y, Vel.z" << std::endl;

  const double step_time = 0.1;
  for (double t = start_time; t <= end_time; t += step_time) {
    Eigen::VectorXd pos = spline.getPosition(t);
    Eigen::VectorXd vel = spline.getVelocity(t);

    std::cout << std::fixed << std::setprecision(4) << t << ", " << pos.transpose() << ", "
              << vel.transpose() << std::endl;
  }
  std::cout << "------------------------------------------\n" << std::endl;

  if (start_constraint.position) {
    Eigen::VectorXd actual_pos = spline.getPosition(start_time);
    ASSERT_EQ(start_constraint.position->size(), actual_pos.size());
    for (int i = 0; i < actual_pos.size(); ++i) {
      EXPECT_NEAR((*start_constraint.position)(i), actual_pos(i), threshold);
    }
  }
  if (start_constraint.velocity) {
    Eigen::VectorXd actual_vel = spline.getVelocity(start_time);
    ASSERT_EQ(start_constraint.velocity->size(), actual_vel.size());
    for (int i = 0; i < actual_vel.size(); ++i) {
      EXPECT_NEAR((*start_constraint.velocity)(i), actual_vel(i), threshold);
    }
  }

  if (mid_constraint.position) {
    Eigen::VectorXd actual_pos = spline.getPosition(mid_time);
    ASSERT_EQ(mid_constraint.position->size(), actual_pos.size());
    for (int i = 0; i < actual_pos.size(); ++i) {
      EXPECT_NEAR((*mid_constraint.position)(i), actual_pos(i), threshold);
    }
  }

  if (end_constraint.position) {
    Eigen::VectorXd actual_pos = spline.getPosition(end_time);
    ASSERT_EQ(end_constraint.position->size(), actual_pos.size());
    for (int i = 0; i < actual_pos.size(); ++i) {
      EXPECT_NEAR((*end_constraint.position)(i), actual_pos(i), threshold);
    }
  }
}

TEST(TrajectoryGenerator, VectorSplineJointSpace)
{
  const double start_time = 0.0;
  const double end_time = 5.0;
  const int dof = 7;
  const double threshold = 1e-5;

  VectorStateConstraint start_constraint;
  start_constraint.time = start_time;
  start_constraint.position = Eigen::VectorXd::Zero(dof);
  start_constraint.velocity = Eigen::VectorXd::Zero(dof);

  VectorStateConstraint end_constraint;
  end_constraint.time = end_time;
  Eigen::VectorXd end_pos(dof);
  end_pos << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7;
  end_constraint.position = end_pos;
  end_constraint.velocity = Eigen::VectorXd::Zero(dof);

  std::vector<VectorStateConstraint> constraints =
    createBoundaryConditions(start_constraint, end_constraint);

  VectorSpline spline(constraints, dof);

  Eigen::VectorXd actual_start_pos = spline.getPosition(start_constraint.time);
  for (int i = 0; i < dof; ++i) {
    EXPECT_NEAR((*start_constraint.position)(i), actual_start_pos(i), threshold);
  }

  Eigen::VectorXd actual_end_pos = spline.getPosition(end_constraint.time);
  for (int i = 0; i < dof; ++i) {
    EXPECT_NEAR((*end_constraint.position)(i), actual_end_pos(i), threshold);
  }
}

TEST(TrajectoryGenerator, OrientationSpline)
{
  const double start_time = 0.0;
  const double end_time = 1.0;
  const double threshold = 1e-5;

  AngularStateConstraint start_constraint;
  start_constraint.time = start_time;
  Eigen::Vector3d start_rpy(0.0, M_PI_2, M_PI);
  start_constraint.orientation = Eigen::AngleAxisd(start_rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(start_rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(start_rpy.x(), Eigen::Vector3d::UnitX());
  start_constraint.angular_velocity = Eigen::Vector3d::Zero();

  AngularStateConstraint end_constraint;
  end_constraint.time = end_time;
  Eigen::Vector3d end_rpy(0.0, 0.0, M_PI);
  end_constraint.orientation = Eigen::AngleAxisd(end_rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(end_rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(end_rpy.x(), Eigen::Vector3d::UnitX());
  end_constraint.angular_velocity = Eigen::Vector3d::Zero();

  std::vector<AngularStateConstraint> constraints =
    createBoundaryConditions(start_constraint, end_constraint);

  OrientationSpline spline(constraints);

  EXPECT_NEAR(
    std::abs(spline.getOrientation(start_time).dot(*start_constraint.orientation)), 1.0, threshold);
  EXPECT_NEAR(
    std::abs(spline.getOrientation(end_time).dot(*end_constraint.orientation)), 1.0, threshold);

  Eigen::Vector3d actual_start_vel = spline.getAngularVelocity(start_constraint.time);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR((*start_constraint.angular_velocity)(i), actual_start_vel(i), threshold);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace trajectory_generator
