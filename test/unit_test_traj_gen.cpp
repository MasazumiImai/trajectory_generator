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

#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

#include "traj_gen/spline.hpp"

namespace traj_gen
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

VectorStateConstraint scalarWaypoint(double time, double position)
{
  VectorStateConstraint constraint;
  constraint.time = time;
  constraint.position = Eigen::VectorXd::Constant(1, position);
  return constraint;
}

AngularStateConstraint orientationWaypoint(double time, const Eigen::Quaterniond & orientation)
{
  AngularStateConstraint constraint;
  constraint.time = time;
  constraint.orientation = orientation;
  return constraint;
}

}  // namespace

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
  Eigen::Vector3d start_rpy(0.0, kPi / 2.0, kPi);
  start_constraint.orientation = Eigen::AngleAxisd(start_rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(start_rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(start_rpy.x(), Eigen::Vector3d::UnitX());
  start_constraint.angular_velocity = Eigen::Vector3d::Zero();

  AngularStateConstraint end_constraint;
  end_constraint.time = end_time;
  Eigen::Vector3d end_rpy(0.0, 0.0, kPi);
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

TEST(TrajectoryGenerator, OrientationSplineIsInvariantToAbsoluteTimeOffset)
{
  constexpr double base_time = 1073741824.0;  // 2^30
  std::vector<AngularStateConstraint> local;
  std::vector<AngularStateConstraint> shifted;
  for (int i = 0; i <= 2; ++i) {
    const Eigen::Quaterniond orientation(
      Eigen::AngleAxisd(i * kPi / 4.0, Eigen::Vector3d::UnitZ()));
    local.push_back(orientationWaypoint(i, orientation));
    shifted.push_back(orientationWaypoint(base_time + i, orientation));
  }

  OrientationSpline local_spline(local);
  OrientationSpline shifted_spline(shifted);
  for (double offset : {0.0, 0.25, 1.0, 1.75, 2.0}) {
    const Eigen::Quaterniond expected(
      Eigen::AngleAxisd(offset * kPi / 4.0, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond local_orientation = local_spline.getOrientation(offset);
    const Eigen::Quaterniond shifted_orientation =
      shifted_spline.getOrientation(base_time + offset);
    EXPECT_NEAR(std::abs(expected.dot(local_orientation)), 1.0, 1e-12);
    EXPECT_NEAR(std::abs(local_orientation.dot(shifted_orientation)), 1.0, 1e-12);
    EXPECT_TRUE(local_spline.getAngularVelocity(offset).isApprox(
      Eigen::Vector3d(0.0, 0.0, kPi / 4.0), 1e-12));
    EXPECT_TRUE(local_spline.getAngularVelocity(offset).isApprox(
      shifted_spline.getAngularVelocity(base_time + offset), 1e-12));
  }

  AngularStateConstraint single = orientationWaypoint(base_time, Eigen::Quaterniond::Identity());
  single.angular_velocity = Eigen::Vector3d(1.0, 2.0, 3.0);
  OrientationSpline single_point({single});
  EXPECT_TRUE(
    single_point.getOrientation(base_time - 1.0).isApprox(Eigen::Quaterniond::Identity()));
  EXPECT_TRUE(
    single_point.getOrientation(base_time + 1.0).isApprox(Eigen::Quaterniond::Identity()));
  EXPECT_EQ(single_point.getAngularVelocity(base_time), *single.angular_velocity);
  EXPECT_TRUE(single_point.getAngularVelocity(base_time - 1.0).isZero());
  EXPECT_TRUE(single_point.getAngularVelocity(base_time + 1.0).isZero());
}

TEST(TrajectoryGenerator, VectorSplineRejectsInvalidWaypoints)
{
  const VectorStateConstraint valid = scalarWaypoint(0.0, 0.0);
  EXPECT_THROW((VectorSpline({}, 1)), std::invalid_argument);
  EXPECT_THROW((VectorSpline({valid}, 0)), std::invalid_argument);

  VectorStateConstraint missing_position;
  missing_position.time = 0.0;
  EXPECT_THROW((VectorSpline({missing_position}, 1)), std::invalid_argument);

  VectorStateConstraint missing_velocity = valid;
  missing_velocity.acceleration = Eigen::VectorXd::Zero(1);
  EXPECT_THROW((VectorSpline({missing_velocity}, 1)), std::invalid_argument);

  VectorStateConstraint wrong_size = valid;
  wrong_size.position = Eigen::VectorXd::Zero(2);
  EXPECT_THROW((VectorSpline({wrong_size}, 1)), std::invalid_argument);

  VectorStateConstraint non_finite_time = valid;
  non_finite_time.time = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((VectorSpline({non_finite_time}, 1)), std::invalid_argument);

  VectorStateConstraint non_finite_value = valid;
  (*non_finite_value.position)(0) = std::numeric_limits<double>::infinity();
  EXPECT_THROW((VectorSpline({non_finite_value}, 1)), std::invalid_argument);

  VectorStateConstraint duplicate = valid;
  duplicate.time = -0.0;
  EXPECT_THROW((VectorSpline({valid, duplicate}, 1)), std::invalid_argument);
}

TEST(TrajectoryGenerator, VectorSplineUsesLocalTimeAndRejectsUnsafeDurations)
{
  const VectorStateConstraint start = scalarWaypoint(0.0, 0.0);
  const VectorStateConstraint indistinguishable =
    scalarWaypoint(std::numeric_limits<double>::denorm_min(), 1.0);
  EXPECT_THROW((VectorSpline({start, indistinguishable}, 1)), std::runtime_error);

  VectorStateConstraint absolute_start = scalarWaypoint(1e9, 0.0);
  absolute_start.velocity = Eigen::VectorXd::Zero(1);
  VectorStateConstraint absolute_end = scalarWaypoint(1e9 + 1.0, 1.0);
  absolute_end.velocity = Eigen::VectorXd::Zero(1);
  VectorSpline absolute_spline({absolute_start, absolute_end}, 1);
  EXPECT_NEAR(absolute_spline.getPosition(1e9)(0), 0.0, 1e-12);
  EXPECT_NEAR(absolute_spline.getPosition(1e9 + 0.5)(0), 0.5, 1e-12);
  EXPECT_NEAR(absolute_spline.getVelocity(1e9 + 0.5)(0), 1.5, 1e-12);
  EXPECT_NEAR(absolute_spline.getPosition(1e9 + 1.0)(0), 1.0, 1e-12);

  for (const double duration : {std::ldexp(1.0, -30), std::ldexp(1.0, 30)}) {
    VectorSpline scaled(
      {scalarWaypoint(0.0, 1.0), scalarWaypoint(duration, 1.0 + 2.0 * duration)}, 1);
    EXPECT_NEAR(scaled.getPosition(duration / 2.0)(0), 1.0 + duration, 1e-10);
    EXPECT_NEAR(scaled.getVelocity(duration / 2.0)(0), 2.0, 1e-12);
  }

  const VectorStateConstraint earliest = scalarWaypoint(-std::numeric_limits<double>::max(), 0.0);
  const VectorStateConstraint latest = scalarWaypoint(std::numeric_limits<double>::max(), 1.0);
  EXPECT_THROW((VectorSpline({earliest, latest}, 1)), std::runtime_error);
}

TEST(TrajectoryGenerator, VectorSplineSupportsEveryEndpointConstraintCombination)
{
  constexpr double start_time = 4.0;
  constexpr double duration = 2.0;
  for (int start_order = 0; start_order <= 2; ++start_order) {
    for (int end_order = 0; end_order <= 2; ++end_order) {
      SCOPED_TRACE(
        "start order " + std::to_string(start_order) + ", end order " + std::to_string(end_order));
      VectorStateConstraint start = scalarWaypoint(start_time, 1.0);
      VectorStateConstraint end = scalarWaypoint(start_time + duration, 3.0);
      if (start_order >= 1) {
        start.velocity = Eigen::VectorXd::Constant(1, 0.25);
      }
      if (end_order >= 1) {
        end.velocity = Eigen::VectorXd::Constant(1, -0.5);
      }
      if (start_order >= 2) {
        start.acceleration = Eigen::VectorXd::Constant(1, 0.75);
      }
      if (end_order >= 2) {
        end.acceleration = Eigen::VectorXd::Constant(1, -1.0);
      }

      VectorSpline spline({end, start}, 1);
      EXPECT_NEAR(spline.getPosition(start.time)(0), 1.0, 1e-10);
      EXPECT_NEAR(spline.getPosition(end.time)(0), 3.0, 1e-10);
      if (start.velocity) {
        EXPECT_NEAR(spline.getVelocity(start.time)(0), 0.25, 1e-10);
      }
      if (end.velocity) {
        EXPECT_NEAR(spline.getVelocity(end.time)(0), -0.5, 1e-10);
      }
      constexpr double delta = 1e-6;
      if (start.acceleration) {
        const double actual =
          (spline.getVelocity(start.time + delta)(0) - spline.getVelocity(start.time)(0)) / delta;
        EXPECT_NEAR(actual, 0.75, 1e-4);
      }
      if (end.acceleration) {
        const double actual =
          (spline.getVelocity(end.time)(0) - spline.getVelocity(end.time - delta)(0)) / delta;
        EXPECT_NEAR(actual, -1.0, 1e-4);
      }
    }
  }

  VectorStateConstraint p = scalarWaypoint(0.0, 0.0);
  VectorStateConstraint pv = scalarWaypoint(1.0, 1.0);
  pv.velocity = Eigen::VectorXd::Constant(1, 0.5);
  VectorStateConstraint pva = scalarWaypoint(3.0, 2.0);
  pva.velocity = Eigen::VectorXd::Constant(1, -0.25);
  pva.acceleration = Eigen::VectorXd::Constant(1, 0.2);
  VectorStateConstraint final_p = scalarWaypoint(4.0, 3.0);
  VectorSpline mixed_chain({pva, p, final_p, pv}, 1);
  EXPECT_NEAR(mixed_chain.getPosition(pv.time)(0), 1.0, 1e-10);
  EXPECT_NEAR(mixed_chain.getVelocity(pv.time)(0), 0.5, 1e-10);
  EXPECT_NEAR(mixed_chain.getPosition(pva.time)(0), 2.0, 1e-10);
  EXPECT_NEAR(mixed_chain.getVelocity(pva.time)(0), -0.25, 1e-10);
  EXPECT_TRUE(std::isfinite(mixed_chain.getPosition(0.5)(0)));
  EXPECT_TRUE(std::isfinite(mixed_chain.getPosition(2.0)(0)));
  EXPECT_TRUE(std::isfinite(mixed_chain.getPosition(3.5)(0)));

  VectorSpline linear({scalarWaypoint(2.0, 5.0), scalarWaypoint(0.0, 1.0)}, 1);
  EXPECT_NEAR(linear.getPosition(1.0)(0), 3.0, 1e-12);
  EXPECT_NEAR(linear.getVelocity(1.0)(0), 2.0, 1e-12);

  VectorStateConstraint smooth_start = scalarWaypoint(0.0, 0.0);
  smooth_start.velocity = Eigen::VectorXd::Zero(1);
  VectorStateConstraint smooth_end = scalarWaypoint(2.0, 1.0);
  smooth_end.velocity = Eigen::VectorXd::Zero(1);
  VectorSpline cubic({smooth_start, smooth_end}, 1);
  EXPECT_NEAR(cubic.getPosition(1.0)(0), 0.5, 1e-12);
  EXPECT_NEAR(cubic.getVelocity(1.0)(0), 0.75, 1e-12);

  VectorStateConstraint curved_start = scalarWaypoint(10.0, 0.0);
  curved_start.velocity = Eigen::VectorXd::Zero(1);
  curved_start.acceleration = Eigen::VectorXd::Constant(1, 0.5);
  VectorStateConstraint curved_end = scalarWaypoint(12.0, 0.0);
  curved_end.velocity = Eigen::VectorXd::Zero(1);
  curved_end.acceleration = Eigen::VectorXd::Constant(1, 0.5);
  VectorSpline quintic({curved_start, curved_end}, 1);
  EXPECT_NEAR(quintic.getPosition(10.5)(0), 0.03515625, 1e-12);
  EXPECT_NEAR(quintic.getVelocity(10.5)(0), 0.09375, 1e-12);
  EXPECT_NEAR(quintic.getPosition(11.0)(0), 0.0625, 1e-12);
  EXPECT_NEAR(quintic.getVelocity(11.0)(0), 0.0, 1e-12);
}

TEST(TrajectoryGenerator, VectorSplineIsPiecewiseAndScalesToManyWaypoints)
{
  std::vector<VectorStateConstraint> waypoints;
  constexpr int waypoint_count = 1000;
  constexpr double base_time = 1073741824.0;  // 2^30
  constexpr double step = 0.25;
  waypoints.reserve(waypoint_count);
  for (int i = waypoint_count - 1; i >= 0; --i) {
    const double offset = i * step;
    waypoints.push_back(scalarWaypoint(base_time + offset, 2.0 * offset + 1.0));
  }

  VectorSpline spline(waypoints, 1);
  for (int i = 0; i < waypoint_count; i += 137) {
    const double offset = i * step;
    EXPECT_NEAR(spline.getPosition(base_time + offset)(0), 2.0 * offset + 1.0, 1e-10);
  }
  EXPECT_NEAR(spline.getPosition(base_time + 100.125)(0), 201.25, 1e-10);
  EXPECT_NEAR(spline.getVelocity(base_time + 100.125)(0), 2.0, 1e-12);

  VectorSpline first(
    {scalarWaypoint(0.0, 0.0), scalarWaypoint(1.0, 1.0), scalarWaypoint(2.0, 2.0)}, 1);
  VectorSpline changed_far_segment(
    {scalarWaypoint(0.0, 0.0), scalarWaypoint(1.0, 1.0), scalarWaypoint(2.0, 1000.0)}, 1);
  EXPECT_DOUBLE_EQ(first.getPosition(0.5)(0), changed_far_segment.getPosition(0.5)(0));
  EXPECT_DOUBLE_EQ(first.getVelocity(0.5)(0), changed_far_segment.getVelocity(0.5)(0));
}

TEST(TrajectoryGenerator, VectorSplineDefinesKnotContinuityAndSinglePointBehavior)
{
  VectorSpline position_only(
    {scalarWaypoint(0.0, 0.0), scalarWaypoint(1.0, 1.0), scalarWaypoint(2.0, 3.0)}, 1);
  EXPECT_NEAR(position_only.getVelocity(1.0 - 1e-6)(0), 1.0, 1e-12);
  EXPECT_NEAR(position_only.getVelocity(1.0)(0), 2.0, 1e-12);
  EXPECT_NEAR(position_only.getVelocity(1.0 + 1e-6)(0), 2.0, 1e-12);

  VectorStateConstraint left = scalarWaypoint(0.0, 0.0);
  left.velocity = Eigen::VectorXd::Zero(1);
  left.acceleration = Eigen::VectorXd::Zero(1);
  VectorStateConstraint knot = scalarWaypoint(1.0, 0.0);
  knot.velocity = Eigen::VectorXd::Zero(1);
  knot.acceleration = Eigen::VectorXd::Constant(1, 2.0);
  VectorStateConstraint right = scalarWaypoint(2.0, 0.0);
  right.velocity = Eigen::VectorXd::Zero(1);
  right.acceleration = Eigen::VectorXd::Zero(1);
  VectorSpline twice_continuous({right, left, knot}, 1);
  constexpr double epsilon = 1e-5;
  const double acceleration_from_left =
    (twice_continuous.getVelocity(1.0)(0) - twice_continuous.getVelocity(1.0 - epsilon)(0)) /
    epsilon;
  const double acceleration_from_right =
    (twice_continuous.getVelocity(1.0 + epsilon)(0) - twice_continuous.getVelocity(1.0)(0)) /
    epsilon;
  EXPECT_NEAR(acceleration_from_left, 2.0, 2e-4);
  EXPECT_NEAR(acceleration_from_right, 2.0, 2e-4);

  VectorStateConstraint single = scalarWaypoint(5.0, 7.0);
  single.velocity = Eigen::VectorXd::Constant(1, 3.0);
  single.acceleration = Eigen::VectorXd::Constant(1, -2.0);
  VectorSpline single_point({single}, 1);
  EXPECT_DOUBLE_EQ(single_point.getPosition(-100.0)(0), 7.0);
  EXPECT_DOUBLE_EQ(single_point.getPosition(5.0)(0), 7.0);
  EXPECT_DOUBLE_EQ(single_point.getPosition(100.0)(0), 7.0);
  EXPECT_DOUBLE_EQ(single_point.getVelocity(5.0)(0), 3.0);
  EXPECT_DOUBLE_EQ(single_point.getVelocity(5.0 - 1e-12)(0), 0.0);
  EXPECT_DOUBLE_EQ(single_point.getVelocity(5.0 + 1e-12)(0), 0.0);
}

TEST(TrajectoryGenerator, VectorSplineHoldsEndpointsAndRejectsInvalidQueries)
{
  VectorStateConstraint start = scalarWaypoint(0.0, 1.0);
  start.velocity = Eigen::VectorXd::Constant(1, 2.0);
  VectorStateConstraint end = scalarWaypoint(1.0, 3.0);
  end.velocity = Eigen::VectorXd::Constant(1, -2.0);
  VectorSpline spline({end, start}, 1);

  EXPECT_DOUBLE_EQ(spline.getPosition(-1.0)(0), 1.0);
  EXPECT_DOUBLE_EQ(spline.getPosition(2.0)(0), 3.0);
  EXPECT_DOUBLE_EQ(spline.getVelocity(-1.0)(0), 0.0);
  EXPECT_DOUBLE_EQ(spline.getVelocity(2.0)(0), 0.0);
  EXPECT_NEAR(spline.getVelocity(0.0)(0), 2.0, 1e-12);
  EXPECT_NEAR(spline.getVelocity(1.0)(0), -2.0, 1e-12);

  EXPECT_THROW(spline.getPosition(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(spline.getVelocity(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(TrajectoryGenerator, OrientationSplineRejectsInvalidWaypoints)
{
  const AngularStateConstraint valid = orientationWaypoint(0.0, Eigen::Quaterniond::Identity());
  EXPECT_THROW((OrientationSpline({})), std::invalid_argument);

  AngularStateConstraint missing_orientation;
  missing_orientation.time = 0.0;
  EXPECT_THROW((OrientationSpline({missing_orientation})), std::invalid_argument);

  AngularStateConstraint missing_velocity = valid;
  missing_velocity.angular_acceleration = Eigen::Vector3d::Zero();
  EXPECT_THROW((OrientationSpline({missing_velocity})), std::invalid_argument);

  AngularStateConstraint zero_quaternion = orientationWaypoint(0.0, Eigen::Quaterniond(0, 0, 0, 0));
  EXPECT_THROW((OrientationSpline({zero_quaternion})), std::invalid_argument);

  AngularStateConstraint non_finite_quaternion = valid;
  non_finite_quaternion.orientation->coeffs()(0) = std::numeric_limits<double>::infinity();
  EXPECT_THROW((OrientationSpline({non_finite_quaternion})), std::invalid_argument);

  AngularStateConstraint non_finite_velocity = valid;
  non_finite_velocity.angular_velocity =
    Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  EXPECT_THROW((OrientationSpline({non_finite_velocity})), std::invalid_argument);

  AngularStateConstraint duplicate = valid;
  EXPECT_THROW((OrientationSpline({valid, duplicate})), std::invalid_argument);
}

TEST(TrajectoryGenerator, OrientationSplineNormalizesAndHoldsEndpoints)
{
  const Eigen::Quaterniond start_orientation(2.0, 0.0, 0.0, 0.0);
  const Eigen::Quaterniond end_orientation(
    2.0 * std::cos(kPi / 4.0), 0.0, 0.0, 2.0 * std::sin(kPi / 4.0));
  const AngularStateConstraint start = orientationWaypoint(0.0, start_orientation);
  const AngularStateConstraint end = orientationWaypoint(1.0, end_orientation);
  OrientationSpline spline({end, start});

  const Eigen::Quaterniond before = spline.getOrientation(-1.0);
  const Eigen::Quaterniond after = spline.getOrientation(2.0);
  EXPECT_DOUBLE_EQ(before.norm(), 1.0);
  EXPECT_DOUBLE_EQ(after.norm(), 1.0);
  EXPECT_NEAR(std::abs(before.dot(start_orientation.normalized())), 1.0, 1e-12);
  EXPECT_NEAR(std::abs(after.dot(end_orientation.normalized())), 1.0, 1e-12);
  EXPECT_TRUE(spline.getAngularVelocity(-1.0).isZero(0.0));
  EXPECT_TRUE(spline.getAngularVelocity(2.0).isZero(0.0));

  EXPECT_THROW(
    spline.getOrientation(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(
    spline.getAngularVelocity(-std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(TrajectoryGenerator, RotationMapsRejectInvalidInputs)
{
  EXPECT_THROW(
    expMap(Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0)),
    std::invalid_argument);
  EXPECT_THROW(logMap(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)), std::invalid_argument);

  Eigen::Quaterniond non_finite = Eigen::Quaterniond::Identity();
  non_finite.coeffs()(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(logMap(non_finite), std::invalid_argument);

  const Eigen::Quaterniond scaled(2.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(logMap(scaled).isZero(0.0));

  const Eigen::Quaterniond huge_scaled(
    std::numeric_limits<double>::max(), 0.0, 0.0, 0.0);
  EXPECT_TRUE(logMap(huge_scaled).isZero(0.0));
  OrientationSpline normalized_spline({orientationWaypoint(0.0, huge_scaled)});
  EXPECT_TRUE(normalized_spline.getOrientation(0.0).isApprox(Eigen::Quaterniond::Identity()));

  const Eigen::Quaterniond huge_rotation = expMap(Eigen::Vector3d(1e200, 1e200, 0.0));
  EXPECT_TRUE(huge_rotation.coeffs().allFinite());
  EXPECT_NEAR(huge_rotation.norm(), 1.0, 1e-12);
}

}  // namespace traj_gen

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
