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

#ifndef TRAJ_GEN__SPLINE_HPP_
#define TRAJ_GEN__SPLINE_HPP_

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

#include "traj_gen/base.hpp"
#include "traj_gen/common.hpp"
#include "traj_gen/visibility_control.h"

namespace traj_gen
{

/**
 * @brief Class for generating and managing spline trajectories of N-dimensional vectors (joint angles, positions, etc.).
 */
class TRAJ_GEN_PUBLIC VectorSpline : public VectorTrajectoryBase
{
public:
  /**
   * @brief Build a vector trajectory from finite, uniquely timed P/PV/PVA waypoints.
   *
   * Each adjacent pair is joined by the lowest-degree polynomial that satisfies the supplied
   * endpoint constraints. At internal waypoints, continuity is C0 at P, C1 at PV, and C2 at PVA.
   * @throws std::invalid_argument for malformed waypoints or a non-positive dof.
   * @throws std::runtime_error when a segment cannot be represented or solved accurately.
   */
  explicit VectorSpline(const std::vector<VectorStateConstraint> & constraints, int dof);
  ~VectorSpline() override = default;

  /**
   * @brief Get position, holding the nearest endpoint outside the trajectory interval.
   * @throws std::invalid_argument if time is not finite.
   */
  Eigen::VectorXd getPosition(double time) const override;

  /**
   * @brief Get velocity, returning zero outside the trajectory interval.
   *
   * At an internal P-only waypoint, the following segment defines the returned velocity.
   * @throws std::invalid_argument if time is not finite.
   */
  Eigen::VectorXd getVelocity(double time) const override;

  /**
   * @brief Get acceleration, returning zero outside the trajectory interval.
   *
   * At an internal waypoint without acceleration, the following segment defines the result.
   * @throws std::invalid_argument if time is not finite.
   */
  Eigen::VectorXd getAcceleration(double time) const override;

private:
  static std::vector<VectorStateConstraint> validateAndSortConstraints(
    const std::vector<VectorStateConstraint> & constraints, int dof);

  static Eigen::MatrixXd solveSegmentCoefficients(
    const VectorStateConstraint & start, const VectorStateConstraint & end, double duration,
    int dof);

  std::size_t segmentIndex(double time) const;
  Eigen::VectorXd evaluateSegment(std::size_t segment, double time, int derivative_order) const;

  const int kDof_;  // Degree of freedom for trajectory (number of order)
  std::vector<double> knot_times_;
  std::vector<Eigen::MatrixXd> segment_coefficients_;  // Coefficients in normalized local time.
  double start_time_;
  double end_time_;
  Eigen::VectorXd start_position_;
  Eigen::VectorXd end_position_;
  Eigen::VectorXd single_point_velocity_;
  Eigen::VectorXd single_point_acceleration_;
};

/**
 * @brief Class for generating and managing spline trajectories of orientation (quaternion).
 */
class TRAJ_GEN_PUBLIC OrientationSpline : public OrientationTrajectoryBase
{
public:
  /**
   * @brief Build an orientation trajectory from finite, uniquely timed Q/QV/QVA
   * waypoints.
   *
   * Input quaternions must be non-zero and are normalized internally. Each
   * adjacent pair uses the principal (shortest-arc) relative rotation and q(t)
   * = Exp(phi(t)) * q_i. Angular velocity and acceleration constraints use the
   * spatial/world frame, i.e. q_dot = 0.5 * [0, omega] * q.
   * @throws std::invalid_argument for malformed waypoints.
   * @throws std::runtime_error when a segment cannot be represented or solved
   * accurately.
   */
  explicit OrientationSpline(const std::vector<AngularStateConstraint> & constraints);
  ~OrientationSpline() override = default;

  /**
   * @brief Get orientation, holding the nearest endpoint outside the trajectory interval.
   * @throws std::invalid_argument if time is not finite.
   */
  Eigen::Quaterniond getOrientation(double time) const override;

  /**
   * @brief Get spatial/world-frame angular velocity, returning zero outside the interval.
   *
   * At an internal orientation-only waypoint, the following segment defines the result.
   * @throws std::invalid_argument if time is not finite.
   */
  Eigen::Vector3d getAngularVelocity(double time) const override;

  /**
   * @brief Get spatial/world-frame angular acceleration, returning zero outside the interval.
   *
   * At an internal waypoint without angular acceleration, the following segment defines the
   * result.
   * @throws std::invalid_argument if time is not finite.
   */
  Eigen::Vector3d getAngularAcceleration(double time) const override;

private:
  static std::vector<AngularStateConstraint> validateAndSortConstraints(
    const std::vector<AngularStateConstraint> & constraints);

  static std::vector<VectorStateConstraint> buildVectorConstraints(
    const AngularStateConstraint & start, const AngularStateConstraint & end);

  std::size_t segmentIndex(double time) const;

  std::vector<double> knot_times_;
  std::vector<Eigen::Quaterniond> knot_orientations_;
  std::vector<VectorSpline> segment_splines_;
  Eigen::Quaterniond start_orientation_;  // Reference starting orientation
  Eigen::Quaterniond end_orientation_;
  double start_time_;
  double end_time_;
  Eigen::Vector3d single_point_angular_velocity_;
  Eigen::Vector3d single_point_angular_acceleration_;
};

}  // namespace traj_gen

#endif  // TRAJ_GEN__SPLINE_HPP_
