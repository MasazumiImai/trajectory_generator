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

#include "traj_gen/spline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traj_gen
{
namespace
{

void validateQueryTime(double time)
{
  if (!std::isfinite(time)) {
    throw std::invalid_argument("Query time must be finite.");
  }
}

Eigen::Quaterniond normalizedQuaternion(const Eigen::Quaterniond & quaternion)
{
  if (!quaternion.coeffs().allFinite()) {
    throw std::invalid_argument("Quaternion must contain only finite values.");
  }
  const double scale = quaternion.coeffs().cwiseAbs().maxCoeff();
  if (scale == 0.0) {
    throw std::invalid_argument("Quaternion must be non-zero.");
  }

  const Eigen::Vector4d scaled = quaternion.coeffs() / scale;
  return Eigen::Quaterniond(scaled / scaled.norm());
}

int derivativeCount(const VectorStateConstraint & constraint)
{
  if (constraint.acceleration) {
    return 3;
  }
  return constraint.velocity ? 2 : 1;
}

const Eigen::VectorXd & derivativeValue(
  const VectorStateConstraint & constraint, int derivative_order)
{
  if (derivative_order == 0) {
    return *constraint.position;
  }
  if (derivative_order == 1) {
    return *constraint.velocity;
  }
  return *constraint.acceleration;
}

double derivativeFactor(int power, int derivative_order)
{
  double factor = 1.0;
  for (int i = 0; i < derivative_order; ++i) {
    factor *= power - i;
  }
  return factor;
}

Eigen::VectorXd scaleTimeDerivative(
  const Eigen::VectorXd & value, double duration, int derivative_order)
{
  Eigen::VectorXd scaled = value;
  for (int order = 0; order < derivative_order; ++order) {
    const Eigen::VectorXd previous = scaled;
    scaled *= duration;
    if (!scaled.allFinite()) {
      throw std::runtime_error("Segment derivative scaling produced a non-finite value.");
    }
    for (Eigen::Index i = 0; i < scaled.size(); ++i) {
      if (previous(i) != 0.0 && scaled(i) == 0.0) {
        throw std::runtime_error("Segment derivative scaling underflowed.");
      }
    }
  }
  return scaled;
}

Eigen::VectorXd evaluateNormalizedPolynomial(
  const Eigen::MatrixXd & coefficients, double u, int derivative_order)
{
  Eigen::VectorXd value = Eigen::VectorXd::Zero(coefficients.cols());
  for (int power = coefficients.rows() - 1; power >= derivative_order; --power) {
    value =
      value * u + derivativeFactor(power, derivative_order) * coefficients.row(power).transpose();
  }
  return value;
}

Eigen::VectorXd toPhysicalDerivative(Eigen::VectorXd value, double duration, int derivative_order)
{
  for (int order = 0; order < derivative_order; ++order) {
    value /= duration;
  }
  if (!value.allFinite()) {
    throw std::runtime_error("Segment derivative cannot be represented in physical time.");
  }
  return value;
}

}  // namespace

VectorSpline::VectorSpline(const std::vector<VectorStateConstraint> & constraints, int dof)
: kDof_(dof)
{
  const auto sorted_constraints = validateAndSortConstraints(constraints, kDof_);
  start_time_ = sorted_constraints.front().time;
  end_time_ = sorted_constraints.back().time;
  start_position_ = *sorted_constraints.front().position;
  end_position_ = *sorted_constraints.back().position;
  single_point_velocity_ =
    sorted_constraints.front().velocity.value_or(Eigen::VectorXd::Zero(kDof_));

  knot_times_.reserve(sorted_constraints.size());
  for (const auto & constraint : sorted_constraints) {
    knot_times_.push_back(constraint.time);
  }

  segment_coefficients_.reserve(sorted_constraints.size() - 1);
  for (std::size_t i = 0; i + 1 < sorted_constraints.size(); ++i) {
    const double duration = sorted_constraints[i + 1].time - sorted_constraints[i].time;
    if (!std::isfinite(duration) || duration <= 0.0 || !std::isfinite(1.0 / duration)) {
      throw std::runtime_error("Segment duration cannot be represented safely.");
    }
    segment_coefficients_.push_back(
      solveSegmentCoefficients(sorted_constraints[i], sorted_constraints[i + 1], duration, kDof_));
  }
}

Eigen::VectorXd VectorSpline::getPosition(double time)
{
  validateQueryTime(time);
  if (time < start_time_) {
    return start_position_;
  }
  if (time > end_time_) {
    return end_position_;
  }
  if (segment_coefficients_.empty()) {
    return start_position_;
  }
  return evaluateSegment(segmentIndex(time), time, 0);
}

Eigen::VectorXd VectorSpline::getVelocity(double time)
{
  validateQueryTime(time);
  if (time < start_time_ || time > end_time_) {
    return Eigen::VectorXd::Zero(kDof_);
  }
  if (segment_coefficients_.empty()) {
    return single_point_velocity_;
  }
  return evaluateSegment(segmentIndex(time), time, 1);
}

std::vector<VectorStateConstraint> VectorSpline::validateAndSortConstraints(
  const std::vector<VectorStateConstraint> & constraints, int dof)
{
  if (dof <= 0) {
    throw std::invalid_argument("Degrees of freedom must be positive.");
  }
  if (constraints.empty()) {
    throw std::invalid_argument("At least one waypoint is required.");
  }

  const auto validate_value = [dof](const Eigen::VectorXd & value) {
    if (value.size() != static_cast<Eigen::Index>(dof)) {
      throw std::invalid_argument("Waypoint vector size must match degrees of freedom.");
    }
    if (!value.allFinite()) {
      throw std::invalid_argument("Waypoint vectors must contain only finite values.");
    }
  };

  std::vector<VectorStateConstraint> sorted_constraints;
  sorted_constraints.reserve(constraints.size());
  for (const auto & constraint : constraints) {
    if (!std::isfinite(constraint.time)) {
      throw std::invalid_argument("Waypoint time must be finite.");
    }
    if (!constraint.position) {
      throw std::invalid_argument("Every waypoint must specify position.");
    }
    if (constraint.acceleration && !constraint.velocity) {
      throw std::invalid_argument("Acceleration requires velocity at the same waypoint.");
    }

    validate_value(*constraint.position);
    if (constraint.velocity) {
      validate_value(*constraint.velocity);
    }
    if (constraint.acceleration) {
      validate_value(*constraint.acceleration);
    }

    sorted_constraints.push_back(constraint);
  }

  std::sort(
    sorted_constraints.begin(), sorted_constraints.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.time < rhs.time; });
  for (std::size_t i = 1; i < sorted_constraints.size(); ++i) {
    if (sorted_constraints[i - 1].time == sorted_constraints[i].time) {
      throw std::invalid_argument("Waypoint times must be unique.");
    }
  }
  return sorted_constraints;
}

Eigen::MatrixXd VectorSpline::solveSegmentCoefficients(
  const VectorStateConstraint & start, const VectorStateConstraint & end, double duration, int dof)
{
  const int start_constraints = derivativeCount(start);
  const int end_constraints = derivativeCount(end);
  const int num_constraints = start_constraints + end_constraints;

  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(num_constraints, dof);
  for (int derivative_order = 0; derivative_order < start_constraints; ++derivative_order) {
    coefficients.row(derivative_order) =
      (scaleTimeDerivative(derivativeValue(start, derivative_order), duration, derivative_order) /
       derivativeFactor(derivative_order, derivative_order))
        .transpose();
  }

  Eigen::MatrixXd end_basis = Eigen::MatrixXd::Zero(end_constraints, end_constraints);
  Eigen::MatrixXd end_values = Eigen::MatrixXd::Zero(end_constraints, dof);
  for (int derivative_order = 0; derivative_order < end_constraints; ++derivative_order) {
    end_values.row(derivative_order) =
      scaleTimeDerivative(derivativeValue(end, derivative_order), duration, derivative_order)
        .transpose();
    for (int power = derivative_order; power < start_constraints; ++power) {
      end_values.row(derivative_order) -=
        derivativeFactor(power, derivative_order) * coefficients.row(power);
    }
    for (int column = 0; column < end_constraints; ++column) {
      const int power = start_constraints + column;
      if (power >= derivative_order) {
        end_basis(derivative_order, column) = derivativeFactor(power, derivative_order);
      }
    }
  }
  if (!end_values.allFinite()) {
    throw std::runtime_error("Segment endpoint reduction produced non-finite values.");
  }

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> decomposition(end_basis);
  if (decomposition.rank() != end_constraints) {
    throw std::runtime_error("Segment constraints are numerically rank deficient.");
  }

  const Eigen::MatrixXd remaining_coefficients = decomposition.solve(end_values);
  coefficients.bottomRows(end_constraints) = remaining_coefficients;
  if (!coefficients.allFinite()) {
    throw std::runtime_error("Segment coefficient solve produced non-finite values.");
  }

  const Eigen::MatrixXd residual = end_basis * remaining_coefficients - end_values;
  const Eigen::ArrayXXd relative_residual =
    residual.array().abs() / (1.0 + end_values.array().abs());
  constexpr double kResidualTolerance = 1e-10;
  if (!relative_residual.allFinite() || (relative_residual > kResidualTolerance).any()) {
    throw std::runtime_error("Segment constraints could not be satisfied accurately.");
  }

  const auto verify_endpoint = [&](const VectorStateConstraint & constraint, double u) {
    const int count = derivativeCount(constraint);
    for (int derivative_order = 0; derivative_order < count; ++derivative_order) {
      const Eigen::VectorXd actual = toPhysicalDerivative(
        evaluateNormalizedPolynomial(coefficients, u, derivative_order), duration,
        derivative_order);
      const Eigen::VectorXd & expected = derivativeValue(constraint, derivative_order);
      const Eigen::ArrayXd relative_error =
        (actual - expected).array().abs() / (1.0 + expected.array().abs());
      if (!relative_error.allFinite() || (relative_error > kResidualTolerance).any()) {
        throw std::runtime_error("Segment endpoint constraints could not be reproduced.");
      }
    }
  };
  verify_endpoint(start, 0.0);
  verify_endpoint(end, 1.0);

  // This coefficient-wise bound may reject cancellation-heavy but finite segments.
  // Replace it with interval range analysis if those inputs must be supported.
  for (int derivative_order = 0; derivative_order <= 2; ++derivative_order) {
    Eigen::VectorXd bound = Eigen::VectorXd::Zero(dof);
    for (int power = derivative_order; power < coefficients.rows(); ++power) {
      Eigen::VectorXd term =
        derivativeFactor(power, derivative_order) * coefficients.row(power).transpose();
      term = toPhysicalDerivative(term.cwiseAbs(), duration, derivative_order);
      bound += term;
      if (!bound.allFinite()) {
        throw std::runtime_error("Segment evaluation may overflow.");
      }
    }
  }
  return coefficients;
}

std::size_t VectorSpline::segmentIndex(double time) const
{
  const auto upper = std::upper_bound(knot_times_.begin(), knot_times_.end(), time);
  const std::size_t knot = static_cast<std::size_t>(upper - knot_times_.begin());
  return std::min(knot - 1, segment_coefficients_.size() - 1);
}

Eigen::VectorXd VectorSpline::evaluateSegment(
  std::size_t segment, double time, int derivative_order) const
{
  const double duration = knot_times_[segment + 1] - knot_times_[segment];
  const double u = (time - knot_times_[segment]) / duration;
  return toPhysicalDerivative(
    evaluateNormalizedPolynomial(segment_coefficients_[segment], u, derivative_order), duration,
    derivative_order);
}

OrientationSpline::OrientationSpline(const std::vector<AngularStateConstraint> & constraints)
: vector_spline_(buildVectorConstraints(constraints), 3)
{
  const auto [start, end] = std::minmax_element(
    constraints.begin(), constraints.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.time < rhs.time; });
  start_orientation_ = normalizedQuaternion(*start->orientation);
  end_orientation_ = normalizedQuaternion(*end->orientation);
  start_time_ = start->time;
  end_time_ = end->time;
}

Eigen::Quaterniond OrientationSpline::getOrientation(double time)
{
  validateQueryTime(time);
  if (time < start_time_) {
    return start_orientation_;
  }
  if (time > end_time_) {
    return end_orientation_;
  }

  Eigen::Vector3d delta_omega = vector_spline_.getPosition(time);
  Eigen::Quaterniond delta_q = expMap(delta_omega);
  return delta_q * start_orientation_;
}

Eigen::Vector3d OrientationSpline::getAngularVelocity(double time)
{
  validateQueryTime(time);
  if (time < start_time_ || time > end_time_) {
    return Eigen::Vector3d::Zero();
  }
  return vector_spline_.getVelocity(time);
}

std::vector<VectorStateConstraint> OrientationSpline::buildVectorConstraints(
  const std::vector<AngularStateConstraint> & constraints)
{
  if (constraints.empty()) {
    throw std::invalid_argument("At least one orientation waypoint is required.");
  }

  for (const auto & constraint : constraints) {
    if (!std::isfinite(constraint.time)) {
      throw std::invalid_argument("Orientation waypoint time must be finite.");
    }
    if (!constraint.orientation) {
      throw std::invalid_argument("Every orientation waypoint must specify orientation.");
    }
    if (constraint.angular_acceleration && !constraint.angular_velocity) {
      throw std::invalid_argument(
        "Angular acceleration requires angular "
        "velocity at the same waypoint.");
    }
    normalizedQuaternion(*constraint.orientation);
    if (constraint.angular_velocity && !constraint.angular_velocity->allFinite()) {
      throw std::invalid_argument("Angular velocity must contain only finite values.");
    }
    if (constraint.angular_acceleration && !constraint.angular_acceleration->allFinite()) {
      throw std::invalid_argument("Angular acceleration must contain only finite values.");
    }
  }

  const auto start = std::min_element(
    constraints.begin(), constraints.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.time < rhs.time; });
  const Eigen::Quaterniond start_orientation = normalizedQuaternion(*start->orientation);

  std::vector<VectorStateConstraint> vector_constraints;
  vector_constraints.reserve(constraints.size());
  for (const auto & ac : constraints) {
    VectorStateConstraint vc;
    vc.time = ac.time;
    // Calculate the difference quaternion and convert it to a rotation vector
    // using a logarithmic mapping.
    const Eigen::Quaterniond orientation = normalizedQuaternion(*ac.orientation);
    const Eigen::Quaterniond delta_q = orientation * start_orientation.conjugate();
    vc.position = logMap(delta_q);
    if (ac.angular_velocity) {
      vc.velocity = *ac.angular_velocity;
    }
    if (ac.angular_acceleration) {
      vc.acceleration = *ac.angular_acceleration;
    }
    vector_constraints.push_back(vc);
  }
  return vector_constraints;
}

}  // namespace traj_gen
