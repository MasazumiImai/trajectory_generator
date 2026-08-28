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

double rotationVectorNorm(const Eigen::Vector3d & rotation_vector)
{
  if (!rotation_vector.allFinite()) {
    throw std::runtime_error("Rotation-vector interpolation produced a non-finite value.");
  }
  const double theta = rotation_vector.stableNorm();
  if (!std::isfinite(theta)) {
    throw std::runtime_error("Rotation-vector norm cannot be represented.");
  }
  return theta;
}

Eigen::Vector3d applyLeftJacobian(
  const Eigen::Vector3d & rotation_vector, const Eigen::Vector3d & value)
{
  constexpr double kSmallAngle = 1e-2;
  const double theta = rotationVectorNorm(rotation_vector);
  Eigen::Vector3d result;
  if (theta < kSmallAngle) {
    const double squared = theta * theta;
    const double fourth = squared * squared;
    const double a = 0.5 - squared / 24.0 + fourth / 720.0;
    const double b = 1.0 / 6.0 - squared / 120.0 + fourth / 5040.0;
    result = value + a * rotation_vector.cross(value) +
      b * rotation_vector.cross(rotation_vector.cross(value));
  } else {
    const Eigen::Vector3d axis = rotation_vector / theta;
    result = value + ((1.0 - std::cos(theta)) / theta) * axis.cross(value) +
      (1.0 - std::sin(theta) / theta) * axis.cross(axis.cross(value));
  }
  if (!result.allFinite()) {
    throw std::runtime_error("SO(3) left-Jacobian mapping produced a non-finite value.");
  }
  return result;
}

Eigen::Vector3d applyInverseLeftJacobian(
  const Eigen::Vector3d & rotation_vector, const Eigen::Vector3d & value)
{
  constexpr double kSmallAngle = 1e-2;
  const double theta = rotationVectorNorm(rotation_vector);
  Eigen::Vector3d result;
  if (theta < kSmallAngle) {
    const double squared = theta * theta;
    const double fourth = squared * squared;
    const double c = 1.0 / 12.0 + squared / 720.0 + fourth / 30240.0;
    result = value - 0.5 * rotation_vector.cross(value) +
      c * rotation_vector.cross(rotation_vector.cross(value));
  } else {
    const double half_theta = 0.5 * theta;
    const double sin_half_theta = std::sin(half_theta);
    if (sin_half_theta == 0.0) {
      throw std::runtime_error("SO(3) left Jacobian is singular for this rotation vector.");
    }
    const Eigen::Vector3d axis = rotation_vector / theta;
    const double quadratic_coefficient = 1.0 - half_theta * std::cos(half_theta) / sin_half_theta;
    result = value - half_theta * axis.cross(value) +
      quadratic_coefficient * axis.cross(axis.cross(value));
  }
  if (!result.allFinite()) {
    throw std::runtime_error("SO(3) inverse left-Jacobian mapping produced a non-finite value.");
  }
  return result;
}

Eigen::Vector3d leftJacobianDerivativeTimes(
  const Eigen::Vector3d & rotation_vector, const Eigen::Vector3d & rotation_vector_velocity)
{
  constexpr double kSmallAngle = 1e-2;
  const double theta = rotationVectorNorm(rotation_vector);
  const double projection = rotation_vector.dot(rotation_vector_velocity);
  if (!std::isfinite(projection)) {
    throw std::runtime_error("SO(3) Jacobian derivative cannot be represented.");
  }

  double b;
  double a_dot;
  double b_dot;
  if (theta < kSmallAngle) {
    const double squared = theta * theta;
    const double fourth = squared * squared;
    b = 1.0 / 6.0 - squared / 120.0 + fourth / 5040.0;
    a_dot = (-1.0 / 12.0 + squared / 180.0 - fourth / 6720.0) * projection;
    b_dot = (-1.0 / 60.0 + squared / 1260.0 - fourth / 60480.0) * projection;
  } else {
    const double squared = theta * theta;
    const double cubed = squared * theta;
    const double fourth = squared * squared;
    const double fifth = fourth * theta;
    const double sin_theta = std::sin(theta);
    const double one_minus_cos = 1.0 - std::cos(theta);
    b = (theta - sin_theta) / cubed;
    a_dot = (theta * sin_theta - 2.0 * one_minus_cos) * projection / fourth;
    b_dot = (theta * one_minus_cos - 3.0 * (theta - sin_theta)) * projection / fifth;
  }

  const Eigen::Vector3d first_cross = rotation_vector.cross(rotation_vector_velocity);
  const Eigen::Vector3d result = a_dot * first_cross + b_dot * rotation_vector.cross(first_cross) +
    b * rotation_vector_velocity.cross(first_cross);
  if (!result.allFinite()) {
    throw std::runtime_error("SO(3) Jacobian derivative produced a non-finite value.");
  }
  return result;
}

VectorStateConstraint toRotationVectorConstraint(
  const AngularStateConstraint & constraint, const Eigen::Vector3d & rotation_vector)
{
  VectorStateConstraint vector_constraint;
  vector_constraint.time = constraint.time;
  vector_constraint.position = rotation_vector;
  if (!constraint.angular_velocity) {
    return vector_constraint;
  }

  const Eigen::Vector3d rotation_vector_velocity =
    applyInverseLeftJacobian(rotation_vector, *constraint.angular_velocity);
  vector_constraint.velocity = rotation_vector_velocity;
  if (constraint.angular_acceleration) {
    const Eigen::Vector3d jacobian_derivative =
      leftJacobianDerivativeTimes(rotation_vector, rotation_vector_velocity);
    const Eigen::Vector3d remaining_acceleration =
      *constraint.angular_acceleration - jacobian_derivative;
    if (!remaining_acceleration.allFinite()) {
      throw std::runtime_error("Angular-acceleration mapping produced a non-finite value.");
    }
    vector_constraint.acceleration =
      applyInverseLeftJacobian(rotation_vector, remaining_acceleration);
  }
  return vector_constraint;
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
{
  const auto sorted_constraints = validateAndSortConstraints(constraints);
  start_time_ = sorted_constraints.front().time;
  end_time_ = sorted_constraints.back().time;
  start_orientation_ = *sorted_constraints.front().orientation;
  end_orientation_ = *sorted_constraints.back().orientation;
  single_point_angular_velocity_ =
    sorted_constraints.front().angular_velocity.value_or(Eigen::Vector3d::Zero());

  knot_times_.reserve(sorted_constraints.size());
  knot_orientations_.reserve(sorted_constraints.size());
  for (const auto & constraint : sorted_constraints) {
    knot_times_.push_back(constraint.time);
    knot_orientations_.push_back(*constraint.orientation);
  }

  segment_splines_.reserve(sorted_constraints.size() - 1);
  for (std::size_t i = 0; i + 1 < sorted_constraints.size(); ++i) {
    segment_splines_.emplace_back(
      buildVectorConstraints(sorted_constraints[i], sorted_constraints[i + 1]), 3);
  }
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
  if (segment_splines_.empty()) {
    return start_orientation_;
  }

  const std::size_t segment = segmentIndex(time);
  if (time == knot_times_[segment]) {
    return knot_orientations_[segment];
  }
  if (time == knot_times_[segment + 1]) {
    return knot_orientations_[segment + 1];
  }
  const Eigen::Vector3d rotation_vector = segment_splines_[segment].getPosition(time);
  return normalizedQuaternion(expMap(rotation_vector) * knot_orientations_[segment]);
}

Eigen::Vector3d OrientationSpline::getAngularVelocity(double time)
{
  validateQueryTime(time);
  if (time < start_time_ || time > end_time_) {
    return Eigen::Vector3d::Zero();
  }
  if (segment_splines_.empty()) {
    return single_point_angular_velocity_;
  }

  const std::size_t segment = segmentIndex(time);
  const Eigen::Vector3d rotation_vector = segment_splines_[segment].getPosition(time);
  const Eigen::Vector3d rotation_vector_velocity = segment_splines_[segment].getVelocity(time);
  return applyLeftJacobian(rotation_vector, rotation_vector_velocity);
}

std::vector<AngularStateConstraint> OrientationSpline::validateAndSortConstraints(
  const std::vector<AngularStateConstraint> & constraints)
{
  if (constraints.empty()) {
    throw std::invalid_argument("At least one orientation waypoint is required.");
  }

  std::vector<AngularStateConstraint> sorted_constraints;
  sorted_constraints.reserve(constraints.size());
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
    if (constraint.angular_velocity && !constraint.angular_velocity->allFinite()) {
      throw std::invalid_argument("Angular velocity must contain only finite values.");
    }
    if (constraint.angular_acceleration && !constraint.angular_acceleration->allFinite()) {
      throw std::invalid_argument("Angular acceleration must contain only finite values.");
    }

    AngularStateConstraint normalized = constraint;
    normalized.orientation = normalizedQuaternion(*constraint.orientation);
    sorted_constraints.push_back(normalized);
  }

  std::sort(
    sorted_constraints.begin(), sorted_constraints.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.time < rhs.time; });
  for (std::size_t i = 1; i < sorted_constraints.size(); ++i) {
    if (sorted_constraints[i - 1].time == sorted_constraints[i].time) {
      throw std::invalid_argument("Orientation waypoint times must be unique.");
    }

    const Eigen::Quaterniond & previous = *sorted_constraints[i - 1].orientation;
    Eigen::Quaterniond & current = *sorted_constraints[i].orientation;
    const double dot = previous.dot(current);
    bool flip = dot < 0.0;
    if (dot == 0.0) {
      const Eigen::Quaterniond relative = current * previous.conjugate();
      Eigen::Index dominant_axis;
      relative.vec().cwiseAbs().maxCoeff(&dominant_axis);
      flip = relative.vec()(dominant_axis) < 0.0;
    }
    if (flip) {
      sorted_constraints[i].orientation->coeffs() *= -1.0;
    }
  }
  return sorted_constraints;
}

std::vector<VectorStateConstraint> OrientationSpline::buildVectorConstraints(
  const AngularStateConstraint & start, const AngularStateConstraint & end)
{
  const Eigen::Quaterniond relative_orientation = *end.orientation * start.orientation->conjugate();
  const Eigen::Vector3d end_rotation_vector = logMap(relative_orientation);
  return {
    toRotationVectorConstraint(start, Eigen::Vector3d::Zero()),
    toRotationVectorConstraint(end, end_rotation_vector)};
}

std::size_t OrientationSpline::segmentIndex(double time) const
{
  const auto upper = std::upper_bound(knot_times_.begin(), knot_times_.end(), time);
  const std::size_t knot = static_cast<std::size_t>(upper - knot_times_.begin());
  return std::min(knot - 1, segment_splines_.size() - 1);
}

}  // namespace traj_gen
