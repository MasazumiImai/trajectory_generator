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

#include "trajectory_generator/spline_trajectory.hpp"

#define DEBUG_ENABLED false

namespace trajectory_generator
{

SplineTrajectory::SplineTrajectory()
{
  std::cout << "SplineTrajectory class is established." << std::endl;
}

SplineTrajectory::~SplineTrajectory()
{
  std::cout << "SplineTrajectory class is destructed." << std::endl;
}

Eigen::MatrixXd SplineTrajectory::calculateCoefficients(
  const std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> & constraints_map)
{
  const Eigen::MatrixXd X = getSplineValueVector(constraints_map);

  const Eigen::MatrixXd T = getBasisMatrix(constraints_map);

  const Eigen::MatrixXd coefficients = T.colPivHouseholderQr().solve(X);  // A = T^-1 * X

  return coefficients;  // num_constraints x 3
}

Eigen::Vector3d SplineTrajectory::calculatePositionAtCertainTime(
  const double & time, const Eigen::MatrixXd & coefficients)
{
  const int num_coeff = coefficients.rows();
  const int dof = coefficients.cols();  // 3

  Eigen::Vector3d position;

  for (int i = 0; i < dof; ++i) {
    double position_temp = 0.0;

    for (int j = 0; j < num_coeff; ++j) {
      position_temp += coefficients(j, i) * pow(time, j);
    }

    position(i) = position_temp;
  }

  return position;
}

Eigen::Quaterniond SplineTrajectory::calculateOrientationAtCertainTime(
  const double & time, const Eigen::MatrixXd & coefficients)
{
  const int num_coeff = coefficients.rows();
  const int dof = coefficients.cols();  // 3

  Eigen::Vector3d orientation;

  for (int i = 0; i < dof; ++i) {
    double orientation_temp = 0.0;

    for (int j = 0; j < num_coeff; ++j) {
      orientation_temp += coefficients(j, i) * pow(time, j);
    }

    orientation(i) = orientation_temp;
  }

  Eigen::Quaterniond quaternion = TrajectoryGenerator::expMap(orientation);

  return quaternion;
}

Eigen::Matrix3Xd SplineTrajectory::calculateTrajectory(
  const std::vector<LinearStateConstraint> & constraints, const int & step_time)
{
  auto constraints_map = TrajectoryGenerator::getConstraintsMap(constraints);

  if (!TrajectoryGenerator::isStartFromZero(constraints_map.begin()->first)) {
    std::cerr << "ERROR: Start time always should be 0 ms" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  const int motion_duration_ms = TrajectoryGenerator::calculateMotionDuration(
    constraints_map.begin()->first, constraints_map.rbegin()->first);

  const Eigen::MatrixXd coefficients = calculateCoefficients(constraints_map);

  const int num_coeff = coefficients.rows();
  const int dof = coefficients.cols();  // 3

  Eigen::Matrix3Xd trajectory(dof, motion_duration_ms / step_time + 1);

  for (int time_ms = 0; time_ms <= motion_duration_ms; time_ms += step_time) {
    Eigen::Vector3d vector;
    double time = (double)time_ms / 1000.0;  // [s]

    for (int i = 0; i < dof; ++i) {
      double value = 0.0;

      for (int j = 0; j < num_coeff; ++j) {
        value += coefficients(j, i) * pow(time, j);
      }

      vector(i) = value;
    }

    trajectory.col(time_ms / step_time) = vector;
  }

  return trajectory;
}

Eigen::Matrix4Xd SplineTrajectory::calculateTrajectory(
  const std::vector<AngularStateConstraint> & constraints, const int & step_time)
{
  auto constraints_map = TrajectoryGenerator::getConstraintsMap(constraints);

  if (!TrajectoryGenerator::isStartFromZero(constraints_map.begin()->first)) {
    std::cerr << "ERROR: Start time always should be 0.0" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  const int motion_duration_ms = TrajectoryGenerator::calculateMotionDuration(
    constraints_map.begin()->first, constraints_map.rbegin()->first);

  const Eigen::MatrixXd coefficients = calculateCoefficients(constraints_map);

  const int num_coeff = coefficients.rows();
  const int dof = coefficients.cols();  // 3

  Eigen::Matrix4Xd trajectory(4, motion_duration_ms / step_time + 1);

  for (int time_ms = 0; time_ms <= motion_duration_ms; time_ms += step_time) {
    Eigen::Vector3d vector;
    double time = (double)time_ms / 1000.0;  // [s]

    for (int i = 0; i < dof; ++i) {
      double value = 0.0;

      for (int j = 0; j < num_coeff; ++j) {
        value += coefficients(j, i) * pow(time, j);
      }

      vector(i) = value;
    }

    Eigen::Quaterniond quaternion = TrajectoryGenerator::expMap(vector);

    trajectory.col(time_ms / step_time) =
      Eigen::Vector4d(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
  }

  return trajectory;
}

Eigen::Matrix3Xd SplineTrajectory::calculateVelocityTrajectory(
  const std::vector<LinearStateConstraint> & constraints, const int & step_time)
{
  auto constraints_map = TrajectoryGenerator::getConstraintsMap(constraints);

  if (!TrajectoryGenerator::isStartFromZero(constraints_map.begin()->first)) {
    std::cerr << "ERROR: Start time always should be 0.0" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  const int motion_duration_ms = TrajectoryGenerator::calculateMotionDuration(
    constraints_map.begin()->first, constraints_map.rbegin()->first);

  const Eigen::MatrixXd coefficients = calculateCoefficients(constraints_map);

  const int num_coeff = coefficients.rows();
  const int dof = coefficients.cols();  // 3

  Eigen::Matrix3Xd trajectory(dof, motion_duration_ms / step_time + 1);

  for (int time_ms = 0; time_ms <= motion_duration_ms; time_ms += step_time) {
    Eigen::Vector3d vector;
    double time = (double)time_ms / 1000.0;  // [s]

    for (int i = 0; i < dof; ++i) {
      double value = 0.0;

      for (int j = 0; j < num_coeff; ++j) {
        if (j - 1 < 0) {
          value += 0.0;
        } else {
          value += coefficients(j, i) * j * pow(time, j - 1);
        }
      }

      vector(i) = value;
    }

    trajectory.col(time_ms / step_time) = vector;
  }

  return trajectory;
}

Eigen::Matrix3Xd SplineTrajectory::calculateVelocityTrajectory(
  const std::vector<AngularStateConstraint> & constraints, const int & step_time)
{
  auto constraints_map = TrajectoryGenerator::getConstraintsMap(constraints);

  if (!TrajectoryGenerator::isStartFromZero(constraints_map.begin()->first)) {
    std::cerr << "ERROR: Start time always should be 0.0" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  const int motion_duration_ms = TrajectoryGenerator::calculateMotionDuration(
    constraints_map.begin()->first, constraints_map.rbegin()->first);

  const Eigen::MatrixXd coefficients = calculateCoefficients(constraints_map);

  const int num_coeff = coefficients.rows();
  const int dof = coefficients.cols();  // 3

  Eigen::Matrix3Xd trajectory(dof, motion_duration_ms / step_time + 1);

  for (int time_ms = 0; time_ms <= motion_duration_ms; time_ms += step_time) {
    Eigen::Vector3d vector;
    double time = (double)time_ms / 1000.0;  // [s]

    for (int i = 0; i < dof; ++i) {
      double value = 0.0;

      for (int j = 0; j < num_coeff; ++j) {
        if (j - 1 < 0) {
          value += 0.0;
        } else {
          value += coefficients(j, i) * j * pow(time, j - 1);
        }
      }

      vector(i) = value;
    }

    trajectory.col(time_ms / step_time) = vector;
  }

  return trajectory;
}

Eigen::MatrixXd SplineTrajectory::getSplineValueVector(
  const std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> & constraints_map)
{
  const int num_constraints = TrajectoryGenerator::getNumberOfConstraints(constraints_map);
  const int kDof = 3;
  Eigen::MatrixXd spline_value_matrix = Eigen::MatrixXd::Zero(num_constraints, kDof);

  int idx = 0;
  for (const auto & [_, state_map] : constraints_map) {
    for (const auto & [_, vector] : state_map) {
      spline_value_matrix.row(idx) = vector;
      ++idx;
    }
  }

  return spline_value_matrix;  // num_constraints x 3
}

Eigen::MatrixXd SplineTrajectory::getBasisMatrix(
  const std::map<double, std::map<std::string, Eigen::Vector3d, std::greater<>>> & constraints_map)
{
  const int num_constraints = TrajectoryGenerator::getNumberOfConstraints(constraints_map);

  Eigen::MatrixXd basis_matrix = Eigen::MatrixXd::Zero(num_constraints, num_constraints);

  int idx = 0;
  for (const auto & [time, state_map] : constraints_map) {
    for (const auto & [state, _] : state_map) {
      if (state == "x") {
        for (int i = 0; i < basis_matrix.cols(); ++i) {
          basis_matrix(idx, i) = pow(time, i);
        }
      } else if (state == "dx") {
        for (int i = 0; i < basis_matrix.cols(); ++i) {
          if (i - 1 < 0) {
            basis_matrix(idx, i) = 0.0;
          } else {
            basis_matrix(idx, i) = i * pow(time, i - 1);
          }
        }
      } else if (state == "ddx") {
        for (int i = 0; i < basis_matrix.cols(); ++i) {
          if (i - 2 < 0) {
            basis_matrix(idx, i) = 0.0;
          } else {
            basis_matrix(idx, i) = i * (i - 1) * pow(time, i - 2);
          }
        }
      }
      ++idx;
    }
  }

  if (basis_matrix.hasNaN()) {
    // TODO: Add error report
  }

  return basis_matrix;  // num_constraints x num_constraints
}

}  // namespace trajectory_generator
