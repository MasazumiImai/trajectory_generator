# trajectory_generator

[![Build Status](https://github.com/MasazumiImai/trajectory_generator/actions/workflows/build.yml/badge.svg)](https://github.com/MasazumiImai/trajectory_generator/actions)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

A lightweight C++ library for generating and interpolating trajectories for robotics.
It provides robust interpolation for N-dimensional vectors (e.g., joint angles, task-space positions) and SO(3) orientations (quaternions).

This package can be used as a standalone C++ library or from a ROS 2 (`colcon`) workspace.

## Features

* **N-Dimensional Vector Trajectory:** Generate local piecewise-polynomial trajectories with position, velocity, and acceleration constraints.
* **SO(3) Orientation Trajectory:** Perform accurate quaternion interpolation in the SO(3) space using Exponential and Logarithmic maps.
* **ROS 2 Ready:** Fully compatible with the ROS 2 build system (`colcon`).

## Input Contract

* Every vector waypoint must contain position. Supported combinations are position only,
  position and velocity, or position, velocity, and acceleration.
* Every orientation waypoint must contain a finite, non-zero quaternion. Quaternions are
  normalized internally; angular acceleration requires angular velocity at the same waypoint.
  Angular velocity and acceleration are spatial/world-frame values, following
  `q_dot = 0.5 * [0, omega] * q`.
* Waypoint times and state values must be finite, times must be unique, and vector dimensions
  must match the configured degrees of freedom.
* Each adjacent waypoint pair is interpolated independently in normalized local time with the
  lowest-degree polynomial that satisfies its endpoint constraints. Position is always continuous;
  at internal waypoints, velocity and acceleration continuity is guaranteed only when those
  derivatives are specified. Missing derivatives are not guessed or treated as zero. At an internal
  waypoint without velocity, an exact-time query uses the following segment's velocity.
* Each orientation segment uses `q(t) = Exp(phi(t)) * q_i` and the principal logarithm of the
  adjacent relative quaternion, so the selected endpoint-to-endpoint rotation is at most pi.
  Quaternion waypoints alone cannot encode intentional extra turns beyond pi. At an exact pi tie
  (a zero scalar quaternion component after normalization), the dominant rotation-axis component
  is chosen positive. Explicit derivative constraints may still make the polynomial path overshoot
  the principal rotation.
* Queries outside the trajectory interval hold the nearest endpoint position or orientation and
  return zero velocity.
* Segments whose duration, constraints, coefficients, or evaluation cannot be represented safely
  in `double` are rejected with an exception.

## Installation & Build

### Prerequisites

* C++ 17 or higher
* Eigen3
* GTest (only when building tests)
* ROS 2 (Optional, for `colcon` build)

### As a Standalone C++ Library

```bash
git clone https://github.com/MasazumiImai/trajectory_generator.git
cd trajectory_generator
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF
make
sudo make install
```

### As a ROS 2 Package

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/MasazumiImai/trajectory_generator.git
cd ~/ros2_ws
colcon build --packages-select traj_gen
source install/setup.bash
```

## Quick Start / Usage

### For Pure C++ Projects

If you are using standard CMake without ROS 2, simply find the package and link it in your `CMakeLists.txt`:

```CMake
find_package(traj_gen REQUIRED)

add_executable(your_app src/main.cpp)
target_link_libraries(your_app traj_gen::traj_gen)
```

### For ROS 2 Projects

If you are using this library within a ROS 2 package, add the dependency to your `package.xml`:

```xml
<depend>traj_gen</depend>
```

Then, configure your `CMakeLists.txt`:

```CMake
find_package(traj_gen REQUIRED)

add_executable(your_node src/your_node.cpp)
target_link_libraries(your_node PRIVATE traj_gen::traj_gen)
```

### C++ Example

Include the master header to access all trajectory generation features.

```C++
#include <iostream>
#include <memory>
#include <vector>

#include <traj_gen/traj_gen.hpp>

int main() {
  // 1. Define constraints
  int dof = 3;

  traj_gen::VectorStateConstraint start;
  start.time = 0.0;
  start.position = Eigen::VectorXd::Zero(dof);
  start.velocity = Eigen::VectorXd::Zero(dof);

  traj_gen::VectorStateConstraint end;
  end.time = 5.0;
  end.position = Eigen::VectorXd::Ones(dof) * 1.5;
  end.velocity = Eigen::VectorXd::Zero(dof);

  std::vector<traj_gen::VectorStateConstraint> constraints = traj_gen::createBoundaryConditions(start, end);

  // 2. Generate Spline Trajectory
  std::shared_ptr<traj_gen::VectorTrajectoryBase> planner = std::make_shared<traj_gen::VectorSpline>(constraints, dof);

  // 3. Get state at specific time
  double current_time = 2.5;
  Eigen::VectorXd pos = planner->getPosition(current_time);
  Eigen::VectorXd vel = planner->getVelocity(current_time);

  std::cout << "Position at t=2.5: \n" << pos << std::endl;

  return 0;
}
```

## Testing

If you want to run the unit tests:

```bash
cd trajectory_generator
mkdir -p build && cd build
cmake .. -DBUILD_TESTING=ON
make
ctest -V
```
