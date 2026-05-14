# trajectory_generator

[![Build Status](https://github.com/MasazumiImai/trajectory_generator/actions/workflows/build.yml/badge.svg)](https://github.com/MasazumiImai/trajectory_generator/actions)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

A lightweight and extensible C++ library for generating and interpolating trajectories for robotics. It provides robust interpolation for N-dimensional vectors (e.g., joint angles, task-space positions) and SO(3) orientations (quaternions).

This package can be used as a standalone C++ library or integrated seamlessly as a ROS 2 (`ament_cmake`) package.

## Features

* **N-Dimensional Vector Trajectory:** Generate smooth spline trajectories with position, velocity, and acceleration constraints.
* **SO(3) Orientation Trajectory:** Perform accurate quaternion interpolation in the SO(3) space using Exponential and Logarithmic maps (`expMap` / `logMap`).
* **ROS 2 Ready:** Fully compatible with the ROS 2 build system (`colcon`).

## Installation & Build

### Prerequisites

* C++ 17 or higher
* Eigen3
* ROS 2 (Optional, for `colcon` build)

### As a Standalone C++ Library

```bash
git clone https://github.com/MasazumiImai/trajectory_generator.git
cd trajectory_generator
mkdir build && cd build
cmake ..
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
````

Then, configure your `CMakeLists.txt`:

```CMake
find_package(traj_gen REQUIRED)

add_executable(your_node src/your_node.cpp)
ament_target_dependencies(your_node traj_gen)
```

### C++ Example

Include the master header to access all trajectory generation features.

```C++
#include <iostream>
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
