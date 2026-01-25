# trajectory_generator

## How to Install

```bash
mkdir -p ~/ros2_ws/src && ~/ros2_ws/src
git clone https://github.com/MasazumiImai/trajectory_generator.git
```

## How to Build

### C++

```bash
cd trajectory_generator
mkdir -p build && cd build
cmake .. && make && sudo make install
```

### ROS 2

```bash
cd ~/ros2_ws
colcon build --packages-select trajectory_generator && . install/setup.bash
```

## How to Test

### C++

```bash
mkdir -p build && cd build
cmake .. -DBUILD_TESTING=ON && make
ctest -V
```
