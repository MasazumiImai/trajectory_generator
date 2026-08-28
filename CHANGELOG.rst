^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package traj_gen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-08-28)
------------------

* Require finite, uniquely timed waypoints with position or orientation at every point.
* Reject acceleration-only constraints and normalize finite, non-zero quaternions.
* Replace the global vector solve with normalized, piecewise endpoint polynomials.
* Interpolate each orientation segment on SO(3) using the adjacent principal rotation.
* Define angular velocity and acceleration in the spatial/world frame.
* Hold endpoint position and orientation outside the trajectory interval and return zero
  derivatives.
* Add const trajectory queries and public linear and angular acceleration evaluation.
* Export a standalone CMake package and a ROS 2 discoverable plain-CMake package.

This release is not API- or ABI-compatible with the previous unreleased implementation.
