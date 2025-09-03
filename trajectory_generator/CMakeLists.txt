cmake_minimum_required(VERSION 3.8)
project(trajectory_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# library
ament_auto_add_library(trajectory_generator
  src/trajectory_generator.cpp
  src/spline.cpp
)

# install header files
install(DIRECTORY include/
  DESTINATION include
)

# gtest
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(unit_test_trajectory_generator test/unit_test_trajectory_generator.cpp)
  target_include_directories(unit_test_trajectory_generator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(unit_test_trajectory_generator trajectory_generator)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
