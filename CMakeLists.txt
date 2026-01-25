cmake_minimum_required(VERSION 3.14)
project(trajectory_generator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(rclcpp QUIET)

# library
add_library(${PROJECT_NAME} SHARED
  src/trajectory_generator.cpp
  src/spline.cpp
)

# include directory
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "TRAJECTORY_GENERATOR_BUILDING_LIBRARY"
)

# install setting
include(GNUInstallDirs)

# install header file
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# export targets
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# generate version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion
)

# generate config file
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# install config file
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(rclcpp_FOUND)
  add_executable(trajectory_generator_node
    src/trajectory_generator_node.cpp
  )

  target_link_libraries(trajectory_generator_node
    ${PROJECT_NAME}
    rclcpp::rclcpp
  )

  install(TARGETS trajectory_generator_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# gtest
if(BUILD_TESTING)
  enable_testing()

  find_package(GTest REQUIRED)

  add_executable(unit_test_trajectory_generator
    test/unit_test_trajectory_generator.cpp
  )
  target_link_libraries(unit_test_trajectory_generator
    ${PROJECT_NAME}
    GTest::gtest
  )

  include(GoogleTest)
  gtest_discover_tests(unit_test_trajectory_generator)
endif()
