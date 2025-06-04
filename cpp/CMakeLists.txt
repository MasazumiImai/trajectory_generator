cmake_minimum_required(VERSION 3.8)

project(trajectory_generator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

# find dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# include directory
set(INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# library
add_library(${PROJECT_NAME} SHARED
  src/trajectory_generator.cpp
  src/spline.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

# install header files
install(DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# gtest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

include(GoogleTest)

add_executable(unit_test_trajectory_generator test/unit_test_trajectory_generator.cpp)
target_include_directories(unit_test_trajectory_generator PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(unit_test_trajectory_generator
  PRIVATE
    GTest::gtest
    GTest::gtest_main
    Eigen3::Eigen
    ${PROJECT_NAME}
)
gtest_discover_tests(unit_test_trajectory_generator)
