cmake_minimum_required(VERSION 3.14)
project(traj_gen_install_consumer LANGUAGES CXX)

find_package(traj_gen 0.1 REQUIRED)
if(NOT "${traj_gen_DIR}" STREQUAL "${EXPECTED_TRAJ_GEN_DIR}")
  message(FATAL_ERROR "traj_gen was not found in the test install prefix")
endif()

add_executable(install_consumer main.cpp)
target_link_libraries(install_consumer PRIVATE traj_gen::traj_gen)

enable_testing()
add_test(NAME install_consumer.runs COMMAND install_consumer)
