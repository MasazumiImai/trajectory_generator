cmake_minimum_required(VERSION 3.14)
project(traj_gen_add_subdirectory_consumer LANGUAGES CXX)

set(BUILD_TESTING ON CACHE BOOL "" FORCE)
add_subdirectory("${TRAJ_GEN_SOURCE_DIR}" traj_gen)

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(FATAL_ERROR "traj_gen changed the parent CMAKE_BUILD_TYPE")
endif()
if(TARGET unit_test_traj_gen)
  message(FATAL_ERROR "traj_gen added its tests to the parent project")
endif()

add_executable(traj_gen_add_subdirectory_consumer main.cpp)
target_link_libraries(traj_gen_add_subdirectory_consumer PRIVATE traj_gen::traj_gen)

enable_testing()
add_test(NAME runs COMMAND traj_gen_add_subdirectory_consumer)
