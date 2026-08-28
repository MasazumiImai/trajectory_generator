cmake_minimum_required(VERSION 3.14)
project(traj_gen VERSION 0.2.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
  message(STATUS "traj_gen: CMAKE_BUILD_TYPE not set, defaulting to Release (-O3).")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
include(CTest)
include(GNUInstallDirs)
find_package(Eigen3 REQUIRED)

# library
add_library(${PROJECT_NAME} SHARED
  src/common.cpp
  src/spline.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# include directory
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "TRAJ_GEN_BUILDING_LIBRARY"
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
)

# install header file
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES LICENSE package.xml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)

install(FILES resource/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_DATADIR}/ament_index/resource_index/packages
)

install(FILES resource/ament_prefix_path.dsv
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/environment
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
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMinorVersion
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

# gtest
if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  add_executable(unit_test_traj_gen
    test/unit_test_traj_gen.cpp
  )
  target_link_libraries(unit_test_traj_gen
    PRIVATE ${PROJECT_NAME}
    GTest::gtest
  )

  include(GoogleTest)
  gtest_discover_tests(unit_test_traj_gen)

  set(install_test_prefix "${CMAKE_CURRENT_BINARY_DIR}/test_install")
  set(install_test_package_dir
    "${install_test_prefix}/${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
  add_test(NAME traj_gen.install
    COMMAND ${CMAKE_COMMAND}
      -DCMAKE_INSTALL_PREFIX=${install_test_prefix}
      -DCMAKE_INSTALL_CONFIG_NAME=$<CONFIG>
      -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_install.cmake
  )
  set_tests_properties(traj_gen.install PROPERTIES
    FIXTURES_SETUP traj_gen_installed
  )

  add_test(NAME traj_gen.install_consumer
    COMMAND ${CMAKE_CTEST_COMMAND}
      --build-and-test
      ${CMAKE_CURRENT_SOURCE_DIR}/test/install_consumer
      ${CMAKE_CURRENT_BINARY_DIR}/test_install_consumer
      --build-generator ${CMAKE_GENERATOR}
      --build-config $<CONFIG>
      --build-options
        -DCMAKE_PREFIX_PATH=${install_test_prefix}
        -DCMAKE_CXX_STANDARD=14
        -DEXPECTED_TRAJ_GEN_DIR=${install_test_package_dir}
      --test-command ${CMAKE_CTEST_COMMAND} -C $<CONFIG> --output-on-failure
  )
  set_tests_properties(traj_gen.install_consumer PROPERTIES
    FIXTURES_REQUIRED traj_gen_installed
  )
endif()
