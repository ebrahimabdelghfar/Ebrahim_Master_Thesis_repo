cmake_minimum_required(VERSION 3.16)
project(mpc_path_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(f1tenth_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(adaptive_controller_interfaces REQUIRED)
find_package(track_geometry_utils REQUIRED)

# OSQP + osqp-eigen are not vendored anywhere else in this workspace
# (pinned to the versions already validated together elsewhere on this
# machine: osqp v0.6.3 / osqp-eigen v0.8.1). osqp-eigen's own CMakeLists
# locates OSQP via find_package(osqp), which needs an *installed* package
# config (osqp's build-tree config export is unreliable across CMake
# versions in exactly this nested-FetchContent setup), so OSQP is fetched
# and installed into a local prefix at configure time (a small superbuild
# step) before osqp-eigen is fetched.
include(FetchContent)

set(BUILD_TESTING_SAVED ${BUILD_TESTING})
set(BUILD_TESTING OFF)

FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG v0.6.3
)
FetchContent_Populate(osqp)

set(OSQP_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/osqp_install")
set(OSQP_BUILD_DIR "${CMAKE_BINARY_DIR}/osqp_build")
if(NOT EXISTS "${OSQP_INSTALL_PREFIX}/lib/cmake/osqp/osqp-config.cmake")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -S "${osqp_SOURCE_DIR}" -B "${OSQP_BUILD_DIR}"
            -DBUILD_SHARED_LIBS=ON
            -DCMAKE_INSTALL_PREFIX=${OSQP_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE=Release
    RESULT_VARIABLE OSQP_CONFIGURE_RESULT)
  if(NOT OSQP_CONFIGURE_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to configure vendored OSQP (v0.6.3)")
  endif()
  execute_process(
    COMMAND ${CMAKE_COMMAND} --build "${OSQP_BUILD_DIR}" --target install -- -j
    RESULT_VARIABLE OSQP_BUILD_RESULT)
  if(NOT OSQP_BUILD_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to build/install vendored OSQP (v0.6.3)")
  endif()
endif()
set(osqp_DIR "${OSQP_INSTALL_PREFIX}/lib/cmake/osqp" CACHE PATH "" FORCE)
list(APPEND CMAKE_PREFIX_PATH "${OSQP_INSTALL_PREFIX}")

FetchContent_Declare(
  osqp-eigen
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
  GIT_TAG v0.8.1
)
FetchContent_MakeAvailable(osqp-eigen)

set(BUILD_TESTING ${BUILD_TESTING_SAVED})

add_library(${PROJECT_NAME}_core
  src/vehicle_model.cpp
  src/reference_trajectory_handler.cpp
  src/solver_interface.cpp
  src/mpc_controller.cpp
  src/parameter_manager.cpp
  src/debug_publisher.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  ${f1tenth_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC tf2_geometry_msgs::tf2_geometry_msgs)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC
  adaptive_controller_interfaces
  track_geometry_utils
)

add_executable(mpc_node src/mpc_node.cpp)
target_link_libraries(mpc_node ${PROJECT_NAME}_core)
# libosqp.so lives in the private osqp_install superbuild prefix under
# build/, which does not survive a bare `rm -rf build`; use that only for
# the build-tree RPATH, and install a copy of libosqp.so alongside this
# package's own libraries so the installed mpc_node keeps working via a
# relative $ORIGIN RPATH even after build/ is deleted.
set_target_properties(mpc_node PROPERTIES
  BUILD_RPATH "${OSQP_INSTALL_PREFIX}/lib"
  INSTALL_RPATH "$ORIGIN/..")

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES "${OSQP_INSTALL_PREFIX}/lib/libosqp.so"
  DESTINATION lib)

install(TARGETS mpc_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_vehicle_model test/test_vehicle_model.cpp)
  target_link_libraries(test_vehicle_model ${PROJECT_NAME}_core)
  set_target_properties(test_vehicle_model PROPERTIES BUILD_RPATH "${OSQP_INSTALL_PREFIX}/lib")

  ament_add_gtest(test_reference_trajectory_handler test/test_reference_trajectory_handler.cpp)
  target_link_libraries(test_reference_trajectory_handler ${PROJECT_NAME}_core)
  set_target_properties(test_reference_trajectory_handler PROPERTIES BUILD_RPATH "${OSQP_INSTALL_PREFIX}/lib")
endif()

ament_package()
