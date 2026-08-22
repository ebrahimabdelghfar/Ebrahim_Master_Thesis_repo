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

# All 5 thirdparty libs are vendored under thirdparty_lib/ at the repo
# root and built out-of-tree into an isolated, repo-local install prefix
# by scripts/thirdparty_lib_build/build_all.sh (no network fetch at
# configure time).
set(THIRDPARTY_INSTALL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../install/thirdparty_lib")
list(APPEND CMAKE_PREFIX_PATH
  "${THIRDPARTY_INSTALL_ROOT}/osqp-0.6.3"
  "${THIRDPARTY_INSTALL_ROOT}/osqp-eigen-0.8.1"
  "${THIRDPARTY_INSTALL_ROOT}/casadi-3.7.0"
  "${THIRDPARTY_INSTALL_ROOT}/acados-0.5.5")

find_package(osqp REQUIRED CONFIG)
find_package(OsqpEigen REQUIRED CONFIG)
find_package(casadi REQUIRED CONFIG)

# acados installs its CMake config (and hpipm's/qpOASES_e's) to a plain
# <prefix>/cmake/ dir, which isn't a location find_package() searches
# automatically even via CMAKE_PREFIX_PATH - point the _DIR vars at it
# directly. blasfeo's config lands in the standard share/cmake/ location.
set(acados_DIR "${THIRDPARTY_INSTALL_ROOT}/acados-0.5.5/cmake")
set(hpipm_DIR "${THIRDPARTY_INSTALL_ROOT}/acados-0.5.5/cmake")
set(qpOASES_e_DIR "${THIRDPARTY_INSTALL_ROOT}/acados-0.5.5/cmake")
set(blasfeo_DIR "${THIRDPARTY_INSTALL_ROOT}/acados-0.5.5/share/cmake/blasfeo")
find_package(acados REQUIRED CONFIG)

# Ipopt (Autotools build, no CMake config export) - manual include/lib
# paths, same pattern scripts/thirdparty_lib_build/casadi.sh uses for it.
set(IPOPT_INSTALL_DIR "${THIRDPARTY_INSTALL_ROOT}/Ipopt-stable-3.14")
set(IPOPT_INCLUDE_DIRS "${IPOPT_INSTALL_DIR}/include/coin-or")
set(IPOPT_LIBRARIES "${IPOPT_INSTALL_DIR}/lib/libipopt.so")

add_library(${PROJECT_NAME}_core
  src/vehicle_model.cpp
  src/reference_trajectory_handler.cpp
  src/solver_interface.cpp
  src/acados_mpc_solver.cpp
  src/mpc_controller.cpp
  src/parameter_manager.cpp
  src/debug_publisher.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "${IPOPT_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
  casadi::casadi
  acados
  "${IPOPT_LIBRARIES}"
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
# Thirdparty libs live at a stable, repo-local install prefix outside
# build/ (see THIRDPARTY_INSTALL_ROOT above), so it survives `rm -rf
# build` and both the build-tree and installed binary can RPATH straight
# to it.
set(THIRDPARTY_LIB_RPATH
  "${THIRDPARTY_INSTALL_ROOT}/osqp-0.6.3/lib;${THIRDPARTY_INSTALL_ROOT}/osqp-eigen-0.8.1/lib;${THIRDPARTY_INSTALL_ROOT}/casadi-3.7.0/lib;${THIRDPARTY_INSTALL_ROOT}/acados-0.5.5/lib;${THIRDPARTY_INSTALL_ROOT}/Ipopt-stable-3.14/lib")
set_target_properties(mpc_node PROPERTIES
  BUILD_RPATH "${THIRDPARTY_LIB_RPATH}"
  INSTALL_RPATH "${THIRDPARTY_LIB_RPATH}")

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
  set_target_properties(test_vehicle_model PROPERTIES BUILD_RPATH "${THIRDPARTY_LIB_RPATH}")

  ament_add_gtest(test_reference_trajectory_handler test/test_reference_trajectory_handler.cpp)
  target_link_libraries(test_reference_trajectory_handler ${PROJECT_NAME}_core)
  set_target_properties(test_reference_trajectory_handler PROPERTIES BUILD_RPATH "${THIRDPARTY_LIB_RPATH}")

  ament_add_gtest(test_acados_solver test/test_acados_solver.cpp)
  target_link_libraries(test_acados_solver ${PROJECT_NAME}_core)
  set_target_properties(test_acados_solver PROPERTIES BUILD_RPATH "${THIRDPARTY_LIB_RPATH}")

  ament_add_gtest(test_mpc_controller test/test_mpc_controller.cpp)
  target_link_libraries(test_mpc_controller ${PROJECT_NAME}_core)
  set_target_properties(test_mpc_controller PROPERTIES BUILD_RPATH "${THIRDPARTY_LIB_RPATH}")
endif()

ament_package()
