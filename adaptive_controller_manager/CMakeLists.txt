cmake_minimum_required(VERSION 3.16)
project(adaptive_controller_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(f1tenth_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(adaptive_controller_interfaces REQUIRED)
find_package(track_geometry_utils REQUIRED)

add_library(${PROJECT_NAME}_core src/parameter_manager.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}_core PUBLIC rclcpp::rclcpp)

add_executable(adaptive_controller_manager_node src/manager_node.cpp)
target_link_libraries(adaptive_controller_manager_node PUBLIC
  ${PROJECT_NAME}_core
  ${nav_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${f1tenth_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)
ament_target_dependencies(adaptive_controller_manager_node PUBLIC
  adaptive_controller_interfaces
  track_geometry_utils
)

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS adaptive_controller_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
