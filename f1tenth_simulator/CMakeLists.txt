cmake_minimum_required(VERSION 3.8)
project(f1tenth_simulator)

# Default to C++20 (needed for designated initializers)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  ackermann_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  interactive_markers
  visualization_msgs
  std_msgs
  ament_index_cpp
)

######################################
# Build the core library
######################################

# Include directories
include_directories(include)

# Collect source files for the library
file(GLOB LIB_SRC_FILES src/*.cpp)

# Create the library
add_library(${PROJECT_NAME}_lib ${LIB_SRC_FILES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

######################################
# Build the node executables
######################################

# Simulator node
add_executable(simulator node/simulator.cpp)
target_link_libraries(simulator ${PROJECT_NAME}_lib)
ament_target_dependencies(simulator ${dependencies})

# Mux node
add_executable(mux node/mux.cpp)
target_link_libraries(mux ${PROJECT_NAME}_lib)
ament_target_dependencies(mux ${dependencies})

# Behavior controller node
add_executable(behavior_controller node/behavior_controller.cpp)
target_link_libraries(behavior_controller ${PROJECT_NAME}_lib)
ament_target_dependencies(behavior_controller ${dependencies})

# Random walk node
add_executable(random_walk node/random_walk.cpp)
target_link_libraries(random_walk ${PROJECT_NAME}_lib)
ament_target_dependencies(random_walk ${dependencies})

# Keyboard node
add_executable(keyboard node/keyboard.cpp)
target_link_libraries(keyboard ${PROJECT_NAME}_lib)
ament_target_dependencies(keyboard ${dependencies})

######################################
# Install targets
######################################

# Install library
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  simulator
  mux
  behavior_controller
  random_walk
  keyboard
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install param files
install(FILES params.yaml
  DESTINATION share/${PROJECT_NAME}
)

# Install map files
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install URDF/Xacro files
install(FILES racecar.xacro
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz config
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.rviz"
)

# Create logs directory in share
install(DIRECTORY DESTINATION share/${PROJECT_NAME}/logs)

######################################
# Export and finalize
######################################

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
