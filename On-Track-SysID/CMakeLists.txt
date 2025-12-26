cmake_minimum_required(VERSION 3.8)
project(on_track_sys_id)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python executables
install(PROGRAMS
  src/on_track_sys_id.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python helper modules
install(DIRECTORY
  src/helpers
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install parameter files
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

# Install model files
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
