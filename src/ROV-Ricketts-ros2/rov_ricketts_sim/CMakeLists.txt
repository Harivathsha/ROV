cmake_minimum_required(VERSION 3.8)
project(rov_ricketts_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)   # <-- add this
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install Python executables
install(
  PROGRAMS
    scripts/thruster_teleop_node.py     # <-- path to your node (inside 'scripts/' or 'src/')
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources
install(
  DIRECTORY config launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
