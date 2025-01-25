cmake_minimum_required(VERSION 3.8)
project(smartcar_msgs)

# Enable stricter warnings if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Enable testing and linting tools if in a test environment
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks (remove these lines when adding copyright)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate ROS interfaces from .msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  DEPENDENCIES geometry_msgs
)

# Install auxiliary files such as message definitions
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE
)

ament_package()

