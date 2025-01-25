cmake_minimum_required(VERSION 3.8)
project(smart_car)

# Add compile options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)  # Required for processing Xacro files
find_package(ament_cmake_python REQUIRED)  # Required for Python package setup

# Optional testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Skip copyright and lint checks (uncomment if needed)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()  # Automatically add test dependencies
endif()

# Install directories for resources and plugins
install(DIRECTORY urdf launch config world
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE  # Exclude any version control artifacts
)

# Install plugin directory, if it contains custom Gazebo plugins
install(DIRECTORY plugins 
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.so"  # Only shared libraries (.so files)
)

# Set up the Python package for installation
ament_python_install_package(${PROJECT_NAME})

# Install Python executable scripts
install(PROGRAMS script/odom.py  # Ensure odom.py exists in script folder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()  # Declare this as a ROS 2 package

