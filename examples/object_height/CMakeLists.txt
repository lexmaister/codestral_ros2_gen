cmake_minimum_required(VERSION 3.8)
project(object_height)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
file(GLOB srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs
)

# Python setup
ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR src/${PROJECT_NAME}
)

# Install Python executables - added entry point
install(PROGRAMS
    src/${PROJECT_NAME}/service_node.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME object_height_service
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(${PROJECT_NAME}_test test/test_service_node.py)
endif()

ament_package()
