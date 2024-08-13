cmake_minimum_required(VERSION 3.8)
project(localization_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(voxl_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
localization_package/inertial_camera_coord.py
DESTINATION lib/${PROJECT_NAME}
)

add_executable(localization_node src/object_inertial.cpp)
ament_target_dependencies(localization_node
  rclcpp
  geometry_msgs
  sensor_msgs
  px4_msgs
  Eigen3
)

install(TARGETS
  localization_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(localization_node_inertial src/pixel_inertial.cpp)
ament_target_dependencies(localization_node_inertial
  rclcpp
  geometry_msgs
  sensor_msgs
  px4_msgs
  std_msgs
  Eigen3
)

install(TARGETS
  localization_node_inertial
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
