cmake_minimum_required(VERSION 3.8)
project(custom_ur_franka)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)





install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY src
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/UR5PickAndMotion.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(complex_pick_and_motion src/complex_pick_and_motion.cpp)

ament_target_dependencies(complex_pick_and_motion
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_eigen
)
install(TARGETS
  complex_pick_and_motion
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
