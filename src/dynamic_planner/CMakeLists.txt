cmake_minimum_required(VERSION 3.8)
project(dynamic_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Start of what we added after the package creation
include_directories(include) 

add_library(dynamic_planner_lib
    src/map.cpp
    src/planner.cpp
)

add_executable(dynamic_planner_node
    src/dynamic_planner_node.cpp
)

add_executable(fake_robot src/fake_robot_motion.cpp)

add_executable(fake_laser_scan src/fake_laser_scan.cpp)

ament_target_dependencies(dynamic_planner_lib
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    std_msgs
)

ament_target_dependencies(dynamic_planner_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
)


ament_target_dependencies(fake_laser_scan
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)


ament_target_dependencies(fake_robot
    rclcpp
    geometry_msgs
    nav_msgs
    tf2_ros
)

target_link_libraries(dynamic_planner_node
    dynamic_planner_lib
)

install(TARGETS
    dynamic_planner_node
    dynamic_planner_lib
    fake_laser_scan
    fake_robot
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include/
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/
)

# Present in the package creation
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
