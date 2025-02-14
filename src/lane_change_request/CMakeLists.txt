cmake_minimum_required(VERSION 3.8)
project(lane_change_request)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tier4_planning_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(tier4_autoware_utils REQUIRED)
find_package(vehicle_info_util REQUIRED)
find_package(motion_utils REQUIRED)
find_package(obstacle_avoidance_planner REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  ${tier4_planning_msgs_INCLUDE_DIRS}
  ${autoware_auto_planning_msgs_INCLUDE_DIRS}
  ${tier4_autoware_utils_INCLUDE_DIRS}
  ${vehicle_info_util_INCLUDE_DIRS}
  ${motion_utils_INCLUDE_DIRS}
  ${obstacle_avoidance_planner_INCLUDE_DIRS}
)
add_executable(trajectory_remmaper src/main.cpp)
ament_target_dependencies(trajectory_remmaper rclcpp tier4_planning_msgs autoware_auto_planning_msgs tier4_autoware_utils vehicle_info_util motion_utils obstacle_avoidance_planner)


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

install(TARGETS
  trajectory_remmaper
  DESTINATION lib/${PROJECT_NAME})

ament_package()
