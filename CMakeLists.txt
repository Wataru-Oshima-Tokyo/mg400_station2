cmake_minimum_required(VERSION 3.5)
project(mg400_station2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mg400_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(techshare_ros_pkg2 REQUIRED)
find_package(mg400_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mg400_station src/mg400_station.cpp)
ament_target_dependencies(mg400_station
  geometry_msgs
  rclcpp
  rclcpp_action
  mg400_msgs
  geometry_msgs
  std_srvs
  techshare_ros_pkg2
  )
# target_link_libraries(mg400_station ${PROJECT_NAME})
target_include_directories(
  mg400_station PRIVATE include)

add_executable(mg400_pose_publisher src/mg400_pose_publisher.cpp)
  ament_target_dependencies(mg400_pose_publisher
    geometry_msgs
    rclcpp
    rclcpp_action
    mg400_msgs
    mg400_interface
    geometry_msgs
    std_srvs
    techshare_ros_pkg2
    )
# target_link_libraries(mg400_pose_publisher ${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  mg400_station
  mg400_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    include
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)
ament_package()
