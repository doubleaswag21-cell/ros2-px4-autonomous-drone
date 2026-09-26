cmake_minimum_required(VERSION 3.8)
project(drone_3d_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)

add_executable(octomap_query
  src/octomap_query.cpp
)

ament_target_dependencies(
  octomap_query
  rclcpp
  geometry_msgs
)
target_include_directories(
  octomap_query PRIVATE
  ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(
  octomap_query
  ${OCTOMAP_LIBRARIES}
)

add_executable(astar_3d
  src/astar_3d.cpp
)

ament_target_dependencies(
  astar_3d
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)

target_include_directories(
  astar_3d PRIVATE
  ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(
  astar_3d
  ${OCTOMAP_LIBRARIES}
)

install(
  TARGETS
    octomap_query
    astar_3d
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
