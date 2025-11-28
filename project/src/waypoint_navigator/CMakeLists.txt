cmake_minimum_required(VERSION 3.0.2)
project(waypoint_navigator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  move_base_msgs
  visualization_msgs
  nav_msgs
  simulation
)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib tf2 tf2_ros tf2_geometry_msgs geometry_msgs move_base_msgs visualization_msgs nav_msgs simulation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_navigator_node src/waypoint_navigator_node.cpp)
target_link_libraries(waypoint_navigator_node ${catkin_LIBRARIES})

install(TARGETS waypoint_navigator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)