cmake_minimum_required(VERSION 3.0.2)
project(perception_pipeline)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_node src/tf.cpp)

target_link_libraries(tf_node
  ${catkin_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

