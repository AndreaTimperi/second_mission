cmake_minimum_required(VERSION 3.0.2)
project(second_mission)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  navigation_manager_msgs
  pluginlib
  rosbag
  roscpp
  rospy
  state_machine_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES second_mission
 CATKIN_DEPENDS pluginlib rosbag roscpp rospy std_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs
 DEPENDS Eigen3
#  DEPENDS system_lib
)


include_directories(include
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(second_mission_node src/InspectionMission.cpp src/main_mission.cpp )
add_executable(second_mission_online_node src/InspectionMissionOnline.cpp src/main_mission_online.cpp )

add_dependencies(second_mission_node ${catkin_EXPORTED_TARGETS})
add_dependencies(second_mission_online_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(second_mission_node
${catkin_LIBRARIES} )
target_link_libraries(second_mission_online_node
${catkin_LIBRARIES} )

