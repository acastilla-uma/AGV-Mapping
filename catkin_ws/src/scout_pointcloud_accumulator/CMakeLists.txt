cmake_minimum_required(VERSION 3.0.2)
project(scout_pointcloud_accumulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf
  tf2_ros
  tf2_sensor_msgs
  std_srvs
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs pcl_ros pcl_conversions tf tf2_ros tf2_sensor_msgs std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(accumulator_node src/accumulator.cpp)
target_link_libraries(accumulator_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(accumulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS accumulator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/mapping_metadata_logger.py
  scripts/static_transform_rpy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
