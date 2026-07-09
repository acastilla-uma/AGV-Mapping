cmake_minimum_required(VERSION 3.0.2)
project(scout_pointcloud_accumulator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  std_srvs
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs pcl_ros pcl_conversions tf tf2_ros tf2_sensor_msgs visualization_msgs std_srvs
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
  scripts/static_transform_rpy.py
  scripts/camera_lidar_calibrator.py
  scripts/camera_lidar_calibration_viewer.py
  scripts/camera_lidar_calibration_env.py
  scripts/agv_direction_marker.py
  scripts/mapping_quality_eval.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
