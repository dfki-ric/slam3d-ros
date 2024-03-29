cmake_minimum_required(VERSION 2.8.3)
cmake_policy(VERSION 3.5.1)
project(slam3d_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  message_generation
  sensor_msgs
  geometry_msgs
  tf_conversions
  visualization_msgs
  std_srvs
  laser_geometry
  cv_bridge
  interactive_markers
  eigen_conversions
)

find_package(slam3d 2.5 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    slam3d_ros_interface
	slam3d_ros_visualization
  CATKIN_DEPENDS
    roscpp
	pcl_ros
	sensor_msgs
	geometry_msgs
	visualization_msgs
	std_srvs
	tf
	message_runtime
  DEPENDS
    slam3d
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/slam3d_ros/visualization
  include/slam3d_ros/interface
)

add_subdirectory(src)

# 3D-Mapper
add_executable(mapper_node
  src/mapper_node.cpp
)

target_include_directories(mapper_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(mapper_node PUBLIC
  slam3d::core
  slam3d::graph-boost
  slam3d::sensor-pcl
  slam3d::sensor-gdal
  slam3d::solver-g2o
  slam3d_ros_visualization
  slam3d_ros_interface
  ${catkin_LIBRARIES}
)

# 2D-Mapper
add_executable(mapper2d_node src/mapper2d_node.cpp)
target_include_directories(mapper2d_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(mapper2d_node PUBLIC
  slam3d::core
  slam3d::graph-boost
  slam3d::sensor-pcl
  slam3d::sensor-pointmatcher
  slam3d::solver-g2o
  slam3d_ros_visualization
  slam3d_ros_interface
  ${catkin_LIBRARIES}
)

# Velodyne Publisher
add_executable(velodyne_publisher src/velodyne_publisher.cpp)
target_include_directories(velodyne_publisher PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(velodyne_publisher ${catkin_LIBRARIES})

# Kitti Publisher
add_executable(kitti_publisher src/kitti_publisher.cpp)
target_include_directories(kitti_publisher PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(kitti_publisher ${catkin_LIBRARIES})

add_executable(kitti_mapper src/kitti_mapper.cpp)
target_include_directories(kitti_mapper PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(kitti_mapper PUBLIC
  slam3d::graph-boost
  slam3d::sensor-pcl
  slam3d::solver-g2o
  slam3d_ros_visualization
  slam3d_ros_interface
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS mapper_node velodyne_publisher kitti_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

