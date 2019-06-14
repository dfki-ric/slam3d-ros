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
)

find_package(slam3d REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros sensor_msgs geometry_msgs visualization_msgs std_srvs tf message_runtime
)

###########
## Build ##
###########

add_library(visualization
  src/GraphPublisher.cpp
  src/GpsPublisher.cpp
  src/LoopCloser.cpp
)
target_include_directories(visualization PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(visualization PUBLIC slam3d::core ${catkin_LIBRARIES})

# 3D-Mapper
add_executable(mapper_node
  src/mapper_node.cpp
  src/ros_common.cpp
  src/ros_tf.cpp
)

target_include_directories(mapper_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(mapper_node PUBLIC
  slam3d::core
  slam3d::graph-boost
  slam3d::sensor-pcl
  slam3d::sensor-gdal
  slam3d::solver-g2o
  visualization
  ${catkin_LIBRARIES}
)

# 2D-Mapper
#add_executable(mapper2d_node src/mapper2d_node.cpp)
#target_include_directories(mapper2d_node PRIVATE ${catkin_INCLUDE_DIRS})
#target_link_libraries(mapper2d_node PUBLIC
#  slam3d::core
#  slam3d::graph-boost
#  slam3d::sensor-pcl
#  slam3d::sensor-pointmatcher
#  slam3d::solver-g2o
#  visualization
#  ${catkin_LIBRARIES}
#)

# Velodyne Publisher
add_executable(velodyne_publisher src/velodyne_publisher.cpp)
target_include_directories(velodyne_publisher PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(velodyne_publisher ${catkin_LIBRARIES})

# Kitti Publisher
add_executable(kitti_publisher src/kitti_publisher.cpp)
target_include_directories(kitti_publisher PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(kitti_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS mapper_node velodyne_publisher kitti_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_slam3d.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
