cmake_minimum_required(VERSION 2.8.3)
project(slam3d_ros)
add_definitions(-std=c++11)

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
)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(SLAM3D REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros sensor_msgs geometry_msgs visualization_msgs std_srvs tf message_runtime
)

###########
## Build ##
###########

include_directories(${SLAM3D_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(mapper_node src/mapper_node.cpp)
add_executable(velodyne_publisher src/velodyne_publisher.cpp)
add_executable(kitti_publisher src/kitti_publisher.cpp)

target_link_libraries(mapper_node
  ${SLAM3D_LIBRARY}
  ${catkin_LIBRARIES}
)

target_link_libraries(velodyne_publisher ${catkin_LIBRARIES})
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
