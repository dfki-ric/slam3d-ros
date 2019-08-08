add_library(interface
	SensorRos.cpp
	PointcloudSensorRos.cpp
	ros_common.cpp
	ros_tf.cpp
)

target_include_directories(interface
	PUBLIC
		${catkin_INCLUDE_DIRS}
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(interface
	PUBLIC
		slam3d::core
		slam3d::sensor-pcl
		${catkin_LIBRARIES}
)

## Install header files
install(
	FILES
		SensorRos.hpp
		PointcloudSensorRos.hpp
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS interface EXPORT slam3d_ros-targets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

set_target_properties(interface PROPERTIES OUTPUT_NAME slam3d_ros_interface)
