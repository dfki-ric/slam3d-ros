add_library(slam3d_ros_interface
	ros_common.cpp
	ros_tf.cpp
	helper_functions.cpp
)

target_link_libraries(slam3d_ros_interface
	PUBLIC
		slam3d::core
		slam3d::sensor-pcl
		${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS slam3d_ros_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
