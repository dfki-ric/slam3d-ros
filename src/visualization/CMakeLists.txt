add_library(slam3d_ros_visualization
  GraphPublisher.cpp
  GpsPublisher.cpp
  LoopCloser.cpp
)

target_link_libraries(slam3d_ros_visualization
	PUBLIC
		slam3d::core
		${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS slam3d_ros_visualization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
