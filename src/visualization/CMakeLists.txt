add_library(visualization
  GraphPublisher.cpp
  GpsPublisher.cpp
  LoopCloser.cpp
)
target_include_directories(visualization
	PUBLIC
		${catkin_INCLUDE_DIRS}
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
	
target_link_libraries(visualization
	PUBLIC
		slam3d::core
		${catkin_LIBRARIES}
)

## Install header files
install(
	FILES
		GraphPublisher.hpp
		GpsPublisher.hpp
		LoopCloser.hpp
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS visualization EXPORT slam3d_ros-targets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

set_target_properties(visualization PROPERTIES OUTPUT_NAME slam3d_ros_visualization)
