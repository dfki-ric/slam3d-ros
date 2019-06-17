#ifndef LOOP_CLOSER_HPP
#define LOOP_CLOSER_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Transform.h>

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>


class LoopCloser
{
public:
	LoopCloser();
	
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void closeLoopCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	
	geometry_msgs::Transform getTransform();
	void initLoopClosing(const slam3d::PointCloud::Ptr& pc);

private:
	interactive_markers::InteractiveMarkerServer mServer;
	interactive_markers::MenuHandler mMenuHandler;
	
	geometry_msgs::Transform mMarkerPose;
};

#endif
