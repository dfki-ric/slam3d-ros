#ifndef LOOP_CLOSER_HPP
#define LOOP_CLOSER_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Transform.h>

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

namespace slam3d
{
	class Mapper;
}

class LoopCloser
{
public:
	LoopCloser(slam3d::Mapper* m);
	
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void closeLoopCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	
	geometry_msgs::Transform getTransform();
	void initLoopClosing(const slam3d::PointCloudMeasurement::Ptr& pc);
	
	void setCovarianceScale(slam3d::ScalarType s) { mCovarianceScale = s; }

private:
	interactive_markers::InteractiveMarkerServer mServer;
	interactive_markers::MenuHandler mMenuHandler;
	
	geometry_msgs::Transform mMarkerPose;
	
	slam3d::Mapper* mMapper;
	slam3d::PointCloudMeasurement::Ptr mSourceCloud;
	
	slam3d::ScalarType mCovarianceScale;
};

#endif
