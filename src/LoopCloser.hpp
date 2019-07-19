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
	LoopCloser(slam3d::Mapper* m, slam3d::PointCloudSensor* pcs);
	
	void closeLoopCB(slam3d::PointCloudSensor* pcs, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
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
