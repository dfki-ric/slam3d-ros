#ifndef LOOP_CLOSER_HPP
#define LOOP_CLOSER_HPP

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

namespace slam3d
{
	class Mapper;
}

class LoopCloser
{
public:
	LoopCloser(slam3d::Mapper* m, slam3d::PointCloudSensor* pcs, bool space = false);
	
	void closeLoopCB(slam3d::PointCloudSensor* pcs, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void initLoopClosing(const slam3d::PointCloudMeasurement::Ptr& pc);
	void receiveTwist(const geometry_msgs::Twist::ConstPtr& twist);
	
	void setCovarianceScale(slam3d::ScalarType s) { mCovarianceScale = s; }

private:
	ros::Subscriber mTwistSubscriber;
	interactive_markers::InteractiveMarkerServer mServer;
	interactive_markers::MenuHandler mMenuHandler;
	
	geometry_msgs::Transform mMarkerPose;
	
	slam3d::Mapper* mMapper;
	slam3d::PointCloudMeasurement::Ptr mSourceCloud;
	
	slam3d::ScalarType mCovarianceScale;
	bool mUseSpaceMouse;
};

#endif
