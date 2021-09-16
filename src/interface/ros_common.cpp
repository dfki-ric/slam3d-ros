#include "ros_common.hpp"
#include <tf2_eigen/tf2_eigen.h>

ros::Time fromTimeval(timeval tv)
{
	ros::Time rt;
	rt.sec = tv.tv_sec;
	rt.nsec = tv.tv_usec * 1000;
	return rt;
}

timeval fromRosTime(ros::Time rt)
{
	timeval tv;
	tv.tv_sec = rt.sec;
	tv.tv_usec = rt.nsec / 1000;
	return tv;
}

timeval RosClock::now()
{
	return fromRosTime(ros::Time::now());
}

RosLogger::RosLogger():Logger(RosClock())
{
}

geometry_msgs::TransformStamped eigen2tf(const slam3d::Transform& tf,
	const std::string& source, const std::string& target, const ros::Time& t)
{
	geometry_msgs::TransformStamped result = tf2::eigenToTransform(tf);
	result.header.stamp = t;
	result.header.frame_id = source;
	result.child_frame_id = target;
	return result;
}

void RosLogger::message(LOG_LEVEL lvl, const std::string& message)
{
	switch(lvl)
	{
	case DEBUG:
		ROS_DEBUG("%s", message.c_str());
		break;
	case INFO:
		ROS_INFO("%s", message.c_str());
		break;
	case WARNING:
		ROS_WARN("%s", message.c_str());
		break;
	case ERROR:
		ROS_ERROR("%s", message.c_str());
		break;
	case FATAL:
		ROS_FATAL("%s", message.c_str());
		break;
	}
}
