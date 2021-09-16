#ifndef ROS_COMMON_HPP
#define ROS_COMMON_HPP

#include <ros/ros.h>
#include <tf2/convert.h>

#include <slam3d/core/Logger.hpp>
#include <slam3d/core/Clock.hpp>
#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/core/Graph.hpp>

using namespace slam3d;

ros::Time fromTimeval(timeval tv);
timeval fromRosTime(ros::Time rt);

geometry_msgs::TransformStamped eigen2tf(const slam3d::Transform& tf,
	const std::string& source, const std::string& traget, const ros::Time& t);

class RosClock : public Clock
{
public:
	virtual timeval now();
};

class RosLogger : public Logger
{
public:
	RosLogger();
	virtual void message(LOG_LEVEL lvl, const std::string& message);
};

#endif
