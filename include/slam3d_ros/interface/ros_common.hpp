#ifndef ROS_COMMON_HPP
#define ROS_COMMON_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <slam3d/core/Logger.hpp>
#include <slam3d/core/Clock.hpp>
#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/core/Graph.hpp>

using namespace slam3d;

ros::Time fromTimeval(timeval tv);
timeval fromRosTime(ros::Time rt);

tf::Transform eigen2tf(Transform in);
Transform tf2eigen(tf::Transform in);

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
