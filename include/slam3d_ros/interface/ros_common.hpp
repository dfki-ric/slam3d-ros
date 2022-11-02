#ifndef ROS_COMMON_HPP
#define ROS_COMMON_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/Imu.h>

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

class Imu : public PoseSensor
{
public:
	Imu(const std::string& n, Graph* g, Logger* l);
	Quaternion getOrientation(timeval stamp);
	Transform getPose(timeval stamp);
	Covariance<3> getCovariance(timeval stamp);
	void handleNewVertex(IdType vertex);
	void update(const sensor_msgs::Imu::ConstPtr& imu);

protected:
	sensor_msgs::Imu mMeasurement;
	ros::Subscriber mSubscriber;

	slam3d::Transform mLastImuPose;
	slam3d::IdType mLastVertex;
};

#endif
