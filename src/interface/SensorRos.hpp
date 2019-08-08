#include <ros/ros.h>
#include <slam3d/core/Sensor.hpp>

#include "ros_common.hpp"

class SensorRos
{
public:
	SensorRos(const std::string& sensor, const std::string robot, slam3d::Sensor* pntr);
	~SensorRos();
	
	void readParams();
	
protected:
	slam3d::Sensor* mSensor;
	ros::NodeHandle mNode;
	
	std::string mRobotName;
	std::string mSensorName;
};