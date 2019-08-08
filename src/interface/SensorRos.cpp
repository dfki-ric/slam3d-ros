#include "SensorRos.hpp"

#include <boost/format.hpp>

SensorRos::SensorRos(const std::string& sensor, const std::string robot, slam3d::Sensor* pntr)
: mNode((boost::format("/%1%") % sensor).str()), mSensor(pntr)
{
	mSensorName = sensor;
	mRobotName = robot;
}

void SensorRos::readParams()
{
	mSensor->setCovarianceScale(mNode.param("cov_scale", 1.0));

	double translation, rotation;
	mNode.param("min_translation", translation, 0.5);
	mNode.param("min_rotation", rotation, 0.1);
	mSensor->setMinPoseDistance(translation, rotation);
}

SensorRos::~SensorRos()
{
	
}