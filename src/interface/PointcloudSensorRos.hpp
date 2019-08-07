#include <ros/ros.h>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

#include "ros_common.hpp"

class PointcloudSensorRos
{
public:
	PointcloudSensorRos(const std::string& sensor, const std::string robot);
	~PointcloudSensorRos();
	
	bool addScanWithOdometry(const slam3d::PointCloud::ConstPtr& pcl,
                             const slam3d::Transform& laser_pose,
                             const slam3d::Transform& odom);
	
	// For debugging only, this method will be removed!
	slam3d::PointCloudSensor* getSensor() {return mPclSensor;}
	
protected:
	slam3d::PointCloudSensor* mPclSensor;
	RosLogger mLogger;
	
private:
	double mScanResolution;
	bool mSeqScanMatch;
	
	std::string mRobotName;
	std::string mSensorName;
};