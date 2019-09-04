#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <ros/ros.h>

void readSensorParameters(ros::NodeHandle& parent, slam3d::Sensor* sensor);
void readScanSensorParameters(ros::NodeHandle& parent, slam3d::ScanSensor* ss);
void readPointcloudSensorParameters(ros::NodeHandle& parent, slam3d::PointCloudSensor* pcs);
