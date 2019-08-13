#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <ros/ros.h>

void readSensorParameters(ros::NodeHandle& parent, slam3d::Sensor* sensor);
void readPointcloudSensorParameters(ros::NodeHandle& parent, slam3d::PointCloudSensor* pcs);
