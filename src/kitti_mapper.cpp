#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Empty.h>

#include <boost/uuid/uuid_io.hpp>

#include <slam3d/core/Mapper.hpp>
#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <slam3d/sensor/gdal/GpsSensor.hpp>
#include <slam3d/solver/g2o/G2oSolver.hpp>

#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>

#include "GraphPublisher.hpp"
#include "GpsPublisher.hpp"
#include "ros_common.hpp"
#include "ros_tf.hpp"
#include "helper_functions.hpp"

#define ROBOT_NAME "kitti"
#define SENSOR_NAME "velodyne"

#define MAP_FRAME "map"
#define SENSOR_FRAME "velodyne_laser"

tf::TransformBroadcaster* gTransformBroadcaster;

ros::Time gLastTime;
ros::Publisher* gMapPublisher;
ros::Publisher* gSignalPublisher;
GraphPublisher* gGraphPublisher;

slam3d::BoostGraph* gGraph;
slam3d::Mapper* gMapper;
slam3d::PointCloudSensor* gPclSensor;
slam3d::G2oSolver* gSolver;
slam3d::G2oSolver* gPatchSolver;

double gScanResolution;

void publishTransforms(const ros::Time& t)
{
	slam3d::Transform current = gMapper->getCurrentPose();
	if(current.matrix().determinant() == 0)
	{
		ROS_ERROR("Current pose from mapper has 0 determinant!");
	}

	tf::StampedTransform robot_in_map(eigen2tf(current), t, MAP_FRAME, SENSOR_FRAME);
	gTransformBroadcaster->sendTransform(robot_in_map);
}

void publishGraph(const ros::Time& t)
{
	gGraphPublisher->publishNodes(t, MAP_FRAME);
	gGraphPublisher->publishEdges(SENSOR_NAME, t, MAP_FRAME);
}

void receivePointCloud(const slam3d::PointCloud::ConstPtr& pcl)
{
	ROS_DEBUG("Received scan");
	gSignalPublisher->publish(std_msgs::Empty());

	// Get the pose of the laser scanner
	ros::Time t;
	t.fromNSec(pcl->header.stamp * 1000);
	
	if(t < gLastTime)
	{
		ROS_WARN("Received late Scan!");
		return;
	}
	gLastTime = t;

	slam3d::PointCloud::Ptr cloud;
	if(gScanResolution > 0)
	{
		cloud = gPclSensor->downsample(pcl, gScanResolution);
	}else
	{	
		cloud = slam3d::PointCloud::Ptr(new slam3d::PointCloud(*pcl));
	}
	
	slam3d::PointCloudMeasurement::Ptr m(
		new slam3d::PointCloudMeasurement(cloud, ROBOT_NAME, SENSOR_NAME, slam3d::Transform::Identity()));
	
	if(gPclSensor->addMeasurement(m))
	{
		gPclSensor->linkLastToNeighbors(true);
		publishTransforms(t);
		publishGraph(t);
	}else
	{
		publishTransforms(t);
	}
}

bool optimize(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	gSolver->saveGraph("input_graph.g2o");
	bool success = gGraph->optimize();

	publishTransforms(gLastTime);
	publishGraph(gLastTime);
	return true;
}

void build_map(VertexObjectList vertices)
{
	PointCloud::Ptr downsampled = gPclSensor->buildMap(vertices);
	downsampled->header.frame_id = MAP_FRAME;
	gMapPublisher->publish(downsampled);
	
	pcl::io::savePCDFileASCII("slam3d_map.pcd", *downsampled);
}

bool show_map(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	// Get the accumulated point cloud
	VertexObjectList vertices = gGraph->getVerticesFromSensor(gPclSensor->getName());
	std::thread t(build_map, vertices);
	t.detach();
	return true;
}

bool write_graph(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	gGraph->writeGraphToFile("pose_graph");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper");
	ros::NodeHandle n;
	ros::NodeHandle pn("~/");
	gLastTime = ros::Time(0);
	gTransformBroadcaster = new tf::TransformBroadcaster;
	
	slam3d::Clock* clock = new RosClock();
	slam3d::Logger* logger = new RosLogger();
	
	gGraph = new slam3d::BoostGraph(logger);
	gMapper = new slam3d::Mapper(gGraph, logger);
	gSolver = new slam3d::G2oSolver(logger);

	int rate;
	pn.param("optimization_rate", rate, 10);
	gGraph->setSolver(gSolver, rate);
	
	// Create the PointCloudSensor for the velodyne laser
	gPclSensor = new slam3d::PointCloudSensor(SENSOR_NAME, logger);
	readPointcloudSensorParameters(pn, gPclSensor);
	gMapper->registerSensor(gPclSensor);
	
	pn.param("scan_resolution", gScanResolution, 0.5);
	
	gPatchSolver = new slam3d::G2oSolver(logger);
	gPclSensor->setPatchSolver(gPatchSolver);

	// Subscribe to topics
	ros::Publisher pclPub = n.advertise<slam3d::PointCloud>("map", 1);
	ros::Subscriber pclSub = n.subscribe<slam3d::PointCloud>("pointcloud", 10, &receivePointCloud);
	ros::ServiceServer optSrv = n.advertiseService("optimize", &optimize);
	ros::ServiceServer showSrv = n.advertiseService("show_map", &show_map);
	ros::ServiceServer writeSrv = n.advertiseService("write_graph", &write_graph);
	ros::Publisher signalPub = n.advertise<std_msgs::Empty>("trigger", 1, true);
	
	gMapPublisher = &pclPub;
	gSignalPublisher = &signalPub;

	gGraphPublisher = new GraphPublisher(gGraph);
	gGraphPublisher->addNodeSensor(SENSOR_NAME, 0,1,0);
	gGraphPublisher->addEdgeSensor(SENSOR_NAME);
	
	ROS_INFO("Mapper ready!");
	ROS_WARN(" - ScanResolution: %.2f", gScanResolution);
	
	ros::spin();
	
	delete gGraph;
	delete gGraphPublisher;
	delete gPclSensor;
	delete gSolver;
	delete gPatchSolver;
	delete logger;
	delete clock;
	delete gTransformBroadcaster;
	return 0;
}