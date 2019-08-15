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
ros::Publisher* gMapPublisher;
ros::Publisher* gSignalPublisher;
GraphPublisher* gGraphPublisher;

slam3d::BoostGraph* gGraph;
slam3d::Mapper* gMapper;
slam3d::PointCloudSensor* gPclSensor;
slam3d::G2oSolver* gSolver;
slam3d::G2oSolver* gPatchSolver;

double gScanResolution;
bool gMultiThreaded;

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

	ros::Time t;
	t.fromNSec(pcl->header.stamp * 1000);

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
		gPclSensor->linkLastToNeighbors(gMultiThreaded);
		publishTransforms(t);
		publishGraph(t);
	}
}

bool optimize(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	bool success = gGraph->optimize();
	return true;
}

void build_map(VertexObjectList vertices)
{
	PointCloud::Ptr downsampled = gPclSensor->buildMap(vertices);
	downsampled->header.frame_id = MAP_FRAME;
	gMapPublisher->publish(downsampled);
}

bool show_map(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	// Get the accumulated point cloud
	VertexObjectList vertices = gGraph->getVerticesFromSensor(gPclSensor->getName());
	std::thread t(build_map, vertices);
	t.detach();
	return true;
}

int main(int argc, char **argv)
{
	// Create the ROS node
	ros::init(argc, argv, "mapper");
	ros::NodeHandle n;
	ros::NodeHandle pn("~/");
	pn.param("scan_resolution", gScanResolution, 0.5);
	pn.param("multi_threaded", gMultiThreaded, true);
	gTransformBroadcaster = new tf::TransformBroadcaster;
	
	// Clock and Logger to be used by all slam3d components
	slam3d::Clock* clock = new RosClock();
	slam3d::Logger* logger = new RosLogger();
	
	// Create the Graph, that will hold all mapping information
	gGraph = new slam3d::BoostGraph(logger);
	
	// We have no absolute positioning (e.g. GPS), so we fix the 
	// first node in the graph
	gGraph->fixNext();

	// Create the Mapper object
	gMapper = new slam3d::Mapper(gGraph, logger);

	// Create the Backend for global graph optimization
	gSolver = new slam3d::G2oSolver(logger);
	gGraph->setSolver(gSolver, pn.param("optimization_rate", 10));
	
	// Create the PointCloudSensor for the velodyne laser
	gPclSensor = new slam3d::PointCloudSensor(SENSOR_NAME, logger);
	readPointcloudSensorParameters(pn, gPclSensor);
	gMapper->registerSensor(gPclSensor);
	
	// Create a solver to optimize local scan patches
	gPatchSolver = new slam3d::G2oSolver(logger);
	gPclSensor->setPatchSolver(gPatchSolver);

	// Subscribe to topics
	ros::Publisher pclPub = n.advertise<slam3d::PointCloud>("map", 1);
	ros::Subscriber pclSub = n.subscribe<slam3d::PointCloud>("pointcloud", 10, &receivePointCloud);
	ros::ServiceServer optSrv = n.advertiseService("optimize", &optimize);
	ros::ServiceServer showSrv = n.advertiseService("show_map", &show_map);
	ros::Publisher signalPub = n.advertise<std_msgs::Empty>("trigger", 1, true);
	
	gMapPublisher = &pclPub;
	gSignalPublisher = &signalPub;

	// Create the visualizer for the nodes and edges
	gGraphPublisher = new GraphPublisher(gGraph);
	gGraphPublisher->addNodeSensor(SENSOR_NAME, 0,1,0);
	gGraphPublisher->addEdgeSensor(SENSOR_NAME);
	
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