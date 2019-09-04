#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <std_srvs/Empty.h>
#include <pcl_ros/point_cloud.h>

#include <boost/uuid/uuid_io.hpp>

#include <slam3d/core/Mapper.hpp>
#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d/sensor/pointmatcher/Scan2DSensor.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <slam3d/solver/g2o/G2oSolver.hpp>

#include <iostream>

#include "GraphPublisher.hpp"
#include "ros_common.hpp"
#include "ros_tf.hpp"
#include "helper_functions.hpp"

tf::TransformBroadcaster* gTransformBroadcaster;
tf::TransformListener* gTransformListener;

ros::Publisher* gMapPublisher;
ros::Publisher* gCloudPublisher;
ros::ServiceServer* gOptimizeService;
ros::ServiceServer* gShowMapService;
ros::ServiceServer* gWriteGraphService;
GraphPublisher* gGraphPublisher;

slam3d::Logger* gLogger;
slam3d::BoostGraph* gGraph;
slam3d::Mapper* gMapper;
slam3d::Scan2DSensor* gScan2DSensor;
slam3d::PointCloudSensor* gPclSensor;
slam3d::G2oSolver* gSolver;
slam3d::G2oSolver* gPatchSolver;

std::string gRobotName;
std::string gLaserName;
std::string gCameraName;

std::string gOdometryFrame;
std::string gRobotFrame;
std::string gMapFrame;
std::string gLaserFrame;
std::string gCameraFrame;

double gScanResolution;
double gMapOutlierRadius;
int gMapOutlierNeighbors;

slam3d::Transform gSensorPose;
Odometry* gOdometry;
tf::StampedTransform gOdomInMap;

typedef slam3d::PM::DataPoints::Label Label;
typedef slam3d::PM::DataPoints::Labels Labels;

using namespace slam3d;

bool optimize(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	gSolver->saveGraph("input_graph.g2o");
	return gGraph->optimize();
}

typedef Eigen::Matrix<ScalarType,4,1> Point;

bool show_map(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	// Show 2D map
	sensor_msgs::PointCloud pc_msg;
	pc_msg.header.stamp = ros::Time::now();
	pc_msg.header.frame_id = gMapFrame;
	
	
	VertexObjectList vertices = gGraph->getVerticesFromSensor(gLaserName);
	for(VertexObjectList::iterator v = vertices.begin(); v != vertices.end(); v++)
	{
		Point point;
		point(2) = 0; // z = 0
		point(3) = 1; // homogenous vector
		Scan2DMeasurement::Ptr scan = boost::dynamic_pointer_cast<Scan2DMeasurement>(v->measurement);
		assert(scan);
		
		Transform pose = v->corrected_pose * v->measurement->getSensorPose();
		
		// Transform to map and write into ROS-Msg
		unsigned numPoints = scan->getDataPoints().features.cols();
		for(unsigned p = 0; p < numPoints; p++)
		{
			point(0) = scan->getDataPoints().features(0,p);
			point(1) = scan->getDataPoints().features(1,p);
			Point point_tf = pose * point;
			
			geometry_msgs::Point32 p_msg;
			p_msg.x = point_tf(0);
			p_msg.y = point_tf(1);
			p_msg.z = point_tf(2);
			pc_msg.points.push_back(p_msg);
		}
	}
	gMapPublisher->publish(pc_msg);
	
	// Show 3D map
	PointCloud::Ptr downsampled = gPclSensor->buildMap(gGraph->getVerticesFromSensor(gPclSensor->getName()));
	gLogger->message(INFO, (boost::format("Map cloud has %1% points.") % downsampled->size()).str());
	downsampled->header.frame_id = gMapFrame;
	gCloudPublisher->publish(downsampled);
	
	return true;
}

bool write_graph(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	gGraph->writeGraphToFile("pose_graph");
	return true;
}

void receiveScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	Labels feat_labels;
	feat_labels.push_back(Label("x", 1));
	feat_labels.push_back(Label("y", 1));
	feat_labels.push_back(Label("w", 1));
	
	laser_geometry::LaserProjection projection;
	sensor_msgs::PointCloud cloud;
	projection.projectLaser(*scan, cloud);
	PM::Matrix feat_points(3, cloud.points.size());
	unsigned i = 0;
	for(auto it = cloud.points.begin(); it < cloud.points.end(); it++)
	{
		feat_points(0, i) = it->x;
		feat_points(1, i) = it->y;
		feat_points(2, i) = 1;
		i++;
	}
	
	PM::DataPoints dp(feat_points, feat_labels);
	ros::Time rostime = scan->header.stamp;
	timeval stamp;
	stamp.tv_sec  = rostime.sec;
	stamp.tv_usec = rostime.nsec / 1000;
	
	// Get the pose of the laser scanner
	tf::StampedTransform laser_pose;
	try
	{
		gTransformListener->waitForTransform(gRobotFrame, gLaserFrame, rostime, ros::Duration(0.1));
		gTransformListener->lookupTransform(gRobotFrame, gLaserFrame, rostime, laser_pose);
	}catch(tf2::TransformException &e)
	{
		ROS_WARN("Could not get transform: '%s' => '%s'!", gRobotFrame.c_str(), gLaserFrame.c_str());
		return;
	}
	
	Scan2DMeasurement::Ptr m(new Scan2DMeasurement(dp, stamp, gRobotName, gLaserName, tf2eigen(laser_pose)));
	
	bool added;
	try
	{
		added = gScan2DSensor->addMeasurement(m, gOdometry->getPose(stamp));
	}catch(std::exception &e)
	{
		ROS_ERROR("Could not add scan: %s", e.what());
		return;
	}
	
	slam3d::Transform current = gMapper->getCurrentPose();
	if(current.matrix().determinant() == 0)
	{
		ROS_ERROR("Current pose from mapper has 0 determinant!");
	}
	
	// Publish the transformations
	if(added)
	{
		tf::Stamped<tf::Pose> map_in_robot(eigen2tf(current.inverse()), rostime, gRobotFrame);
		tf::Stamped<tf::Pose> map_in_odom;
		try
		{
			gTransformListener->waitForTransform(gRobotFrame, gOdometryFrame, rostime, ros::Duration(0.1));
			gTransformListener->transformPose(gOdometryFrame, map_in_robot, map_in_odom);
			gOdomInMap = tf::StampedTransform(map_in_odom.inverse(), rostime, gMapFrame, gOdometryFrame);
		}catch(tf2::TransformException &e)
		{
			ROS_ERROR("%s", e.what());
		}
	}
	gOdomInMap.stamp_ = rostime;
	gTransformBroadcaster->sendTransform(gOdomInMap);
	
	// Publish the graph
	if(added)
	{
		gGraphPublisher->publishNodes(scan->header.stamp, gMapFrame);
		gGraphPublisher->publishEdges(gScan2DSensor->getName(), scan->header.stamp, gMapFrame);
		gGraphPublisher->publishEdges(gPclSensor->getName(), scan->header.stamp, gMapFrame);
	}
}

void receivePointCloud(const slam3d::PointCloud::ConstPtr& pcl)
{
	// Get the pose of the laser scanner
	ros::Time t;
	t.fromNSec(pcl->header.stamp * 1000);
	tf::StampedTransform laser_pose;
	try
	{
		gTransformListener->waitForTransform(gRobotFrame, gCameraFrame, t, ros::Duration(0.1));
		gTransformListener->lookupTransform(gRobotFrame, gCameraFrame, t, laser_pose);
	}catch(tf2::TransformException &e)
	{
		ROS_WARN("Could not get transform: '%s' => '%s'!", gRobotFrame.c_str(), gLaserFrame.c_str());
		return;
	}

	slam3d::PointCloud::Ptr cloud;
	if(gScanResolution > 0)
	{
		cloud = gPclSensor->downsample(pcl, gScanResolution);
	}else
	{	
		cloud = slam3d::PointCloud::Ptr(new slam3d::PointCloud(*pcl));
	}
	
	bool added;
	slam3d::PointCloudMeasurement::Ptr m(
		new slam3d::PointCloudMeasurement(cloud, gRobotName, gCameraName, tf2eigen(laser_pose)));
	try
	{
		added = gPclSensor->addMeasurement(m, gOdometry->getPose(m->getTimestamp()));
	}catch(std::exception &e)
	{
		ROS_ERROR("Could not add new pointcloud measurement: %s", e.what());
		return;
	}
	
	if(added)
	{
		gPclSensor->linkLastToNeighbors(true);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper2d");
	ros::NodeHandle n;
	ros::NodeHandle pn("~/");
	
	gTransformBroadcaster = new tf::TransformBroadcaster;
	gTransformListener = new tf::TransformListener();
	
	gLogger = new RosLogger();
	gLogger->setLogLevel(DEBUG);
	
	gGraph = new slam3d::BoostGraph(gLogger);
	gMapper = new slam3d::Mapper(gGraph, gLogger);
	gSolver = new slam3d::G2oSolver(gLogger);
	gGraph->setSolver(gSolver);
	
	pn.param("robot_name", gRobotName, std::string("Robot"));
	pn.param("odometry_frame", gOdometryFrame, std::string("odometry"));
	pn.param("robot_frame", gRobotFrame, std::string("robot"));
	pn.param("map_frame", gMapFrame, std::string("map"));
	pn.param("laser_frame", gLaserFrame, std::string("laser"));
	pn.param("camera_frame", gCameraFrame, std::string("camera"));

	// Apply tf-prefix to all frames
	gRobotFrame = gTransformListener->resolve(gRobotFrame);
	gOdometryFrame = gTransformListener->resolve(gOdometryFrame);
	gMapFrame = gTransformListener->resolve(gMapFrame);
	gLaserFrame = gTransformListener->resolve(gLaserFrame);
	
	// Create the ScanSensor for the 2d laser
	std::string icp_config;
	pn.param("laser_name", gLaserName, std::string("LineScanner"));
	pn.param("icp_config_file", icp_config, std::string(""));
	gScan2DSensor = new slam3d::Scan2DSensor(gLaserName, gLogger, icp_config);
	readScanSensorParameters(pn, gScan2DSensor);
	gScan2DSensor->writeDebugData();
	gMapper->registerSensor(gScan2DSensor);

	// Create the PointcloudSensor
	pn.param("camera_name", gCameraName, std::string("DepthCam"));
	gPclSensor = new PointCloudSensor(gCameraName, gLogger);
	readPointcloudSensorParameters(pn, gPclSensor);
	
	pn.param("scan_resolution", gScanResolution, 0.5);
	
	gPatchSolver = new G2oSolver(gLogger);
	gPclSensor->setPatchSolver(gPatchSolver);

	gMapper->registerSensor(gPclSensor);
	gGraph->fixNext();

	// Create the Odometry
	gOdometry = new Odometry(gGraph, gLogger);
	gOdometry->setTF(gTransformListener, gOdometryFrame, gRobotFrame);
	gMapper->registerPoseSensor(gOdometry);

	// Subscribe to point cloud
	ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan>("scan", 10, &receiveScan);
	ros::Subscriber pclSub = n.subscribe<slam3d::PointCloud>("pointcloud", 10, &receivePointCloud);
	ros::ServiceServer optSrv = n.advertiseService("optimize", &optimize);
	ros::ServiceServer showSrv = n.advertiseService("show_map", &show_map);
	ros::ServiceServer writeSrv = n.advertiseService("write_graph", &write_graph);
	ros::Publisher pclPub = n.advertise<sensor_msgs::PointCloud>("map", 1);
	ros::Publisher cloudPub = n.advertise<slam3d::PointCloud>("map_cloud", 1);
	
	gOptimizeService = &optSrv;
	gShowMapService = &showSrv;
	gWriteGraphService = &writeSrv;
	gMapPublisher = &pclPub;
	gCloudPublisher = &cloudPub;
	gGraphPublisher = new GraphPublisher(gGraph);
	gGraphPublisher->addNodeSensor(gLaserName, 0,0.8,0);
	gGraphPublisher->addNodeSensor(gCameraName, 0, 0, 0.8);
	gGraphPublisher->addEdgeSensor(gOdometry->getName());
	gGraphPublisher->addEdgeSensor(gLaserName);
	gGraphPublisher->addEdgeSensor(gCameraName);

	gLogger->message(INFO, "Mapper2D is ready!");
	
	ros::spin();
	
	delete gMapper;
	delete gGraph;
	delete gGraphPublisher;
	delete gScan2DSensor;
	delete gPatchSolver;
	delete gSolver;
	delete gLogger;
	delete gTransformBroadcaster;
	delete gTransformListener;
	return 0;
}
