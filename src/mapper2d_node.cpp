#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <std_srvs/Empty.h>
#include <boost/uuid/uuid_io.hpp>

#include <slam3d/core/Mapper.hpp>
#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d/sensor/pointmatcher/Scan2DSensor.hpp>
#include <slam3d/solver/g2o/G2oSolver.hpp>

#include <iostream>

#include "GraphPublisher.hpp"
#include "ros_common.hpp"

tf::TransformBroadcaster* gTransformBroadcaster;
tf::TransformListener* gTransformListener;

ros::Publisher* gMapPublisher;
ros::ServiceServer* gOptimizeService;
ros::ServiceServer* gShowMapService;
ros::ServiceServer* gWriteGraphService;
GraphPublisher* gGraphPublisher;

slam3d::Logger* gLogger;
slam3d::BoostGraph* gGraph;
slam3d::Mapper* gMapper;
slam3d::Scan2DSensor* gScan2DSensor;
slam3d::G2oSolver* gSolver;
slam3d::G2oSolver* gPatchSolver;

std::string gRobotName;
std::string gSensorName;

std::string gOdometryFrame;
std::string gRobotFrame;
std::string gMapFrame;
std::string gLaserFrame;

slam3d::Transform gSensorPose;
RosTfOdometry* gOdometry;
tf::StampedTransform gOdomInMap;

typedef slam3d::PM::DataPoints::Label Label;
typedef slam3d::PM::DataPoints::Labels Labels;

using namespace slam3d;

bool optimize(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	gSolver->saveGraph("input_graph.g2o");
	return gGraph->optimize();
}

bool show_map(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	sensor_msgs::PointCloud pc_msg;
	pc_msg.header.stamp = ros::Time::now();
	pc_msg.header.frame_id = gMapFrame;
	
	geometry_msgs::Point32 p_msg;
	p_msg.z = 0;
	VertexObjectList vertices = gGraph->getVerticesFromSensor(gSensorName);
	for(VertexObjectList::iterator v = vertices.begin(); v != vertices.end(); v++)
	{
		Scan2DMeasurement::Ptr scan = boost::dynamic_pointer_cast<Scan2DMeasurement>(v->measurement);
		assert(scan);
		
		Transform pose = v->corrected_pose * v->measurement->getSensorPose();
		PM::DataPoints dp_in_map = gScan2DSensor->transformDataPoints(scan->getDataPoints(), pose);
		
		// Write into ROS-Msg
		unsigned numPoints = dp_in_map.features.cols();
		for(unsigned p = 0; p < numPoints; p++)
		{
			p_msg.x = dp_in_map.features(0,p);
			p_msg.y = dp_in_map.features(1,p);
			pc_msg.points.push_back(p_msg);
		}
		
		gMapPublisher->publish(pc_msg);
	}
	
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
	
	Scan2DMeasurement::Ptr m(new Scan2DMeasurement(dp, stamp, gRobotName, gSensorName, tf2eigen(laser_pose)));
	
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
		gGraphPublisher->publishGraph(scan->header.stamp, gMapFrame);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper2d");
	ros::NodeHandle n;
	
	gTransformBroadcaster = new tf::TransformBroadcaster;
	gTransformListener = new tf::TransformListener();
	
	gLogger = new RosLogger();
	gLogger->setLogLevel(DEBUG);
	
	gGraph = new slam3d::BoostGraph(gLogger);
	gMapper = new slam3d::Mapper(gGraph, gLogger);
	gSolver = new slam3d::G2oSolver(gLogger);
	gGraph->setSolver(gSolver);
	
	n.param("robot_name", gRobotName, std::string("Robot"));
	n.param("odometry_frame", gOdometryFrame, std::string("odometry"));
	n.param("robot_frame", gRobotFrame, std::string("robot"));
	n.param("map_frame", gMapFrame, std::string("map"));
	n.param("laser_frame", gLaserFrame, std::string("velodyne_laser"));

	// Apply tf-prefix to all frames
	gRobotFrame = gTransformListener->resolve(gRobotFrame);
	gOdometryFrame = gTransformListener->resolve(gOdometryFrame);
	gMapFrame = gTransformListener->resolve(gMapFrame);
	gLaserFrame = gTransformListener->resolve(gLaserFrame);
	
	// Create the ScanSensor for the 2d laser
	std::string icp_config;
	n.param("sensor_name", gSensorName, std::string("ScanSensor"));
	n.param("icp_config_file", icp_config, std::string(""));
	gScan2DSensor = new slam3d::Scan2DSensor(gSensorName, gLogger, icp_config);
	gScan2DSensor->writeDebugData();
	gMapper->registerSensor(gScan2DSensor);
	
	double translation;
	double rotation;
	n.param("min_translation", translation, 0.5);
	n.param("min_rotation", rotation, 0.1);
	gScan2DSensor->setMinPoseDistance(translation, rotation);

	int range;
	n.param("patch_building_range", range, 5);
	gPatchSolver = new G2oSolver(gLogger);
	gScan2DSensor->setPatchBuildingRange(range);
	gScan2DSensor->setPatchSolver(gPatchSolver);

	double radius;
	int links;
	n.param("neighbor_radius", radius, 5.0);
	n.param("max_neighbor_links", links, 5);
	gScan2DSensor->setNeighborRadius(radius, links);

	gOdometry = new RosTfOdometry(gGraph, gLogger, n);
	gMapper->registerPoseSensor(gOdometry);

	// Subscribe to point cloud
	ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan>("scan", 10, &receiveScan);
	ros::ServiceServer optSrv = n.advertiseService("optimize", &optimize);
	ros::ServiceServer showSrv = n.advertiseService("show_map", &show_map);
	ros::ServiceServer writeSrv = n.advertiseService("write_graph", &write_graph);
	ros::Publisher pclPub = n.advertise<sensor_msgs::PointCloud>("map", 1);
	
	gOptimizeService = &optSrv;
	gShowMapService = &showSrv;
	gWriteGraphService = &writeSrv;
	gMapPublisher = &pclPub;
	gGraphPublisher = new GraphPublisher(gGraph, gSensorName);

	gLogger->message(INFO, "Mapper2D is ready!");
	
	ros::spin();
	
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
