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

tf::TransformBroadcaster* gTransformBroadcaster;
tf::TransformListener* gTransformListener;

ros::Publisher* gMapPublisher;
ros::ServiceServer* gOptimizeService;
ros::ServiceServer* gShowMapService;
ros::ServiceServer* gWriteGraphService;
ros::Publisher* gSignalPublisher;
GraphPublisher* gGraphPublisher;
GpsPublisher* gGpsPublisher;


slam3d::BoostGraph* gGraph;
slam3d::Mapper* gMapper;
slam3d::PointCloudSensor* gPclSensor;
slam3d::GpsSensor* gGpsSensor;
slam3d::G2oSolver* gSolver;
slam3d::G2oSolver* gPatchSolver;

tf::StampedTransform gOdomInMap;
Odometry* gOdometry;
IMU* gIMU;

int gCount;
double gMapResolution;
double gScanResolution;
bool gUseOdometry;
double gMapOutlierRadius;
int gMapOutlierNeighbors;

std::string gRobotName;
std::string gSensorName;
std::string gGpsName;

std::string gOdometryFrame;
std::string gRobotFrame;
std::string gMapFrame;
std::string gLaserFrame;
std::string gGpsFrame;

ros::Time gLastTime;

bool checkOdometry(const ros::Time& t)
{
	if(gUseOdometry)
	{
		try
		{
			gTransformListener->waitForTransform(gOdometryFrame, gRobotFrame, t, ros::Duration(0.1));
			if(!gTransformListener->canTransform(gRobotFrame, gOdometryFrame, t))
				return false;
		}catch(tf2::TransformException &e)
		{
			ROS_WARN("Waiting for transform: '%s' => '%s' to become available...", gOdometryFrame.c_str(), gRobotFrame.c_str());
			return false;
		}
	}
	return true;
}

void receiveGPS(const sensor_msgs::NavSatFix::ConstPtr& gps)
{
	if(gps->header.stamp < gLastTime)
	{
		ROS_WARN("Received late GPS sample!");
	}
	gLastTime = gps->header.stamp;
	
	// Get the pose of the laser scanner
	tf::StampedTransform gps_pose;
	try
	{
		gTransformListener->waitForTransform(gRobotFrame, gGpsFrame, gps->header.stamp, ros::Duration(0.01));
		gTransformListener->lookupTransform(gRobotFrame, gGpsFrame, gps->header.stamp, gps_pose);
	}catch(tf2::TransformException &e)
	{
		ROS_WARN("Could not get transform: '%s' => '%s'!", gRobotFrame.c_str(), gGpsFrame.c_str());
		return;
	}
	
	if(!checkOdometry(gps->header.stamp))
		return;
	
	Position pos = gGpsSensor->toUTM(gps->longitude, gps->latitude, gps->altitude);
	Covariance<3> cov = Covariance<3>::Identity(); //TODO: Read covariance from message
	timeval t = fromRosTime(gps->header.stamp);
	GpsMeasurement::Ptr m(new GpsMeasurement(pos, cov, t, gRobotName, gGpsName, tf2eigen(gps_pose)));
//	try
//	{
		gGpsSensor->addMeasurement(m);
//	}catch(std::exception &e)
//	{
//		ROS_ERROR("Could not add new gps measurement: %s", e.what());
//		return;
//	}
//	gGraphPublisher->publishGraph(gps->header.stamp, gMapFrame);
	gGpsPublisher->publishPoints(gGpsName, gMapFrame);
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
	}
	gLastTime = t;
	
	tf::StampedTransform laser_pose;
	try
	{
		gTransformListener->waitForTransform(gRobotFrame, gLaserFrame, t, ros::Duration(0.1));
		gTransformListener->lookupTransform(gRobotFrame, gLaserFrame, t, laser_pose);
	}catch(tf2::TransformException &e)
	{
		ROS_WARN("Could not get transform: '%s' => '%s'!", gRobotFrame.c_str(), gLaserFrame.c_str());
		return;
	}

	if(!checkOdometry(t))
		return;

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
		new slam3d::PointCloudMeasurement(cloud, gRobotName, gSensorName, tf2eigen(laser_pose)));
	try
	{
		if(gUseOdometry)
		{
			added = gPclSensor->addMeasurement(m, gOdometry->getPose(m->getTimestamp()), true);
		}else
		{
			added = gPclSensor->addMeasurement(m);
		}
	}catch(std::exception &e)
	{
		ROS_ERROR("Could not add new measurement: %s", e.what());
		return;
	}
	slam3d::Transform current = gMapper->getCurrentPose();
	if(current.matrix().determinant() == 0)
	{
		ROS_ERROR("Current pose from mapper has 0 determinant!");
	}
	
	if(gUseOdometry)
	{
		if(added)
		{
			tf::Stamped<tf::Pose> map_in_robot(eigen2tf(current.inverse()), t, gRobotFrame);
			tf::Stamped<tf::Pose> map_in_odom;
			try
			{
				gTransformListener->waitForTransform(gRobotFrame, gOdometryFrame, t, ros::Duration(0.1));
				gTransformListener->transformPose(gOdometryFrame, map_in_robot, map_in_odom);
				gOdomInMap = tf::StampedTransform(map_in_odom.inverse(), t, gMapFrame, gOdometryFrame);
			}catch(tf2::TransformException &e)
			{
				ROS_ERROR("%s", e.what());
			}
		}
		gOdomInMap.stamp_ = t;
		gTransformBroadcaster->sendTransform(gOdomInMap);
	}else
	{
		tf::StampedTransform robot_in_map(eigen2tf(current), t, gMapFrame, gRobotFrame);
		gTransformBroadcaster->sendTransform(robot_in_map);
	}
	
	// Show the graph in RVIZ
	gGraphPublisher->publishGraph(t, gMapFrame);
//	gPclSensor->linkLastToNeighbors(true);
//	tf::StampedTransform gps_origin(eigen2tf(gGpsSensor->getOrigin()), t, gMapFrame, "gps");
//	gTransformBroadcaster->sendTransform(gps_origin);
}

bool optimize(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	gSolver->saveGraph("input_graph.g2o");
	bool success = gGraph->optimize();
	return true;
}

void build_map(VertexObjectList vertices)
{
	PointCloud::Ptr accu = gPclSensor->getAccumulatedCloud(vertices);
	PointCloud::Ptr cleaned = gPclSensor->removeOutliers(accu, gMapOutlierRadius, gMapOutlierNeighbors);
	PointCloud::Ptr downsampled = gPclSensor->downsample(cleaned, gMapResolution);
	downsampled->header.frame_id = gMapFrame;
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

bool write_graph(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	gGraph->writeGraphToFile("pose_graph");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper");
	ros::NodeHandle n;
	
	gCount = 0;
	gLastTime = ros::Time(0);
	
	gTransformBroadcaster = new tf::TransformBroadcaster;
	gTransformListener = new tf::TransformListener();
	
	slam3d::Clock* clock = new RosClock();
	slam3d::Logger* logger = new RosLogger();
	
	gGraph = new slam3d::BoostGraph(logger);
	gMapper = new slam3d::Mapper(gGraph, logger);
	gSolver = new slam3d::G2oSolver(logger);

	int rate;
	n.param("optimization_rate", rate, 10);
	gGraph->setSolver(gSolver, rate);
	
	n.param("robot_name", gRobotName, std::string("Robot"));
	n.param("odometry_frame", gOdometryFrame, std::string("odometry"));
	n.param("robot_frame", gRobotFrame, std::string("robot"));
	n.param("map_frame", gMapFrame, std::string("map"));
	n.param("laser_frame", gLaserFrame, std::string("laser"));
	n.param("gps_frame", gGpsFrame, std::string("gps"));

	// Apply tf-prefix to all frames
	gRobotFrame = gTransformListener->resolve(gRobotFrame);
	gOdometryFrame = gTransformListener->resolve(gOdometryFrame);
	gMapFrame = gTransformListener->resolve(gMapFrame);
	gLaserFrame = gTransformListener->resolve(gLaserFrame);
	gGpsFrame = gTransformListener->resolve(gGpsFrame);
	
	// Create GpsSensor
	n.param("gps_name", gGpsName, std::string("GpsSensor"));
	gGpsSensor = new slam3d::GpsSensor(gGpsName, logger);
	gGpsSensor->initCoordTransform();
	gMapper->registerSensor(gGpsSensor);
	
	// Create the PointCloudSensor for the velodyne laser
	n.param("sensor_name", gSensorName, std::string("PointCloudSensor"));
	gPclSensor = new slam3d::PointCloudSensor(gSensorName, logger);

	slam3d::GICPConfiguration gicp_conf;
	n.param("icp_fine/correspondence_randomness", gicp_conf.correspondence_randomness, gicp_conf.correspondence_randomness);
	n.param("icp_fine/euclidean_fitness_epsilon", gicp_conf.euclidean_fitness_epsilon, gicp_conf.euclidean_fitness_epsilon);
	n.param("icp_fine/max_correspondence_distance", gicp_conf.max_correspondence_distance, gicp_conf.max_correspondence_distance);
	n.param("icp_fine/max_fitness_score", gicp_conf.max_fitness_score, gicp_conf.max_fitness_score);
	n.param("icp_fine/max_sensor_distance", gicp_conf.max_sensor_distance, gicp_conf.max_sensor_distance);
	n.param("icp_fine/maximum_iterations", gicp_conf.maximum_iterations, gicp_conf.maximum_iterations);
	n.param("icp_fine/maximum_optimizer_iterations", gicp_conf.maximum_optimizer_iterations, gicp_conf.maximum_optimizer_iterations);
	n.param("icp_fine/orientation_sigma", gicp_conf.orientation_sigma, gicp_conf.orientation_sigma);
	n.param("icp_fine/point_cloud_density", gicp_conf.point_cloud_density, gicp_conf.point_cloud_density);
	n.param("icp_fine/position_sigma", gicp_conf.position_sigma, gicp_conf.position_sigma);
	n.param("icp_fine/rotation_epsilon", gicp_conf.rotation_epsilon, gicp_conf.rotation_epsilon);
	n.param("icp_fine/transformation_epsilon", gicp_conf.transformation_epsilon, gicp_conf.transformation_epsilon);
	gPclSensor->setFineConfiguaration(gicp_conf);
	
	n.param("icp_coarse/correspondence_randomness", gicp_conf.correspondence_randomness, gicp_conf.correspondence_randomness);
	n.param("icp_coarse/euclidean_fitness_epsilon", gicp_conf.euclidean_fitness_epsilon, gicp_conf.euclidean_fitness_epsilon);
	n.param("icp_coarse/max_correspondence_distance", gicp_conf.max_correspondence_distance, gicp_conf.max_correspondence_distance);
	n.param("icp_coarse/max_fitness_score", gicp_conf.max_fitness_score, gicp_conf.max_fitness_score);
	n.param("icp_coarse/max_sensor_distance", gicp_conf.max_sensor_distance, gicp_conf.max_sensor_distance);
	n.param("icp_coarse/maximum_iterations", gicp_conf.maximum_iterations, gicp_conf.maximum_iterations);
	n.param("icp_coarse/maximum_optimizer_iterations", gicp_conf.maximum_optimizer_iterations, gicp_conf.maximum_optimizer_iterations);
	n.param("icp_coarse/orientation_sigma", gicp_conf.orientation_sigma, gicp_conf.orientation_sigma);
	n.param("icp_coarse/point_cloud_density", gicp_conf.point_cloud_density, gicp_conf.point_cloud_density);
	n.param("icp_coarse/position_sigma", gicp_conf.position_sigma, gicp_conf.position_sigma);
	n.param("icp_coarse/rotation_epsilon", gicp_conf.rotation_epsilon, gicp_conf.rotation_epsilon);
	n.param("icp_coarse/transformation_epsilon", gicp_conf.transformation_epsilon, gicp_conf.transformation_epsilon);
	gPclSensor->setCoarseConfiguaration(gicp_conf);

	int loop_len;
	n.param("min_loop_length", loop_len, 1);
	gPclSensor->setMinLoopLength(loop_len);

	gMapper->registerSensor(gPclSensor);
	
	double radius, translation, rotation;
	int links, range;

	n.param("scan_resolution", gScanResolution, 0.5);
	n.param("map_resolution", gMapResolution, 0.5);
	n.param("neighbor_radius", radius, 5.0);
	n.param("max_neighbor_links", links, 5);
	n.param("patch_building_range", range, 5);
	n.param("map_outlier_radius", gMapOutlierRadius, 0.2);
	n.param("map_outlier_neighbors", gMapOutlierNeighbors, 2);
	n.param("min_translation", translation, 0.5);
	n.param("min_rotation", rotation, 0.1);
	n.param("use_odometry", gUseOdometry, false);

	gPclSensor->setNeighborRadius(radius, links);
	gPclSensor->setMinPoseDistance(translation, rotation);
	gPclSensor->setPatchBuildingRange(range);

	gPatchSolver = new slam3d::G2oSolver(logger);
	gPclSensor->setPatchSolver(gPatchSolver);

	if(gUseOdometry)
	{
		gOdometry = new Odometry(gGraph, logger);
		gOdometry->setTF(gTransformListener, gOdometryFrame, gRobotFrame);
		gMapper->registerPoseSensor(gOdometry);
		
		gIMU = new IMU(gGraph, logger);
		gIMU->setTF(gTransformListener, gOdometryFrame, gRobotFrame);
		gMapper->registerPoseSensor(gIMU);
	}else
	{
		gOdometry = NULL;
		gIMU = NULL;
	}

	// Subscribe to point cloud
	ros::Publisher pclPub = n.advertise<slam3d::PointCloud>("map", 1);
	ros::Subscriber pclSub = n.subscribe<slam3d::PointCloud>("pointcloud", 10, &receivePointCloud);
	ros::Subscriber gpsSub = n.subscribe<sensor_msgs::NavSatFix>("gps", 10, &receiveGPS);
	ros::ServiceServer optSrv = n.advertiseService("optimize", &optimize);
	ros::ServiceServer showSrv = n.advertiseService("show_map", &show_map);
	ros::ServiceServer writeSrv = n.advertiseService("write_graph", &write_graph);
	ros::Publisher signalPub = n.advertise<std_msgs::Empty>("trigger", 1, true);
	
	gMapPublisher = &pclPub;
	gOptimizeService = &optSrv;
	gShowMapService = &showSrv;
	gWriteGraphService = &writeSrv;
	gSignalPublisher = &signalPub;

	gGraphPublisher = new GraphPublisher(gGraph);
	gGraphPublisher->addNodeSensor(gSensorName, 0,1,0);
	gGraphPublisher->addNodeSensor(gGpsName, 1,1,0);
	gGraphPublisher->addEdgeSensor(gSensorName);
	gGraphPublisher->addEdgeSensor(gOdometry->getName());

	gGpsPublisher = new GpsPublisher(gGraph);

	ROS_INFO("Mapper ready!");
	
	ros::spin();
	
	delete gGraph;
	delete gGraphPublisher;
	delete gGpsPublisher;
	delete gPclSensor;
	delete gSolver;
	delete gPatchSolver;
	delete logger;
	delete clock;
	delete gTransformBroadcaster;
	delete gTransformListener;
	return 0;
}
