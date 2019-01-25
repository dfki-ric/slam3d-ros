#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
#include <boost/uuid/uuid_io.hpp>

#include <slam3d/core/Mapper.hpp>
#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <slam3d/solver/g2o/G2oSolver.hpp>

#include <iostream>

#include "ros_common.hpp"

tf::TransformBroadcaster* gTransformBroadcaster;
tf::TransformListener* gTransformListener;

ros::Publisher* gMapPublisher;
ros::ServiceServer* gOptimizeService;
ros::ServiceServer* gShowMapService;
ros::ServiceServer* gWriteGraphService;
ros::Publisher* gPosePublisher;
ros::Publisher* gEdgePublisher;
ros::Publisher* gSignalPublisher;

slam3d::BoostGraph* gGraph;
slam3d::Mapper* gMapper;
slam3d::PointCloudSensor* gPclSensor;
slam3d::G2oSolver* gSolver;

RosTfOdometry* gOdometry;
tf::StampedTransform gOdomInMap;

int gCount;
double gMapResolution;
double gScanResolution;
bool gUseOdometry;
double gMapOutlierRadius;
int gMapOutlierNeighbors;

std::string gRobotName;
std::string gSensorName;

std::string gOdometryFrame;
std::string gRobotFrame;
std::string gMapFrame;
std::string gLaserFrame;

void publishNodes(const ros::Time& stamp, const std::string& frame)
{
	slam3d::VertexObjectList vertices = gGraph->getVerticesFromSensor(gPclSensor->getName());

	visualization_msgs::Marker marker;
	marker.header.frame_id = frame;
	marker.header.stamp = stamp;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.points.resize(vertices.size());
	
	unsigned i = 0;
	for(VertexObjectList::const_iterator it = vertices.begin(); it != vertices.end(); it++)
	{
		Transform pose = it->corrected_pose;
		marker.points[i].x = pose.translation()[0];
		marker.points[i].y = pose.translation()[1];
		marker.points[i].z = pose.translation()[2];
		i++;
	}
	gPosePublisher->publish(marker);
}

void publishEdges(const ros::Time& stamp, const std::string& frame)
{
	slam3d::EdgeObjectList edges = gGraph->getEdgesFromSensor(gPclSensor->getName());

	visualization_msgs::Marker marker;
	marker.header.frame_id = frame;
	marker.header.stamp = stamp;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.02;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.points.resize(edges.size() * 2);
	marker.colors.resize(edges.size() * 2);
	
	unsigned i = 0;
	for(EdgeObjectList::const_iterator edge = edges.begin(); edge != edges.end(); ++edge)
	{
		const VertexObject& source_obj = gGraph->getVertex(edge->source);
		Transform::ConstTranslationPart source_pose = source_obj.corrected_pose.translation();
		marker.points[2*i].x = source_pose[0];
		marker.points[2*i].y = source_pose[1];
		marker.points[2*i].z = source_pose[2];

		const VertexObject& target_obj = gGraph->getVertex(edge->target);
		Transform::ConstTranslationPart target_pose = target_obj.corrected_pose.translation();
		marker.points[2*i+1].x = target_pose[0];
		marker.points[2*i+1].y = target_pose[1];
		marker.points[2*i+1].z = target_pose[2];
		
		// Edges from PointcloudSensor should be of SE3 type
		assert(edge->constraint.getType() == SE3);
		SE3Constraint::Ptr se3 = boost::dynamic_pointer_cast<SE3Constraint>(edge->constraint);
		
		Transform diff_inv = target_obj.corrected_pose.inverse() * source_obj.corrected_pose;
		Transform error = diff_inv * se3->getRelativePose().transform;
		
		Transform::TranslationPart trans = error.translation();
		Eigen::AngleAxisd aa;
		aa.fromRotationMatrix(error.linear());
		double x = trans(0);
		double y = trans(1);
		double z = trans(2);
		double a = aa.angle();
		double scalar_error = std::min(((x*x) + (y*y) + (z*z) + (a*a))*10, 1.0);
		marker.colors[2*i].a = 1.0;
		marker.colors[2*i].r = scalar_error;
		marker.colors[2*i].g = 1.0 - scalar_error;
		marker.colors[2*i].b = 0;
		marker.colors[(2*i)+1] = marker.colors[2*i];
		i++;
	}
	gEdgePublisher->publish(marker);
}

void receivePointCloud(const slam3d::PointCloud::ConstPtr& pcl)
{
	ROS_DEBUG("Received scan");
	gSignalPublisher->publish(std_msgs::Empty());

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
		new slam3d::PointCloudMeasurement(cloud, gRobotName, gSensorName, gPclSensor->getSensorPose()));
	try
	{
		if(gUseOdometry)
		{
			added = gPclSensor->addMeasurement(m, gOdometry->getPose(m->getTimestamp()));
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
	
	ros::Time t;
	t.fromNSec(pcl->header.stamp * 1000);
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
	publishNodes(t, gMapFrame);
	publishEdges(t, gMapFrame);
}

bool optimize(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	gSolver->saveGraph("input_graph.g2o");
	bool success = gGraph->optimize();
	return true;
}

bool show_map(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	// Get the accumulated point cloud
	PointCloud::Ptr accu = gPclSensor->getAccumulatedCloud(gGraph->getVerticesFromSensor(gPclSensor->getName()));
	PointCloud::Ptr cleaned = gPclSensor->removeOutliers(accu, gMapOutlierRadius, gMapOutlierNeighbors);
	PointCloud::Ptr downsampled = gPclSensor->downsample(cleaned, gMapResolution);
	downsampled->header.frame_id = gMapFrame;
	gMapPublisher->publish(downsampled);
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
	
	gTransformBroadcaster = new tf::TransformBroadcaster;
	gTransformListener = new tf::TransformListener();
	
	slam3d::Clock* clock = new RosClock();
	slam3d::Logger* logger = new RosLogger();
	
	gGraph = new slam3d::BoostGraph(logger);
	gMapper = new slam3d::Mapper(gGraph, logger);
	gSolver = new slam3d::G2oSolver(logger);
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
	
	// Get the pose of the laser scanner
	tf::StampedTransform laser_pose;
	try
	{
		gTransformListener->waitForTransform(gRobotFrame, gLaserFrame, ros::Time(0), ros::Duration(3.0));
		gTransformListener->lookupTransform(gRobotFrame, gLaserFrame, ros::Time(0), laser_pose);
	}catch(tf2::TransformException &e)
	{
		ROS_WARN("Could not get transform: '%s' => '%s'!", gRobotFrame.c_str(), gLaserFrame.c_str());
		laser_pose.setIdentity();
	}
	
	// Create the PointCloudSensor for the velodyne laser
	n.param("sensor_name", gSensorName, std::string("PointCloudSensor"));
	gPclSensor = new slam3d::PointCloudSensor(gSensorName, logger, tf2eigen(laser_pose));

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

	if(gUseOdometry)
	{
		bool add_edges, use_heading;
		n.param("add_odometry_edges", add_edges, false);
		n.param("use_odometry_heading", use_heading, false);

		gOdometry = new RosTfOdometry(gGraph, logger, n);
		gMapper->registerPoseSensor(gOdometry);
	}else
	{
		gOdometry = NULL;
	}

	// Subscribe to point cloud
	ros::Publisher pclPub = n.advertise<slam3d::PointCloud>("map", 1);
	ros::Subscriber pclSub = n.subscribe<slam3d::PointCloud>("pointcloud", 10, &receivePointCloud);
	ros::ServiceServer optSrv = n.advertiseService("optimize", &optimize);
	ros::ServiceServer showSrv = n.advertiseService("show_map", &show_map);
	ros::ServiceServer writeSrv = n.advertiseService("write_graph", &write_graph);
	ros::Publisher posePub = n.advertise<visualization_msgs::Marker>("vertices", 1, true);
	ros::Publisher edgePub = n.advertise<visualization_msgs::Marker>("edges", 1, true);
	ros::Publisher signalPub = n.advertise<std_msgs::Empty>("trigger", 1, true);
	
	gMapPublisher = &pclPub;
	gOptimizeService = &optSrv;
	gShowMapService = &showSrv;
	gWriteGraphService = &writeSrv;
	gPosePublisher = &posePub;
	gEdgePublisher = &edgePub;
	gSignalPublisher = &signalPub;

	ROS_INFO("Mapper ready!");
	
	ros::spin();
	
	delete gGraph;
	delete gPclSensor;
	delete gSolver;
	delete logger;
	delete clock;
	delete gTransformBroadcaster;
	delete gTransformListener;
	return 0;
}
