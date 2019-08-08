#include "PointcloudSensorRos.hpp"

PointcloudSensorRos::PointcloudSensorRos(const std::string& sensor, const std::string robot)
: SensorRos(sensor, robot, mPclSensor)
{
	mPclSensor = new slam3d::PointCloudSensor(mSensorName, &mLogger);
	mSensor = mPclSensor;
	readParams();
	
	slam3d::GICPConfiguration gicp_conf;
	mNode.param("icp_fine/correspondence_randomness", gicp_conf.correspondence_randomness, gicp_conf.correspondence_randomness);
	mNode.param("icp_fine/euclidean_fitness_epsilon", gicp_conf.euclidean_fitness_epsilon, gicp_conf.euclidean_fitness_epsilon);
	mNode.param("icp_fine/max_correspondence_distance", gicp_conf.max_correspondence_distance, gicp_conf.max_correspondence_distance);
	mNode.param("icp_fine/max_fitness_score", gicp_conf.max_fitness_score, gicp_conf.max_fitness_score);
	mNode.param("icp_fine/max_sensor_distance", gicp_conf.max_sensor_distance, gicp_conf.max_sensor_distance);
	mNode.param("icp_fine/maximum_iterations", gicp_conf.maximum_iterations, gicp_conf.maximum_iterations);
	mNode.param("icp_fine/maximum_optimizer_iterations", gicp_conf.maximum_optimizer_iterations, gicp_conf.maximum_optimizer_iterations);
	mNode.param("icp_fine/orientation_sigma", gicp_conf.orientation_sigma, gicp_conf.orientation_sigma);
	mNode.param("icp_fine/point_cloud_density", gicp_conf.point_cloud_density, gicp_conf.point_cloud_density);
	mNode.param("icp_fine/position_sigma", gicp_conf.position_sigma, gicp_conf.position_sigma);
	mNode.param("icp_fine/rotation_epsilon", gicp_conf.rotation_epsilon, gicp_conf.rotation_epsilon);
	mNode.param("icp_fine/transformation_epsilon", gicp_conf.transformation_epsilon, gicp_conf.transformation_epsilon);
	mPclSensor->setFineConfiguaration(gicp_conf);
	
	mNode.param("icp_coarse/correspondence_randomness", gicp_conf.correspondence_randomness, gicp_conf.correspondence_randomness);
	mNode.param("icp_coarse/euclidean_fitness_epsilon", gicp_conf.euclidean_fitness_epsilon, gicp_conf.euclidean_fitness_epsilon);
	mNode.param("icp_coarse/max_correspondence_distance", gicp_conf.max_correspondence_distance, gicp_conf.max_correspondence_distance);
	mNode.param("icp_coarse/max_fitness_score", gicp_conf.max_fitness_score, gicp_conf.max_fitness_score);
	mNode.param("icp_coarse/max_sensor_distance", gicp_conf.max_sensor_distance, gicp_conf.max_sensor_distance);
	mNode.param("icp_coarse/maximum_iterations", gicp_conf.maximum_iterations, gicp_conf.maximum_iterations);
	mNode.param("icp_coarse/maximum_optimizer_iterations", gicp_conf.maximum_optimizer_iterations, gicp_conf.maximum_optimizer_iterations);
	mNode.param("icp_coarse/orientation_sigma", gicp_conf.orientation_sigma, gicp_conf.orientation_sigma);
	mNode.param("icp_coarse/point_cloud_density", gicp_conf.point_cloud_density, gicp_conf.point_cloud_density);
	mNode.param("icp_coarse/position_sigma", gicp_conf.position_sigma, gicp_conf.position_sigma);
	mNode.param("icp_coarse/rotation_epsilon", gicp_conf.rotation_epsilon, gicp_conf.rotation_epsilon);
	mNode.param("icp_coarse/transformation_epsilon", gicp_conf.transformation_epsilon, gicp_conf.transformation_epsilon);
	mPclSensor->setCoarseConfiguaration(gicp_conf);
	
	double radius;
	int links, range, loop_len;

	mNode.param("min_loop_length", loop_len, 1);
	mPclSensor->setMinLoopLength(loop_len);

	
	mNode.param("scan_resolution", mScanResolution, 0.5);

	mNode.param("neighbor_radius", radius, 5.0);
	mNode.param("max_neighbor_links", links, 5);
	mNode.param("patch_building_range", range, 5);

	mNode.param("use_seq_scan_matching", mSeqScanMatch, true);

	mPclSensor->setNeighborRadius(radius, links);
	mPclSensor->setPatchBuildingRange(range);
}

PointcloudSensorRos::~PointcloudSensorRos()
{
	delete mPclSensor;
}

bool PointcloudSensorRos::addScanWithOdometry(const slam3d::PointCloud::ConstPtr& pcl,
                                              const slam3d::Transform& laser_pose,
                                              const slam3d::Transform& odom)
{
	slam3d::PointCloud::Ptr cloud;
	if(mScanResolution > 0)
	{
		cloud = mPclSensor->downsample(pcl, mScanResolution);
	}else
	{	
		cloud = slam3d::PointCloud::Ptr(new slam3d::PointCloud(*pcl));
	}
	
	slam3d::PointCloudMeasurement::Ptr m(
		new slam3d::PointCloudMeasurement(cloud, mRobotName, mSensorName, laser_pose));

	
	if(mPclSensor->addMeasurement(m, odom, mSeqScanMatch))
	{
		mPclSensor->linkLastToNeighbors(true);
		return true;
	}
	return false;
}

