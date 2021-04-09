#include "helper_functions.hpp"


#include <boost/format.hpp>

slam3d::RegistrationParameters readRegistrationParameters(ros::NodeHandle& n)
{
	// General
	slam3d::RegistrationParameters params;
	std::string algorithm;
	n.param("registration_algorithm", algorithm, std::string("GICP"));
	if(algorithm == "ICP")
	{
		params.registration_algorithm = slam3d::ICP;
	}else if(algorithm == "GICP")
	{
		params.registration_algorithm = slam3d::GICP;
	}else if(algorithm == "NDT")
	{
		params.registration_algorithm = slam3d::NDT;
	}else
	{
		ROS_WARN("Unknown registration algorithm '%s', using GICP instead.", algorithm.c_str());
	}
	n.param("point_cloud_density", params.point_cloud_density, params.point_cloud_density);	
	n.param("max_fitness_score", params.max_fitness_score, params.max_fitness_score);
	
	// Registration
	n.param("euclidean_fitness_epsilon", params.euclidean_fitness_epsilon, params.euclidean_fitness_epsilon);
	n.param("max_correspondence_distance", params.max_correspondence_distance, params.max_correspondence_distance);
	n.param("transformation_epsilon", params.transformation_epsilon, params.transformation_epsilon);
	n.param("maximum_iterations", params.maximum_iterations, params.maximum_iterations);

	// GICP
	n.param("correspondence_randomness", params.correspondence_randomness, params.correspondence_randomness);
	n.param("maximum_optimizer_iterations", params.maximum_optimizer_iterations, params.maximum_optimizer_iterations);
	n.param("rotation_epsilon", params.rotation_epsilon, params.rotation_epsilon);
	
	// NDT
	n.param("resolution", params.resolution, params.resolution);
	n.param("step_size", params.step_size, params.step_size);
	n.param("outlier_ratio", params.outlier_ratio, params.outlier_ratio);
	
	return params;
}

void readSensorParameters(ros::NodeHandle& parent, slam3d::Sensor* sensor)
{
	ros::NodeHandle n(parent, sensor->getName());
	sensor->setCovarianceScale(n.param("cov_scale", 1.0));
	sensor->setMinPoseDistance(n.param("min_translation", 0.5), n.param("min_rotation", 0.1));
}

void readScanSensorParameters(ros::NodeHandle& parent, slam3d::ScanSensor* ss)
{
	// Read parameters Sensor
	readSensorParameters(parent, ss);

	// Read parameters specific to ScanSensor
	ros::NodeHandle n(parent, ss->getName());
	ss->setNeighborRadius(n.param("neighbor_radius", 5.0), n.param("max_neighbor_links", 5));
	ss->setPatchBuildingRange(n.param("patch_building_range", 5));
	ss->setMinLoopLength(n.param("min_loop_length", 1));
	ss->setLinkPrevious(n.param("link_previous", true));
}

void readPointcloudSensorParameters(ros::NodeHandle& parent, slam3d::PointCloudSensor* pcs)
{
	// Read parameters for ScanSensor
	readScanSensorParameters(parent, pcs);
	
	// Read parameters specific to PointcloudSensor
	ros::NodeHandle n(parent, pcs->getName());
	pcs->setMapResolution(n.param("map_resolution", 0.5));
	pcs->setMapOutlierRemoval(n.param("map_outlier_radius", 0.2), n.param("map_outlier_neighbors", 2));
	
	// Read registration config for fine alignement
	ros::NodeHandle fine(n, "fine");
	pcs->setRegistrationParameters(readRegistrationParameters(fine), false);
	
	// Read registration config for coarse alignement
	ros::NodeHandle coarse(n, "coarse");
	pcs->setRegistrationParameters(readRegistrationParameters(coarse), true);
}
