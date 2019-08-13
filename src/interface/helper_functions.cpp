#include "helper_functions.hpp"


#include <boost/format.hpp>

void readSensorParameters(ros::NodeHandle& parent, slam3d::Sensor* sensor)
{
	ros::NodeHandle n(parent, sensor->getName());
	sensor->setCovarianceScale(n.param("cov_scale", 1.0));
	sensor->setMinPoseDistance(n.param("min_translation", 0.5), n.param("min_rotation", 0.1));
}

void readPointcloudSensorParameters(ros::NodeHandle& parent, slam3d::PointCloudSensor* pcs)
{
	ros::NodeHandle n(parent, pcs->getName());
	readSensorParameters(parent, pcs);
	
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
	pcs->setFineConfiguaration(gicp_conf);
	
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
	pcs->setCoarseConfiguaration(gicp_conf);

	pcs->setMinLoopLength(n.param("min_loop_length", 1));
	pcs->setNeighborRadius(n.param("neighbor_radius", 5.0), n.param("max_neighbor_links", 5));
	pcs->setPatchBuildingRange(n.param("patch_building_range", 5));
	
	pcs->setMapResolution(n.param("map_resolution", 0.5));
	pcs->setMapOutlierRemoval(n.param("map_outlier_radius", 0.2), n.param("map_outlier_neighbors", 2));
}