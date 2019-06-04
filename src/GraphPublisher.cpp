#include "GraphPublisher.hpp"

#include <slam3d/core/Graph.hpp>
#include <visualization_msgs/Marker.h>

using namespace slam3d;

GraphPublisher::GraphPublisher(slam3d::Graph* g) : mGraph(g)
{
	ros::NodeHandle n;
	mPosePublisher = n.advertise<visualization_msgs::Marker>("vertices", 1, true);
	mEdgePublisher = n.advertise<visualization_msgs::Marker>("edges", 1, true);
}

GraphPublisher::~GraphPublisher()
{
	
}

void GraphPublisher::addSensor(const std::string& sensor, double r, double g, double b)
{
	Color color;
	color.r = r;
	color.g = g;
	color.b = b;
	mSensorMap.insert(std::pair<std::string, Color>(sensor, color));
}

void GraphPublisher::publishNodes(const ros::Time& stamp, const std::string& frame)
{
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
	
	slam3d::VertexObjectList vertices;
	for(auto it = mSensorMap.begin(); it != mSensorMap.end(); it++)
	{
		slam3d::VertexObjectList v = mGraph->getVerticesFromSensor(it->first);
		vertices.insert(vertices.end(), v.begin(), v.end());
	}
	
	marker.points.resize(vertices.size());
	marker.colors.resize(vertices.size());
	
	unsigned i = 0;
	for(VertexObjectList::const_iterator it = vertices.begin(); it != vertices.end(); it++)
	{
		Transform pose = it->corrected_pose;
		marker.points[i].x = pose.translation()[0];
		marker.points[i].y = pose.translation()[1];
		marker.points[i].z = pose.translation()[2];
		
		Color c = mSensorMap.at(it->measurement->getSensorName());
		marker.colors[i].r = c.r;
		marker.colors[i].g = c.g;
		marker.colors[i].b = c.b;
		marker.colors[i].a = 1.0;
		
		i++;
	}
	mPosePublisher.publish(marker);
}

void GraphPublisher::publishEdges(const ros::Time& stamp, const std::string& frame)
{
	slam3d::EdgeObjectList edges = mGraph->getEdgesFromSensor("");

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
		if(edge->constraint->getType() != SE3)
			continue;

		const VertexObject& source_obj = mGraph->getVertex(edge->source);
		Transform::ConstTranslationPart source_pose = source_obj.corrected_pose.translation();
		marker.points[2*i].x = source_pose[0];
		marker.points[2*i].y = source_pose[1];
		marker.points[2*i].z = source_pose[2];

		const VertexObject& target_obj = mGraph->getVertex(edge->target);
		Transform::ConstTranslationPart target_pose = target_obj.corrected_pose.translation();
		marker.points[2*i+1].x = target_pose[0];
		marker.points[2*i+1].y = target_pose[1];
		marker.points[2*i+1].z = target_pose[2];
		
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
	mEdgePublisher.publish(marker);
}

void GraphPublisher::publishGraph(const ros::Time& stamp, const std::string& frame)
{
	publishNodes(stamp, frame);
	publishEdges(stamp, frame);
}
