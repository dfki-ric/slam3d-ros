#include "GpsPublisher.hpp"

#include <slam3d/core/Graph.hpp>
#include <sensor_msgs/PointCloud.h>

GpsPublisher::GpsPublisher(slam3d::Graph* g)
: mGraph(g)
{
	ros::NodeHandle n;
	mPointPublisher = n.advertise<sensor_msgs::PointCloud>("gps_points", 1, true);
}

void GpsPublisher::publishPoints(const std::string& sensor, const std::string& frame)
{
	sensor_msgs::PointCloud msg;
	msg.header.frame_id = frame;
	msg.header.stamp = ros::Time::now();
	
	slam3d::EdgeObjectList edges = mGraph->getEdgesFromSensor(sensor);
	
	msg.points.resize(edges.size());
	unsigned i = 0;
	for(slam3d::EdgeObjectList::const_iterator edge = edges.begin(); edge != edges.end(); ++edge)
	{
		if(edge->constraint->getType() != slam3d::POSITION)
			continue;
			
		slam3d::PositionConstraint::Ptr pos = boost::dynamic_pointer_cast<slam3d::PositionConstraint>(edge->constraint);
		slam3d::Position position = pos->getPosition();
		
		msg.points[i].x = position[0];
		msg.points[i].y = position[1];
		msg.points[i].z = position[2];
		
		i++;
	}
	
	mPointPublisher.publish(msg);
}