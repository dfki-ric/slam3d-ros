#include "LoopCloser.hpp"

#include <slam3d/core/Mapper.hpp>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>

using namespace slam3d;

LoopCloser::LoopCloser(Mapper* m, PointCloudSensor* pcs)
 : mServer("LoopCloser"), mMapper(m), mCovarianceScale(1.0)
{
	interactive_markers::MenuHandler::EntryHandle close_loop =
		mMenuHandler.insert("Close loop", boost::bind(&LoopCloser::closeLoopCB, this, pcs, _1));
}


void LoopCloser::initLoopClosing(const PointCloudMeasurement::Ptr& pc)
{
	mSourceCloud = pc;
	
	// create an interactive marker for our server
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "base_link";
	int_marker.header.stamp=ros::Time::now();
	int_marker.name = "loop";
	int_marker.description = "manual loop closure control";

	// create a non-interactive control which contains the box
	visualization_msgs::InteractiveMarkerControl box_control;
	box_control.always_visible = true;
	box_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
	box_control.orientation.w = 1;
	box_control.orientation.x = 0;
	box_control.orientation.y = 1;
	box_control.orientation.z = 0;
	box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

	// create a grey box marker
	visualization_msgs::Marker box_marker;
	box_marker.type = visualization_msgs::Marker::CUBE;
	box_marker.scale.x = 0.45;
	box_marker.scale.y = 0.45;
	box_marker.scale.z = 0.45;
	box_marker.color.r = 0.5;
	box_marker.color.g = 0.5;
	box_marker.color.b = 0.5;
	box_marker.color.a = 1.0;
	box_control.markers.push_back( box_marker );


	// Create the PointCloud marker
	Transform sensorPose = pc->getSensorPose();
	visualization_msgs::Marker points;
	points.header.frame_id = "";
	points.header.stamp = ros::Time::now();
	points.action = visualization_msgs::Marker::ADD;

	tf::poseEigenToMsg(sensorPose, points.pose);

	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.1;
	points.scale.y = 0.1;
	points.color.r = 1.0;
	points.color.g = 1.0;
	points.color.b = 1.0;
	points.color.a = 1.0;
	
	for(auto p = pc->getPointCloud()->begin(); p!= pc->getPointCloud()->end(); p++)
	{
		geometry_msgs::Point point;
		point.x = p->x;
		point.y = p->y;
		point.z = p->z;
		points.points.push_back(point);
	}
	box_control.markers.push_back( points );

	int_marker.controls.push_back( box_control );

	// Create the controls which will move the box. This control does not contain any markers,
	// which will cause RViz to insert the default markers (arrows and circles)
	visualization_msgs::InteractiveMarkerControl control;
	control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "rotate_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "rotate_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);

	// Add it to the server
	mServer.insert(int_marker);
	mMenuHandler.apply(mServer, "loop");
	mServer.applyChanges();
}

void LoopCloser::closeLoopCB(PointCloudSensor* pcs, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO("Triggered manual loop closure at (%.2f, %.2f, %.2f)", feedback->pose.position.x
	,feedback->pose.position.y, feedback->pose.position.z);
	
	Transform pose;
	tf::poseMsgToEigen(feedback->pose, pose);
	mMapper->getGraph()->buildNeighborIndex(mSourceCloud->getSensorName());
	VertexObjectList neighbors = mMapper->getGraph()->getNearbyVertices(pose, 10.0);
	
	if(neighbors.size() > 0)
	{
		IdType src_id = mMapper->getGraph()->getVertex(mSourceCloud->getUniqueId()).index;
		IdType tgt_id = neighbors[0].index;
//		TransformWithCovariance twc;
//		twc.transform = pose.inverse() * neighbors[0].corrected_pose;
//		twc.covariance = Covariance<6>::Identity() * mCovarianceScale;

		Transform rel_pose = pose.inverse() * neighbors[0].corrected_pose;
		pcs->link(src_id, tgt_id, rel_pose);

//		Constraint::Ptr se3(new SE3Constraint("LoopCloser", twc));
//		mMapper->getGraph()->addConstraint(src_id, tgt_id, se3);
		mMapper->getGraph()->optimize(10000);
	}else
	{
		ROS_ERROR("Could not close the loop, no vertex near target pose.");
		return;
	}
	
	mServer.clear();
	mServer.applyChanges();
}
