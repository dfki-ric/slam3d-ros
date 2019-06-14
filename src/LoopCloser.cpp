#include "LoopCloser.hpp"

LoopCloser::LoopCloser() : mServer("LoopCloser")
{
	// create an interactive marker for our server
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "map";
	int_marker.header.stamp=ros::Time::now();
	int_marker.name = "loop";
	int_marker.description = "manual loop closure control";

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

	// create a non-interactive control which contains the box
	visualization_msgs::InteractiveMarkerControl box_control;
	box_control.always_visible = true;
	box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
	box_control.markers.push_back( box_marker );
	
/*	// Create a POINTS amrker
	visualization_msgs::Marker points_marker;
	points_marker.type = visualization_msgs::Marker::POINTS;
	points_marker.action = visualization_msgs::Marker::ADD;
	points_marker.pose.position.x = 0;
	points_marker.pose.position.y = 0;
	points_marker.pose.position.z = 0;
	points_marker.pose.orientation.x = 0.0;
	points_marker.pose.orientation.y = 0.0;
	points_marker.pose.orientation.z = 0.0;
	points_marker.pose.orientation.w = 1.0;
	points_marker.scale.x = 0.1;
	points_marker.scale.y = 0.1;
	points_marker.scale.z = 0; // unused
	points_marker.color.a = 1.0;
	points_marker.color.r = 1.0;
	points_marker.color.g = 1.0;
	points_marker.color.b = 1.0;
	box_control.markers.push_back(points_marker);
*/
	// add the control to the interactive marker
	int_marker.controls.push_back( box_control );

	// create a control which will move the box
	// this control does not contain any markers,
	// which will cause RViz to insert two arrows
	visualization_msgs::InteractiveMarkerControl control;
	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "rotate_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	control.name = "move_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
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
	control.name = "move_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);
	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	mServer.insert(int_marker, boost::bind(&LoopCloser::processFeedback, this, _1));

	interactive_markers::MenuHandler::EntryHandle close_loop =
		mMenuHandler.insert( "Close loop", boost::bind(&LoopCloser::closeLoopCB, this, _1));
	mMenuHandler.apply(mServer, "loop");

	// 'commit' changes and send to all clients
	mServer.applyChanges();
}

void LoopCloser::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO_STREAM( feedback->marker_name << " is now at "
		<< feedback->pose.position.x << ", "
		<< feedback->pose.position.y << ", "
		<< feedback->pose.position.z );
}

void LoopCloser::closeLoopCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO("Triggered manual loop closure!");
}
