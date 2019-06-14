#ifndef LOOP_CLOSER_HPP
#define LOOP_CLOSER_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <slam3d/core/Types.hpp>

class LoopCloser
{
public:
	LoopCloser();
	
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void closeLoopCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

private:
	interactive_markers::InteractiveMarkerServer mServer;
	interactive_markers::MenuHandler mMenuHandler;
};

#endif
