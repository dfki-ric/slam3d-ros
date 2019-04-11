#include <ros/ros.h>

namespace slam3d
{
	class Graph;
}

class GraphPublisher
{
public:
	GraphPublisher(slam3d::Graph* g, const std::string& s);
	~GraphPublisher();
	
	void publishNodes(const ros::Time& stamp, const std::string& frame);
	void publishEdges(const ros::Time& stamp, const std::string& frame);
	void publishGraph(const ros::Time& stamp, const std::string& frame);
	
private:
	slam3d::Graph* mGraph;
	std::string mSensor;
	ros::Publisher mPosePublisher;
	ros::Publisher mEdgePublisher;
};
