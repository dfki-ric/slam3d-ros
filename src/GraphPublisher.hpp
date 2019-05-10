#include <ros/ros.h>

#include <map>

namespace slam3d
{
	class Graph;
}

struct Color
{
	double r;
	double g;
	double b;
};

class GraphPublisher
{
public:
	GraphPublisher(slam3d::Graph* g);
	~GraphPublisher();
	
	void addSensor(const std::string& sensor, double r, double g, double b);
	
	void publishNodes(const ros::Time& stamp, const std::string& frame);
	void publishEdges(const ros::Time& stamp, const std::string& frame);
	void publishGraph(const ros::Time& stamp, const std::string& frame);
	
private:
	slam3d::Graph* mGraph;
	std::map<std::string, Color> mSensorMap;
	ros::Publisher mPosePublisher;
	ros::Publisher mEdgePublisher;
};
