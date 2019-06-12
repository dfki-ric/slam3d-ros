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
	
	void addNodeSensor(const std::string& sensor, double r, double g, double b);
	void addEdgeSensor(const std::string& sensor);
	
	void publishNodes(const ros::Time& stamp, const std::string& frame);
	void publishEdges(const std::string& sensor, const ros::Time& stamp, const std::string& frame);
	void publishPoseEdges(const std::string& sensor, const ros::Time& stamp, const std::string& frame);
	
private:
	slam3d::Graph* mGraph;
	std::map<std::string, Color> mSensorMap;
	ros::Publisher mPosePublisher;
	std::map<std::string, ros::Publisher> mEdgePublisherMap;
	ros::NodeHandle mNode;
};
