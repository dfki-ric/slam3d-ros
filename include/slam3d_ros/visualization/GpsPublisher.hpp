#include <ros/ros.h>

namespace slam3d
{
	class Graph;
}


class GpsPublisher
{
public:
	GpsPublisher(slam3d::Graph* g);
	~GpsPublisher(){}
	
	void publishPoints(const std::string& sensor, const std::string& frame);
	
private:
	slam3d::Graph* mGraph;
	ros::Publisher mPointPublisher;
};
