#include <slam3d/core/PoseSensor.hpp>

class RosIMU : public slam3d::PoseSensor
{
public:
	RosIMU(Graph* g, Logger* l, ros::NodeHandle& node);
	
	void handleNewVertex(IdType vertex);
	Transform getPose(timeval stamp);
	
private:
	tf::TransformListener mTfListener;
	std::string mOdometryFrame;
	std::string mRobotFrame;
	
	Transform mLastOdometricPose;
	IdType mLastVertex;
	Direction mGravityReference;
};
