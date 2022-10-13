#include <slam3d/core/PoseSensor.hpp>

#include <tf/transform_listener.h>

/**
 * @class TransformSensor
 * @author Sebastian Kasperski
 * @date 07/06/19
 * @file ros_tf.hpp
 * @brief Base class for all PoseSensors that operate on a TF-Transform
 */
class TransformSensor : public slam3d::PoseSensor
{
public:
	TransformSensor(const std::string& n, slam3d::Graph* g, slam3d::Logger* l);
	slam3d::Transform getPose(timeval stamp);
	void setTF(tf::TransformListener* tf, const std::string& ref, const std::string& robot);
	
protected:
	tf::TransformListener* mTfListener;
	std::string mReferenceFrame;
	std::string mRobotFrame;
};

/**
 * @class IMU
 * @author Sebastian Kasperski
 * @date 07/06/19
 * @file ros_tf.hpp
 * @brief Sensor that creates gravity constraints from an IMU's roll-pitch-angle.
 */
class TfImu : public TransformSensor
{
public:
	TfImu(slam3d::Graph* g, slam3d::Logger* l);
	void handleNewVertex(slam3d::IdType vertex);
	
private:
	slam3d::Direction mGravityReference;
};

/**
 * @class Odometry
 * @author Sebastian Kasperski
 * @date 07/06/19
 * @file ros_tf.hpp
 * @brief Sensor that creates SE(3) constraints between adjacent measurements.
 */
class TfOdometry : public TransformSensor
{
public:
	TfOdometry(slam3d::Graph* g, slam3d::Logger* l);
	void handleNewVertex(slam3d::IdType vertex);
	
private:
	slam3d::Transform mLastOdometricPose;
	slam3d::IdType mLastVertex;
};
