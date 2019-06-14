#include "ros_tf.hpp"
#include "ros_common.hpp"

using namespace slam3d;

TransformSensor::TransformSensor(const std::string& n, Graph* g, Logger* l)
: PoseSensor(n, g, l)
{
	mReferenceFrame = "odometry";
	mRobotFrame = "robot";
	mTfListener = NULL;
}

void TransformSensor::setTF(tf::TransformListener* tf, const std::string& ref, const std::string& robot)
{
	mTfListener = tf;
	mReferenceFrame = ref;
	mRobotFrame = robot;
}

Transform TransformSensor::getPose(timeval stamp)
{
	if(!mTfListener)
	{
		ROS_ERROR("You must call setTF() before using the TransformSensor!");
		return Transform::Identity();
	}

	tf::StampedTransform tf_transform;
	try
	{
		mTfListener->waitForTransform(mReferenceFrame, mRobotFrame, fromTimeval(stamp), ros::Duration(1.0));
		mTfListener->lookupTransform(mReferenceFrame, mRobotFrame, fromTimeval(stamp), tf_transform);
	}catch(tf2::TransformException &e)
	{
		throw InvalidPose(e.what());
	}
	return tf2eigen(tf_transform);
}

IMU::IMU(Graph* g, Logger* l)
: TransformSensor("IMU", g, l)
{
	mGravityReference = Eigen::Vector3d::UnitZ();
}

void IMU::handleNewVertex(IdType vertex)
{
	timeval stamp = mGraph->getVertex(vertex).measurement->getTimestamp();
	Transform currentPose = getPose(stamp);
	
	// Add a gravity vector to this vertex
	Eigen::Quaterniond state(currentPose.rotation());
	Direction upVector = state.inverse() * Eigen::Vector3d::UnitZ();
	GravityConstraint::Ptr grav(new GravityConstraint(mName, upVector, mGravityReference, Covariance<2>::Identity() * 10));
	mGraph->addConstraint(vertex, 0, grav);
}

Odometry::Odometry(Graph* g, Logger* l)
: TransformSensor("Odometry", g, l)
{
	mLastVertex = 0;
	mLastOdometricPose = Transform::Identity();
}

void Odometry::handleNewVertex(IdType vertex)
{
	timeval stamp = mGraph->getVertex(vertex).measurement->getTimestamp();
	Transform currentPose = getPose(stamp);
	
	if(mLastVertex > 0)
	{
		Transform t = mLastOdometricPose.inverse() * currentPose;
		ScalarType rot = Eigen::AngleAxis<ScalarType>(t.rotation()).angle();
		ScalarType trans = t.translation().norm();
		ScalarType error = 1.0 + rot + trans; // magic

		TransformWithCovariance twc;
		twc.transform = t;
		twc.covariance = Covariance<6>::Identity() * error;
		SE3Constraint::Ptr se3(new SE3Constraint(mName, twc));
		mGraph->addConstraint(mLastVertex, vertex, se3);
		mGraph->setCorrectedPose(vertex, mGraph->getVertex(mLastVertex).corrected_pose * twc.transform);
	}
	
	mLastVertex = vertex;
	mLastOdometricPose = currentPose;
}
