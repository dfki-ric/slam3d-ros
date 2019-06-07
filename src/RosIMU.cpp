#include "RosIMU.hpp"

RosIMU::RosIMU(Graph* g, Logger* l, ros::NodeHandle& node)
: PoseSensor("IMU", g, l), mTfListener(node)
{
	node.param("reference_frame", mOdometryFrame, std::string("reference"));
	node.param("imu_frame", mRobotFrame, std::string("imu"));
	
	mLastVertex = 0;
}

void RosIMU::handleNewVertex(IdType vertex)
{
	timeval stamp = mGraph->getVertex(vertex).measurement->getTimestamp();
	Transform currentPose = getPose(stamp);
	
	if(mLastVertex > 0)
	{
		TransformWithCovariance twc;
		twc.transform = mLastOdometricPose.inverse() * currentPose;
		twc.covariance = Covariance<6>::Identity() * 100;
		SE3Constraint::Ptr se3(new SE3Constraint(mName, twc));
		mGraph->addConstraint(mLastVertex, vertex, se3);
		mGraph->setCorrectedPose(vertex, mGraph->getVertex(mLastVertex).corrected_pose * twc.transform);
	}
	
	// Add a gravity vector to this vertex
	Eigen::Quaterniond state(currentPose.rotation());
	Direction upVector = state.inverse() * Eigen::Vector3d::UnitZ();
	if(mLastVertex == 0)
	{
		mGravityReference = Eigen::Vector3d::UnitZ();
	}
	GravityConstraint::Ptr grav(new GravityConstraint(mName, upVector, mGravityReference, Covariance<2>::Identity() * 10));
	mGraph->addConstraint(vertex, 0, grav);
	
	mLastVertex = vertex;
	mLastOdometricPose = currentPose;
}

Transform RosIMU::getPose(timeval stamp)
{
	tf::StampedTransform tf_transform;
	try
	{
		mTfListener.waitForTransform(mOdometryFrame, mRobotFrame, fromTimeval(stamp), ros::Duration(1.0));
		mTfListener.lookupTransform(mOdometryFrame, mRobotFrame, fromTimeval(stamp), tf_transform);
	}catch(tf2::TransformException &e)
	{
		throw InvalidPose(e.what());
	}
	return tf2eigen(tf_transform);
}