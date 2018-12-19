#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

#include <slam3d/core/Logger.hpp>
#include <slam3d/core/Clock.hpp>
#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/core/Graph.hpp>

using namespace slam3d;

ros::Time fromTimeval(timeval tv)
{
	ros::Time rt;
	rt.sec = tv.tv_sec;
	rt.nsec = tv.tv_usec * 1000;
	return rt;
}

timeval fromRosTime(ros::Time rt)
{
	timeval tv;
	tv.tv_sec = rt.sec;
	tv.tv_usec = rt.nsec / 1000;
	return tv;
}

tf::Transform eigen2tf(Transform in)
{
	tf::Transform out;
	tf::transformEigenToTF(in.cast<double>(), out);
	return out;
}

Transform tf2eigen(tf::Transform in)
{
	Eigen::Affine3d out;
	tf::transformTFToEigen(in, out);
	Transform tf;
	tf.linear() = out.linear();
	tf.translation() = out.translation();
	return tf;
}

class RosClock : public Clock
{
public:
	virtual timeval now()
	{
		return fromRosTime(ros::Time::now());
	}
};

class RosLogger : public Logger
{
public:
	RosLogger():Logger(RosClock()){}
	~RosLogger(){}
	
	virtual void message(LOG_LEVEL lvl, const std::string& message)
	{
		switch(lvl)
		{
		case DEBUG:
			ROS_DEBUG("%s", message.c_str());
			break;
		case INFO:
			ROS_INFO("%s", message.c_str());
			break;
		case WARNING:
			ROS_WARN("%s", message.c_str());
			break;
		case ERROR:
			ROS_ERROR("%s", message.c_str());
			break;
		case FATAL:
			ROS_FATAL("%s", message.c_str());
			break;
		}
	}
};

class RosTfOdometry : public PoseSensor
{
public:
	RosTfOdometry(Graph* g, Logger* l, ros::NodeHandle& node)
	: PoseSensor("Odometry", g, l), mTfListener(node)
	{
		node.param("odometry_frame", mOdometryFrame, std::string("odometry"));
		node.param("robot_frame", mRobotFrame, std::string("robot"));
		
		mLastVertex = 0;
		mLastOdometricPose = Transform::Identity();
	}
	
	~RosTfOdometry(){}
	
	void handleNewVertex(IdType vertex)
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
		mLastVertex = vertex;
		mLastOdometricPose = currentPose;
	}
	
	Transform getPose(timeval stamp)
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
	
private:
	tf::TransformListener mTfListener;
	std::string mOdometryFrame;
	std::string mRobotFrame;
	
	Transform mLastOdometricPose;
	IdType mLastVertex;
};
