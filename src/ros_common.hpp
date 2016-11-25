#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

#include <slam3d/Logger.hpp>
#include <slam3d/Clock.hpp>
#include <slam3d/Odometry.hpp>
#include <slam3d/GraphMapper.hpp>

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

class RosTfOdometry : public Odometry
{
public:
	RosTfOdometry(Logger* logger, ros::NodeHandle& node) : Odometry(logger), mTfListener(node)
	{
		node.param("odometry_frame", mOdometryFrame, std::string("odometry"));
		node.param("robot_frame", mRobotFrame, std::string("robot"));
	}
	
	~RosTfOdometry(){}
	
	Transform getOdometricPose(timeval stamp)
	{
		tf::StampedTransform tf_transform;
		try
		{
			mTfListener.waitForTransform(mOdometryFrame, mRobotFrame, fromTimeval(stamp), ros::Duration(1.0));
			mTfListener.lookupTransform(mOdometryFrame, mRobotFrame, fromTimeval(stamp), tf_transform);
		}catch(tf2::TransformException &e)
		{
			ROS_ERROR("TF-Odometry: %s", e.what());
			throw OdometryException();
		}
		return tf2eigen(tf_transform);
	}
	
	TransformWithCovariance getRelativePose(timeval last, timeval next)
	{
		ROS_ERROR("Not implemented!");
		throw OdometryException();
/*
		tf::Stamped<tf::Pose> inPose, outPose;
		inPose.frame_id_ = mRobotFrame;
		inPose.stamp_ = fromTimeval(next);
		inPose.setData(tf::Pose::getIdentity());
		
		try
		{
			mTfListener.waitForTransform(mRobotFrame, mOdometryFrame, fromTimeval(next), ros::Duration(1.0));
			mTfListener.transformPose(mRobotFrame, fromTimeval(last), inPose, mOdometryFrame, outPose);
		}catch(tf2::TransformException &e)
		{
			ROS_ERROR("TF-Odometry: %s", e.what());
			throw OdometryException();
		}
		
		TransformWithCovariance result;
		result.transform = tf2eigen(outPose);
		result.covariance = Covariance::Identity(); // Calculate covariance from odometric distance?
		return result;
*/
	}
	
	Covariance calculateCovariance(const Transform &tf)
	{
		return Covariance::Identity() * 100;
	}
	
private:
	tf::TransformListener mTfListener;
	std::string mOdometryFrame;
	std::string mRobotFrame;
};
