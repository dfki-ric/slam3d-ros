#include "ros_common.hpp"

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

timeval RosClock::now()
{
	return fromRosTime(ros::Time::now());
}

RosLogger::RosLogger():Logger(RosClock())
{
}

void RosLogger::message(LOG_LEVEL lvl, const std::string& message)
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

Imu::Imu(const std::string& n, Graph* g, Logger* l)
: PoseSensor(n, g, l)
{
	ros::NodeHandle node;
	mSubscriber = node.subscribe<sensor_msgs::Imu>("imu", 1, &Imu::update, this);
}

void Imu::update(const sensor_msgs::Imu::ConstPtr& imu)
{
	mMeasurement = *imu;
}

Quaternion Imu::getOrientation(timeval stamp)
{
	return Quaternion(mMeasurement.orientation.w,
	                  mMeasurement.orientation.x,
	                  mMeasurement.orientation.y,
	                  mMeasurement.orientation.z);
}

Transform Imu::getPose(timeval stamp)
{
	return Transform(getOrientation(stamp));
}

Covariance<3> Imu::getCovariance(timeval stamp)
{
	Covariance<3> cov;
	cov(0,0) = mMeasurement.orientation_covariance[0];
	cov(0,1) = mMeasurement.orientation_covariance[1];
	cov(0,2) = mMeasurement.orientation_covariance[2];
	cov(1,0) = mMeasurement.orientation_covariance[3];
	cov(1,1) = mMeasurement.orientation_covariance[4];
	cov(1,2) = mMeasurement.orientation_covariance[5];
	cov(2,0) = mMeasurement.orientation_covariance[6];
	cov(2,1) = mMeasurement.orientation_covariance[7];
	cov(2,2) = mMeasurement.orientation_covariance[8];
	return cov * mCovarianceScale;
}

void Imu::handleNewVertex(IdType vertex)
{
	timeval stamp = mGraph->getVertex(vertex).measurement->getTimestamp();
	Transform currentPose = getPose(stamp);
	
	if(mLastVertex > 0)
	{
		Transform t = mLastImuPose.inverse() * currentPose;
		Covariance<6> i = Covariance<6>::Zero();
		i.block<3,3>(3,3) = getCovariance(stamp).inverse();
		SE3Constraint::Ptr se3(new SE3Constraint(mName, t, i));
		mGraph->addConstraint(mLastVertex, vertex, se3);
	}
	
	mLastVertex = vertex;
	mLastImuPose = currentPose;
}
