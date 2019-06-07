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