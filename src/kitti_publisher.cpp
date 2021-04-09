#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Duration.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <fstream>

sensor_msgs::PointCloud2 pcl;
int32_t num;
float* data;
std::string path;
ros::Publisher pcl_pub;
ros::Publisher img_pub;

ros::Time gNextStamp;
int gNextCloud;
int gStepWidth;
std::string gScanFrame;

bool publishNextImage()
{
	char name[200];
	sprintf(name, "%s/image_2/%06d.png", path.c_str(), gNextCloud);

	cv_bridge::CvImage cv_image;
	cv_image.image = cv::imread(name);
	if(!cv_image.image.data)
	{
		ROS_WARN("Could not open: %s", name);
		return false;
	}
	cv_image.encoding = "bgr8";
	sensor_msgs::Image ros_image;
	cv_image.toImageMsg(ros_image);
	ros_image.header.seq = gNextCloud;
	ros_image.header.stamp = gNextStamp;
	img_pub.publish(ros_image);
	return true;
}

bool publishNextPointcloud()
{
	ROS_DEBUG("Now publishing cloud %d.", gNextCloud);
	
	// Read in the file
	char name[200];
	sprintf(name, "%s/velodyne/%06d.bin", path.c_str(), gNextCloud);
	FILE *stream = fopen (name,"rb");
	if(!stream)
	{
		ROS_WARN("Could not open: %s", name);
		return false;
	}
	int32_t points = fread(data, sizeof(float), num, stream) / 4;
	fclose(stream);
	
	pcl.header.seq = gNextCloud;
	pcl.header.stamp = gNextStamp;

	pcl.height = 1;
	pcl.width = points;
	pcl.row_step = points * 4 * sizeof(float);
	
	// Copy the data
	pcl.data.resize(pcl.row_step);
	memcpy(&pcl.data[0], data, pcl.row_step);

	pcl_pub.publish(pcl);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_publisher");
	ros::NodeHandle n;

	pcl_pub = n.advertise<sensor_msgs::PointCloud2>("velodyne_scan", 1);
	img_pub = n.advertise<sensor_msgs::Image>("camera_image", 1);

	int rate;
	n.param("replay_path", path, std::string(""));
	n.param("replay_rate", rate, 1);
	n.param("skip", gNextCloud, 0);
	n.param("step", gStepWidth, 1);
	n.param("scan_frame", gScanFrame, std::string("velodyne_laser"));

	num = 1000000;

	// Create PointCloud2 msg
	tf::TransformListener listener;
	pcl.header.frame_id = listener.resolve(gScanFrame);
	pcl.point_step = 4 * sizeof(float);

	// Description of channels in PointCloud2
	pcl.fields.resize(4);
	pcl.fields[0].name = "x";
	pcl.fields[0].offset = 0;
	pcl.fields[0].datatype = 7;
	pcl.fields[1].name = "y";
	pcl.fields[1].offset = 4;
	pcl.fields[1].datatype = 7;
	pcl.fields[2].name = "z";
	pcl.fields[2].offset = 8;
	pcl.fields[2].datatype = 7;
	pcl.fields[3].name = "intensity";
	pcl.fields[3].offset = 12;
	pcl.fields[3].datatype = 7;

	// Wait for someone to connect to our pointclouds
	while(img_pub.getNumSubscribers() == 0 && pcl_pub.getNumSubscribers() == 0)
	{
		sleep(1);
	}

	// allocate 4 MB buffer (only ~130*4*4 KB are needed) and read in the file
	data = new float[num];

	// Case 2: Publish with a fixed rate
	ROS_INFO("Publish with a fixed rate of %d.", rate);
	ros::Rate publish_rate(rate);
	bool img_ok = true;
	bool pcl_ok = true;
	while(ros::ok())
	{
		gNextStamp = ros::Time::now();

		if(pcl_ok)
			pcl_ok = publishNextPointcloud();
			
		if(img_ok)
			img_ok = publishNextImage();
		
		if(!img_ok && !pcl_ok)
			break;
	
		gNextCloud += gStepWidth;
		publish_rate.sleep();
	}
	
	delete[] data;
	return 0;
}
