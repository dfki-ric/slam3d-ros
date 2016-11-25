#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Duration.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <fstream>

sensor_msgs::PointCloud2 pcl;
int32_t num;
float* data;
std::string path;
ros::Publisher pub;
ros::Subscriber offset_sub;

FILE* gPointcloudLog;
ros::Time gNextStamp;
ros::Duration gTimeOffset;
int gNextCloud;
int gStepWidth;
std::string gScanFrame;
bool gStart;

bool getNextCloud()
{
	for(int i = 0; i < gStepWidth; ++i)
	{
		// Read Timestamp
		int64_t stamp;
		int32_t read = fread(&stamp, sizeof(int64_t), 1, gPointcloudLog);
	
		if(read != 1)
		{
			return false;
		}
		stamp *= 1000; // Microseconds -> Nanoseconds
		gNextStamp.fromNSec(stamp);
		
		// Read Pointcloud number
		int value;
		read = fread(&value, sizeof(int), 1, gPointcloudLog);
		if(read != 1)
		{
			return false;
		}
		gNextCloud++;
	}
	return true;
}

bool publishNext()
{
	ROS_DEBUG("Now publishing cloud %d.", gNextCloud);
	
	// Read in the file
	char name[200];
	sprintf(name, "%s/%06d.bin", path.c_str(), gNextCloud);
	FILE *stream = fopen (name,"rb");
	if(!stream)
	{
		ROS_WARN("Could not open: %s", name);
		return false;
	}
	int32_t points = fread(data, sizeof(float), num, stream) / 4;
	fclose(stream);
	
	pcl.header.seq = gNextCloud;
	pcl.header.stamp = gNextStamp + gTimeOffset;

	pcl.height = 1;
	pcl.width = points;
	pcl.row_step = points * 4 * sizeof(float);
	
	// Copy the data
	pcl.data.resize(pcl.row_step);
	memcpy(&pcl.data[0], data, pcl.row_step);

	pub.publish(pcl);
	return true;
}

void receiveOffset(const std_msgs::Duration::ConstPtr& msg)
{
	gTimeOffset = msg->data;
	gStart = true;
	ROS_INFO("Received offset.");
}

void receiveTriggerSignal(const std_msgs::Empty::ConstPtr& signal)
{
	gNextStamp = ros::Time::now();
	publishNext();
	gNextCloud += gStepWidth;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_publisher");
	ros::NodeHandle n;

	pub = n.advertise<sensor_msgs::PointCloud2>("velodyne_scan", 1);

	int rate;
	n.param("replay_path", path, std::string(""));
	n.param("replay_rate", rate, 1);
	n.param("skip", gNextCloud, 0);
	n.param("step", gStepWidth, 1);
	n.param("scan_frame", gScanFrame, std::string("velodyne_laser"));
	n.param("start", gStart, true);

	if(!gStart)
	{
		offset_sub = n.subscribe<std_msgs::Duration>("offset", 1, &receiveOffset);
	}

	num = 1000000;

	// Open the log-file with the timestamps
	std::string log_path = path + "/pcl.log";
	if(rate == 0)
	{
		gPointcloudLog = fopen(log_path.c_str(), "rb");
	}else
	{
		gPointcloudLog = NULL;
	}
	
	if(gPointcloudLog)
	{
		if(!getNextCloud())
		{
			ROS_ERROR("Could not find first Pointcloud!");
			return 1;
		}
		if(gStart)
		{
			gTimeOffset = ros::Time::now() - gNextStamp;
			ROS_INFO("First Pointcloud at: %d, Offset: %d", gNextStamp.sec, gTimeOffset.sec);
		}
	}

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
	while(pub.getNumSubscribers() == 0)
	{
		sleep(1);
	}

	// allocate 4 MB buffer (only ~130*4*4 KB are needed) and read in the file
	data = new float[num];

	if(gPointcloudLog && rate == 0)
	{
		// Case 1: Publish according to timestamps in the log file
		ROS_INFO("Publishing according to timestamps in the log file.");
		ros::Rate update_rate(100);
		while(ros::ok())
		{
			if(gStart && (gNextStamp + gTimeOffset) <= ros::Time::now())
			{
				publishNext();
				if(!getNextCloud())
					break;
			}
			ros::spinOnce();
			update_rate.sleep();
		}
	}else
	{
		if(rate > 0)
		{
			// Case 2: Publish with a fixed rate
			ROS_INFO("Publish with a fixed rate of %d.", rate);
			ros::Rate publish_rate(rate);
			while(ros::ok())
			{
				gNextStamp = ros::Time::now();
				publishNext();
				gNextCloud += gStepWidth;
				publish_rate.sleep();
			}
		}else
		{
			// Case 3: Publish on request
			ROS_INFO("Publishing on request.");
			ros::Subscriber sub = n.subscribe("trigger", 10, &receiveTriggerSignal);
			gNextStamp = ros::Time::now();
			publishNext();
			gNextCloud += gStepWidth;
			ros::spin();
		}
	}
	
	delete[] data;
	if(gPointcloudLog)
		fclose(gPointcloudLog);
	return 0;
}
