#include <iostream>
#include <iterator>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

sensor_msgs::LaserScan hold;

void scancb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	hold = *msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RpLidar_Scan");
	ros::NodeHandle nh;
	ROS_INFO("Start Scan");

	ros::Publisher obd_pub = nh.advertise<std_msgs::Float64>("/obstacle_distance", 1000);
	ros::Subscriber scan_sub = nh.subscribe("/scan", 1000, scancb);

	ros::Rate rate(100);
	
	std_msgs::Float64 input_;
	float threshold = 0.25;

	while(ros::ok())
	{
		ros::spinOnce();

		for (int i = 0;i < hold.ranges.size();i++)
		{
			if (hold.ranges[i] <= threshold)
			{
				//ROS_INFO("Countering Obstacle, distance: %f", hold.ranges[i]);

				input_.data = hold.ranges[i];
				obd_pub.publish(input_);
			}
		}

		rate.sleep();
	}

	ros::shutdown();
	return 0;
}
