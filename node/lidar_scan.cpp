#include <cmath>
#include <vector>
#include <iostream>
#include <iterator>
#include <ros/ros.h>
#include <algorithm>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>

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

	ros::Publisher obd_pub = nh.advertise<geometry_msgs::PointStamped>("/obstacle_distance", 1000);
	ros::Subscriber scan_sub = nh.subscribe("/scan", 1000, scancb);

	ros::Rate rate(100);
	
	geometry_msgs::PointStamped input_;
	float threshold = 0.35;
	float angle, x, y, min_;
	int index;
	bool ck;

	while(ros::ok())
	{
		ros::spinOnce();
		
		min_ = threshold;
		ck = true;

		for (int i = 0;i < hold.ranges.size();i++)
		{
			if (hold.ranges[i] <= threshold)
			{
				if (hold.ranges[i] <=min_)
				{
					min_ = hold.ranges[i];
					index = i;
					ck = false;
				}
			}
		}

		if (ck)
		{
			input_.point.x = 0.7;
			input_.point.y = 0.7;
			obd_pub.publish(input_);
		}
		else
		{
			angle = hold.angle_min + index*hold.angle_increment;
			x = min_*cos(angle);
			y = min_*sin(angle);
			input_.point.x = x;
			input_.point.y = y;
			obd_pub.publish(input_);
		}
		
		rate.sleep();
	}

	ros::shutdown();
	return 0;
}
