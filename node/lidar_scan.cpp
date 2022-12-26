#include <cmath>
#include <iostream>
#include <iterator>
#include <ros/ros.h>
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
	float threshold = 0.25;
	float angle, x, y;

	while(ros::ok())
	{
		ros::spinOnce();

		for (int i = 0;i < hold.ranges.size();i++)
		{
			if (hold.ranges[i] <= threshold)
			{
				ROS_INFO("Countering Obstacle, distance: %f", hold.ranges[i]);
				angle = hold.angle_min + i*hold.angle_increment;
				//ROS_INFO("Direction: %f", angle);
				x = hold.ranges[i]*cos(angle);
				y = hold.ranges[i]*sin(angle);
				input_.point.x = x;
				input_.point.y = y;
				obd_pub.publish(input_);
			}
		}

		rate.sleep();
	}

	ros::shutdown();
	return 0;
}
