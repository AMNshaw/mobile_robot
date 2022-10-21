#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <tf/tf.h>
#include <string>
//#define SLAM
#define Mocap

geometry_msgs::PoseStamped host_mocap;

#ifdef Mocap
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double roll, pitch, yaw;
    host_mocap = *msg;
    tf::Quaternion Q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}
#else
void host_pos(const nav_msgs::Odometry::ConstPtr& msg)
{
    double roll, pitch, yaw;
    host_mocap.header = msg->header;
    host_mocap.pose.position = msg->pose.pose.position;
    host_mocap.pose.orientation = msg->pose.pose.orientation;

    tf::Quaternion Q(
        host_mocap.pose.orientation.x,
        host_mocap.pose.orientation.y,
        host_mocap.pose.orientation.z,
        host_mocap.pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}
#endif

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic");
    ros::NodeHandle nh;

    ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 2);
#ifdef Mocap
    std::string sub_topic;
    ros::param::get("sub_topic", sub_topic);
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>(sub_topic, 10, host_pos);
#else
//    ros::Subscriber host_sub = nh.subscribe<nav_msgs::Odometry> ("/vins_estimator/odometry",2, host_pos);
    ros::Subscriber host_sub = nh.subscribe<nav_msgs::Odometry> ("/estimator/imu_propagate", 2, host_pos);
#endif
    //recommend less than 50HZ
    ros::Rate rate(30);

    while (ros::ok()) {

        ROS_INFO("odom: %.3f, %.3f, %.3f", host_mocap.pose.position.x, host_mocap.pose.position.y, host_mocap.pose.position.z);
        mocap_pos_pub.publish(host_mocap);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


