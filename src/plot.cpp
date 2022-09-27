#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Point pos;
geometry_msgs::Vector3 vel;
geometry_msgs::Point acc;
void position(const nav_msgs::Odometry::ConstPtr& msg) {
    pos = msg->pose.pose.position;
}
/*void position(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pos = msg->pose.position;
}*/
void velocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    vel = msg->twist.linear;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "qptest");
  ros::NodeHandle nh;

  ros::Publisher pos_pub = nh.advertise<geometry_msgs::Point>("/pos",10);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Point>("/vel",10);
  //ros::Publisher acc_pub = nh.advertise<geometry_msgs::Point>("/acc",10);
  //ros::Subscriber desired_pos = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody7/pose", 2,position);
  ros::Subscriber desired_pos = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/imu_propagate", 2,position);
  //ros::Subscriber desired_vel = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 2,velocity);

  double max;
  double count ;
  ros::Rate loop_rate(50);

  while(ros::ok()){

//    ROS_INFO("%f,%f",count,max);
    //acc_pub.publish(acc);
    vel_pub.publish(vel);
    pos_pub.publish(pos);

    ros::spinOnce();
    loop_rate.sleep();
  }

}
