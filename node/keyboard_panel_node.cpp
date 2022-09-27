#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "getch.h"
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>

bool init = false;
geometry_msgs::PoseStamped initial_pose;
geometry_msgs::PoseStamped host_mocap;



void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    host_mocap.header = msg->header;
    host_mocap.pose.position = msg->pose.position;
    host_mocap.pose.orientation = msg->pose.orientation;

    //store intial pose for first callback
    if(init == false){
        initial_pose = host_mocap;
        init = true;
    }

}
void bound_yaw(double* yaw){
        if(*yaw>M_PI)
        	*yaw = *yaw - 2*M_PI;
        else if(*yaw<-M_PI)
        	*yaw = *yaw + 2*M_PI;
}
int main(int argc, char **argv)
{
    //  ROS_initialize  //
    ros::init(argc, argv, "keyboard_panel");
    ros::NodeHandle nh;
    //    subscriber    //
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV1/pose", 10, host_pos);
    
    //    publisher    //
    ros::Publisher desired_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("desired_pose", 10);

    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(50);
    
    // cmd_msgs
    geometry_msgs::PoseStamped desired_pose;
    double desired_yaw = 0;
    
    //init desired_vel & desired_pos
    desired_pose.pose.position.x = 0;
    desired_pose.pose.position.y = 0;
    desired_pose.pose.position.z = 0;
    desired_pose.pose.orientation.x = 0;
    desired_pose.pose.orientation.y = 0;
    desired_pose.pose.orientation.z = 0;
    desired_pose.pose.orientation.w = 0;
    

    //get init desired_pose
    ROS_INFO("wait for initial pose ");
    while(ros::ok() && !init){
    	ros::spinOnce();
        rate.sleep();
    };
    desired_pose = initial_pose; 
    desired_pose.pose.position.z+= 0.5;
     tf::Quaternion initial_Q(
          initial_pose.pose.orientation.x,
          initial_pose.pose.orientation.y,
          initial_pose.pose.orientation.z,
          initial_pose.pose.orientation.w);
    double init_roll,init_pitch,init_yaw;
    tf::Matrix3x3(initial_Q).getRPY(init_roll,init_pitch,init_yaw);
    desired_yaw = init_yaw;
    bound_yaw(&desired_yaw);
    ROS_INFO("initilized");
    
    while (ros::ok()) {

        //keyboard control
        int c = getch();
        //ROS_INFO("C: %d",c);
        //update desired pose
        if (c != 0) {
          
            switch (c) {
                case 65:    // key up
                    desired_pose.pose.position.z += 0.05;
                    break;
                case 66:    // key down
                    desired_pose.pose.position.z += -0.05;
                    break;
                case 67:    // key CW(->)
                    desired_yaw -= 0.03;
                    bound_yaw(&desired_yaw); 
                    break;
                case 68:    // key CCW(<-)
                    desired_yaw += 0.03;
                    bound_yaw(&desired_yaw); 
                    break;
                case 119:    // key foward(w)
                    desired_pose.pose.position.y += 0.05;
                    break;
                case 120:    // key back(x)
                    desired_pose.pose.position.y -= 0.05;
                    break;
                case 97:    // key left(a)
                    desired_pose.pose.position.x -= 0.05;
                    break;
                case 100:    // key right(d)
                    desired_pose.pose.position.x += 0.05;
                    break;
                case 114:    // key (r) re-init desired_pose to current pose
                {
                    desired_pose = host_mocap;
                    desired_pose.pose.position.z+= 0.5;
                    tf::Quaternion Q(
                      host_mocap.pose.orientation.x,
                      host_mocap.pose.orientation.y,
                      host_mocap.pose.orientation.z,
                      host_mocap.pose.orientation.w);
                    tf::Matrix3x3(Q).getRPY(init_roll,init_pitch,init_yaw);
                    desired_yaw = init_yaw;
                    bound_yaw(&desired_yaw);
                    break;
                }
                case 115:    // key origin(s)
                {
                    desired_pose.pose.position.z = 0.5;
                    break;
                }
                case 63:
                return 0;
                break;
            }
            ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z, desired_yaw/M_PI*180);
        }
        //input desired position and measurement, may plus feedforward velocity
        //output control input vs
        desired_pose.pose.orientation = tf::createQuaternionMsgFromYaw(desired_yaw);
        desired_pose.header.stamp = ros::Time::now();

        desired_pos_pub.publish(desired_pose);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
