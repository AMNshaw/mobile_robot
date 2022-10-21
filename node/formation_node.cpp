#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "getch.h"
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#define gravity 9.806
#define UAV_ID 2
using namespace std;

bool init = false;
bool start = false;
//set control P-gain
double KPx=1, KPy=1, KPz=1.2;
//float KPx=5, KPy=5, KPz=1.2;
double KPyaw = 1;
double roll = 0, pitch = 0, yaw = 0;


geometry_msgs::PoseStamped leader_pose;
geometry_msgs::PoseStamped MAV_pose[5];

void leader_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    MAV_pose[0].header = msg->header;
    MAV_pose[0].pose.position = msg->pose.position;
    MAV_pose[0].pose.orientation = msg->pose.orientation;
}
void mav1_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    MAV_pose[1].header = msg->header;
    MAV_pose[1].pose.position = msg->pose.position;
    MAV_pose[1].pose.orientation = msg->pose.orientation;
}
void mav2_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    MAV_pose[2].header = msg->header;
    MAV_pose[2].pose.position = msg->pose.position;
    MAV_pose[2].pose.orientation = msg->pose.orientation;
}
void mav3_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    MAV_pose[3].header = msg->header;
    MAV_pose[3].pose.position = msg->pose.position;
    MAV_pose[3].pose.orientation = msg->pose.orientation;
}
void mav4_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    MAV_pose[4].header = msg->header;
    MAV_pose[4].pose.position = msg->pose.position;
    MAV_pose[4].pose.orientation = msg->pose.orientation;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation");
    ros::NodeHandle nh;
    
    //Subscriber
    ros::Subscriber leader_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/leader_pose", 10,leader_pose_cb);    
    ros::Subscriber mav1_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV1/pose", 10, mav1_cb);
    ros::Subscriber mav2_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV2/pose", 10, mav2_cb);
    ros::Subscriber mav3_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV3/pose", 10, mav3_cb);
    ros::Subscriber mav4_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV4/pose", 10, mav4_cb);

    //Publisher    
    ros::Publisher desired_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("desired_velocity_raw", 100);

    bool laplacian_map[5][5] = { 1,   0,  0,  0,  0,
                                 1,   1,  0,  0,  0,
                                 1,   1,  1,  0,  0,
                                 1,   1,  1,  1,  0,
                                 1,   1,  1,  1,  1
                               };
    float leader_uav_vector_x[5] = {0,0.5,-0.5,-0.5,0.5 };  //vector x from leader to uav
    float leader_uav_vector_y[5] = {0,0.5,0.5 ,-0.5,-0.5};  //vector y from leader to uav
    float relative_map_x[5][5];
    float relative_map_y[5][5];
    for(int i = 0 ; i<5; i++){
        for(int j = 0 ; j<5 ; j++){
            relative_map_x[i][j] = leader_uav_vector_x[i] - leader_uav_vector_x[j];
            relative_map_y[i][j] = leader_uav_vector_y[i] - leader_uav_vector_y[j];
        }
    }
    cout << "relative map x\n";
    for(int i = 0;i<5;i++){
        for(int j=0;j<5;j++){
            cout << relative_map_x[i][j] << "\t";
        }
        cout << "\n";
    }
    cout << "relative map y\n";
    for(int i = 0;i<5;i++){
        for(int j=0;j<5;j++){
            cout << relative_map_y[i][j] << "\t";
        }
        cout << "\n";
    }
    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(100);

    geometry_msgs::TwistStamped desired_vel;

    while (ros::ok()) {
        desired_vel.twist.linear.x = 0;
        desired_vel.twist.linear.y = 0;
        desired_vel.twist.linear.z = 0;
        for(int i =0 ;i<5;i++){
            if(laplacian_map[UAV_ID][i] == 1){
                desired_vel.twist.linear.x += MAV_pose[i].pose.position.x - MAV_pose[UAV_ID].pose.position.x + relative_map_x[UAV_ID][i] ;
                desired_vel.twist.linear.y += MAV_pose[i].pose.position.y - MAV_pose[UAV_ID].pose.position.y + relative_map_y[UAV_ID][i] ;
                desired_vel.twist.linear.z += MAV_pose[i].pose.position.z - MAV_pose[UAV_ID].pose.position.z;
            }
        }
        desired_vel_pub.publish(desired_vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



