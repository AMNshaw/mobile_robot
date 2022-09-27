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
using namespace std;

bool init = false;
bool start = false;
//set control P-gain
double KPx=1, KPy=1, KPz=1.2;
//float KPx=5, KPy=5, KPz=1.2;
double KPyaw = 1;
double roll = 0, pitch = 0, yaw = 0;


geometry_msgs::PoseStamped leader_pose;
geometry_msgs::PoseStamped[5] MAV_pose;

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

void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped* vs, float vx, float vy, float ax, float ay)
{
    double errx, erry, errz, err_yaw;
    double ux, uy, uz, uyaw;

    //compute error: desired - measurement
    errx = vir.x - host_mocap.pose.position.x;
    erry = vir.y - host_mocap.pose.position.y;
    errz = vir.z - host_mocap.pose.position.z;
    err_yaw = vir.yaw - yaw;

    if(err_yaw>M_PI)
    err_yaw = err_yaw - 2*M_PI;
    else if(err_yaw<-M_PI)
    err_yaw = err_yaw + 2*M_PI;

    ROS_INFO("err: %.3f,%.3f,%.3f,%.3f", errx, erry, errz, err_yaw/M_PI*180);

    if(start == false){
        ux = KPx*errx;
        uy = KPy*erry;
        uz = KPz*errz;
        uyaw = KPyaw*err_yaw;
    }
    else{
        //feedback + feedforward control
        ux = KPx*errx + vx;
        uy = KPy*erry + vy;
        uz = KPz*errz;
        uyaw = KPyaw*err_yaw;
    }

    //set max&min for control input
    if(ux<=-1.5 ||ux>=1.5)
    {
      ux = 1.5*ux/abs(ux);
    }
    if(uy<=-1.5 ||uy>=1.5)
    {
      uy = 1.5*uy/abs(uy);
    }
    if(uz<=-0.4 ||uz>=0.4)
    {
      uz = 0.4*uz/abs(uz);
    }
    //output control input
    vs->twist.linear.x = ux;
    vs->twist.linear.y = uy;
    vs->twist.linear.z = uz;
    vs->twist.angular.z = uyaw;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation");
    ros::NodeHandle nh;
    int MAV_self_id = 1;
    int MAV_connect_num =1;
    int MAV_connect_id[2];
    MAV_connect_id[0] = 2;
    
    string MAV_self_topic = "/vrpn_client_node/MAV" + MAV_self_id.toString() + "/pose";
    for(int i=0; i<MAV_connect_num ; i++){
    }
    string MAV_self_topic = "/vrpn_client_node/MAV" + MAV_self_id.toString() + "/pose";

    
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV1/pose", 10, host_pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
//    ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 2);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV1/pose", 10, host_pos);
    //output final command to flight controller
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 2);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/desired_position", 2);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/desired_velocity", 2);

    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(50);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
        //mocap_pos_pub.publish(host_mocap);
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::PoseStamped desired_pos;
    geometry_msgs::TwistStamped desired_vel;

    //initialize desired state
    vir vir1;
    vir1.x = 0;
    vir1.y = 0;
    vir1.z = 0.5;
    vir1.yaw = 0;
    //initialize control input
    geometry_msgs::TwistStamped vs;
    vs.twist.linear.x = 0;
    vs.twist.linear.y = 0;
    vs.twist.linear.z = 0;
    vs.twist.angular.x = 0;
    vs.twist.angular.y = 0;
    vs.twist.angular.z = 0;
    double T = 2*M_PI;
    double r = 0.5, a = 1;
    double t = 0, dt = 0.02;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(vs);
        vir1.x = 0;
        vir1.y = 0;
        vir1.z = 0.5;
        vir1.yaw = yaw;
        ROS_INFO("initial_pose: %3f, %3f, %3f", initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z);

        ros::spinOnce();
        rate.sleep();
    }
    //set offboard mode and ready to arm
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    //ros::Time last_request(0);

    while (ros::ok()) {
        //mocap_pos_pub.publish(host_mocap);
        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //keyboard control
        int c = getch();
        //ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
                case 65:    // key up
                    vir1.z += 0.05;
                    break;
                case 66:    // key down
                    vir1.z += -0.05;
                    break;
                case 67:    // key CW(->)
                    vir1.yaw -= 0.03;
                    break;
                case 68:    // key CCW(<-)
                    vir1.yaw += 0.03;
                    break;
                case 119:    // key foward(w)
                    vir1.x += 0.05;
                    break;
                case 120:    // key back(x)
                    vir1.x += -0.05;
                    break;
                case 97:    // key left(a)
                    vir1.y += 0.05;
                    break;
                case 100:    // key right(d)
                    vir1.y -= 0.05;
                    break;
                case 115:    // key origin(s)
                {
//                    vir1.x = initial_pose.pose.position.x;
//                    vir1.y = initial_pose.pose.position.y;
                    vir1.z = initial_pose.pose.position.z + 0.1;
                    break;
                }
                case 108:    // closed arming(l)
                {
                    offb_set_mode.request.custom_mode = "MANUAL";
                    set_mode_client.call(offb_set_mode);
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);
                    break;
                }
                case 116:    // (t)
                {
                    if(start == true){
                    start = false;
                    }
                    else if(start == false){
                    start = true;
                    }
                    break;
                }
                case 105:    // i
                {


                break;
                }
                case 63:
                return 0;
                break;
            }
        }

        if(vir1.yaw>M_PI)
        vir1.yaw = vir1.yaw - 2*M_PI;
        else if(vir1.yaw<-M_PI)
        vir1.yaw = vir1.yaw + 2*M_PI;

        if(start == true){
            //circular trajectory: r is amplitude of circle, T is the period
            vir1.x = r*cos(2*M_PI*t/T);
            vir1.y = r*sin(2*M_PI*t/T);

            desired_vel.twist.linear.x = -r*sin(2*M_PI*t/T)*2*M_PI/T;
            desired_vel.twist.linear.y = r*cos(2*M_PI*t/T)*2*M_PI/T;
            desired_vel.twist.linear.z = 0;
            t += dt;
        }
        else{
            desired_vel.twist.linear.x = 0;
            desired_vel.twist.linear.y = 0;
            desired_vel.twist.linear.z = 0;
        }
        ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z, vir1.yaw/M_PI*180);
        //input desired position and measurement, may plus feedforward velocity
        //output control input vs
        follow(vir1, host_mocap, &vs, desired_vel.twist.linear.x, desired_vel.twist.linear.y, 0, 0);

        //update desired position and velocity
        desired_pos.header.stamp = ros::Time::now();
        desired_pos.pose.position.x = vir1.x;
        desired_pos.pose.position.y = vir1.y;
        desired_pos.pose.position.z = vir1.z;
        desired_pos.pose.orientation = tf::createQuaternionMsgFromYaw(vir1.yaw);
        desired_vel.header.stamp = ros::Time::now();

        //mocap_pos_pub.publish(host_mocap);
        local_vel_pub.publish(vs);
        pos_pub.publish(desired_pos);
        vel_pub.publish(desired_vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



