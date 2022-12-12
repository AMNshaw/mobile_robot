#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

using namespace std;

class Mobile
{
private:
    float width;
    float wheelRadius;
    float omega_L;
    float omega_R;
    float omega_self;
    float theta;
    float kp_omega;
    float kd_omega;
    geometry_msgs::TwistStamped desired_vel;
    ros::Subscriber desired_vel;
public:
    Mobile();
    Mobile();
    void desired_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void computeWheelSpd();
}

void Mobile::computeWheelSpd()
{
    float v_norm = sqrt(pow(desired_vel.twist.linear.x, 2) + pow(desired_vel.twist.linear.y, 2));
    theta = acos(desired_vel.twist.linear.x/v_norm);
    omega_self = kp_omega*theta;

    omega_L = (v_norm-omega_self)*width/2;
    omega_R = (v_norm+omega_self)*width/2
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_speed_generation");
    ros::NodeHandle nh;

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("cmd/vel", 2);

    ros::Rate rate(100);

    CBF cbf(nh, "/aprilTag_pos");
    cbf.setCBFparam(0.3, 0.2, 0.6);
    geometry_msgs::TwistStamped desired_vel;
    geometry_msgs::TwistStamped desired_vel_raw;
    desired_vel.twist.linear.x = 0;
    desired_vel.twist.linear.y = 0;
    desired_vel_raw.twist.linear.x = 0;
    desired_vel_raw.twist.linear.y = 0;

    
    while(ros::ok())
    {
        cbf.QPsolve_vel(desired_vel_raw , &desired_vel);
        ros::spinOnce();
        rate.sleep();
        
        cout << desired_vel << endl;
    }
    
    return 0;
}
