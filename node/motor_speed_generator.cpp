#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>
#include <eigen3/Eigen/Dense>

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
    std_msgs::Float32MultiArray motorSpd;
    ros::Subscriber desired_vel_sub;

public:
    Mobile();
    Mobile(ros::NodeHandle nh, string desired_vel_subTopic);
    void setMobileParam(float w, float r);
    void setPdCtrlParam(float kp, float kd);
    void desired_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void computeWheelSpd();
    std_msgs::Float32MultiArray getMotorSpd();
};

Mobile::Mobile(ros::NodeHandle nh, string desired_vel_subTopic)
{
    desired_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(desired_vel_subTopic, 10, &Mobile::desired_vel_cb, this);
    width = 1;
    wheelRadius = 0.0325;
    kp_omega = 1;
    kd_omega = 1;
}

void Mobile::desired_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{  
    desired_vel.twist.linear.x = msg->twist.linear.x;
    desired_vel.twist.linear.y = msg->twist.linear.y;
    computeWheelSpd();
}

void Mobile::computeWheelSpd()
{
    float v_norm = sqrt(pow(desired_vel.twist.linear.x, 2) + pow(desired_vel.twist.linear.y, 2));
    theta = acos(desired_vel.twist.linear.x/v_norm);

    // PD control of self spinning
    omega_self = kp_omega*theta;

    omega_L = (v_norm-omega_self*width/2)/wheelRadius;
    omega_R = (v_norm+omega_self*width/2)/wheelRadius;

    motorSpd.data.push_back(omega_L);
    motorSpd.data.push_back(omega_R);
}

void Mobile::setMobileParam(float w, float r){width = w; wheelRadius = r;}
void Mobile::setPdCtrlParam(float kp, float kd){kp_omega = kp; kd_omega = kd;}
std_msgs::Float32MultiArray Mobile::getMotorSpd(){return motorSpd;}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_speed_generation");
    ros::NodeHandle nh;
    ros::Rate rate(100);

    ros::Publisher motorSpd_pub = nh.advertise<std_msgs::Float32MultiArray>("/motor_speed", 2);

    Mobile car(nh, "/cmd/vel");
    car.setMobileParam(0.11, 0.0325);
    car.setPdCtrlParam(1, 1);

    while(ros::ok())
    {
        motorSpd_pub.publish(car.getMotorSpd());
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
