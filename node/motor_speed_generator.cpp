#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
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
    float kp_acc;
    float delta_t;
    struct ACC
    {
        float x;
        float y;
        float z;
    }acc;
    geometry_msgs::TwistStamped desired_vel;
    geometry_msgs::TwistStamped current_vel;
    std_msgs::Float64 motorSpd_R;
    std_msgs::Float64 motorSpd_L;
    ros::Subscriber desired_vel_sub;

public:
    Mobile();
    Mobile(ros::NodeHandle nh, string desired_vel_subTopic);
    void setMobileParam(float w, float r);
    void setPdCtrlParam(float kp, float kd);
    void setViutualInputParam(float kp);
    void desired_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void computeWheelSpd();
    void computeAcc();
    std_msgs::Float64 getMotorSpd_L();
    std_msgs::Float64 getMotorSpd_R();
};

Mobile::Mobile(ros::NodeHandle nh, string desired_vel_subTopic)
{
    desired_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(desired_vel_subTopic, 10, &Mobile::desired_vel_cb, this);
    width = 1;
    wheelRadius = 0.0325;
    kp_omega = 1;
    kd_omega = 1;
    kp_acc = 1;
    delta_t = 0.005;
}

void Mobile::computeAcc()
{
    acc.x = kp_acc*(desired_vel.twist.linear.x - current_vel.twist.linear.x);
    acc.y = kp_acc*(desired_vel.twist.linear.y - current_vel.twist.linear.y);
    //cout << "accx " << acc.x << " accy " << acc.y << endl;
}

void Mobile::desired_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{  
    desired_vel.twist.linear.x = msg->twist.linear.x;
    desired_vel.twist.linear.y = msg->twist.linear.y;
    computeAcc();
    current_vel.twist.linear.x = current_vel.twist.linear.x + acc.x*delta_t;
    current_vel.twist.linear.y = current_vel.twist.linear.y + acc.y*delta_t;
    computeWheelSpd();

    if(abs(omega_L) < 0.01 && abs(omega_R) < 0.01)
        omega_L = omega_R = 0;

    cout << "L: " << omega_L << " R: " << omega_R << endl;

    motorSpd_L.data = omega_L;
    motorSpd_R.data = omega_R;    
    while(abs(motorSpd_L.data) > 3.0 || abs(motorSpd_R.data) > 3.0)
    {
        motorSpd_L.data = motorSpd_L.data*0.9;
        motorSpd_R.data = motorSpd_R.data*0.9;
    }
}

void Mobile::computeWheelSpd()
{
    float v_norm = sqrt(pow(current_vel.twist.linear.x, 2) + pow(current_vel.twist.linear.y, 2));
    if(current_vel.twist.linear.y >= 0)
        theta = acos(current_vel.twist.linear.x/v_norm);
    else
        theta = -acos(current_vel.twist.linear.x/v_norm);

    // PD control of self spinning
    omega_self = kp_omega*theta;

    omega_L = (v_norm-omega_self*width/2)/wheelRadius;
    omega_R = (v_norm+omega_self*width/2)/wheelRadius;
}

void Mobile::setMobileParam(float w, float r){width = w; wheelRadius = r;}
void Mobile::setPdCtrlParam(float kp, float kd){kp_omega = kp; kd_omega = kd;}
void Mobile::setViutualInputParam(float kp){kp_acc = kp;}
std_msgs::Float64 Mobile::getMotorSpd_L(){return motorSpd_L;}
std_msgs::Float64 Mobile::getMotorSpd_R(){return motorSpd_R;}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_speed_generation");
    ros::NodeHandle nh;
    ros::Rate rate(200);

    ros::Publisher motorSpd_L_pub = nh.advertise<std_msgs::Float64>("/motor_speed_L", 2);
    ros::Publisher motorSpd_R_pub = nh.advertise<std_msgs::Float64>("/motor_speed_R", 2);

    Mobile car(nh, "/track/vel");
    car.setMobileParam(0.11, 0.0325);
    car.setPdCtrlParam(0.7, 1);
    car.setViutualInputParam(1.0);

    while(ros::ok())
    {
        motorSpd_L_pub.publish(car.getMotorSpd_L());
        motorSpd_R_pub.publish(car.getMotorSpd_R());
        ros::spinOnce();

        rate.sleep();
    }
    
    return 0;
}
