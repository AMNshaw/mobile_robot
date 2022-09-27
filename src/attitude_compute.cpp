#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include "math.h"
#define pi 3.1415926
bool calibrate = false;
float dt = 0.02;
float beta = 1;
float beta2 = 0.1;
float beta3 = 0.01;
float avg_ax = 0, avg_ay = 0, avg_az = 0, avg_wx = 0, avg_wy = 0, avg_wz = 0;
float avg_mx = 0, avg_my = 0, avg_mz = 0;
geometry_msgs::Quaternion Q;
geometry_msgs::Quaternion Q2;
geometry_msgs::Quaternion Q3;

sensor_msgs::Imu imu;
void compute(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu = *msg;
}
geometry_msgs::Vector3 mag;
sensor_msgs::MagneticField temp;
geometry_msgs::Vector3 scale;
void calib(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    temp = *msg;
    mag = temp.magnetic_field;
}
geometry_msgs::Vector3 result(geometry_msgs::Vector3 w, geometry_msgs::Vector3 a, geometry_msgs::Quaternion *Q, float beta){
    geometry_msgs::Quaternion qdot;
    geometry_msgs::Vector3 f;
    geometry_msgs::Vector3 normalized_acc;
    double roll,pitch,yaw;

    qdot.w = 0.5*(-w.x*Q->x - w.y*Q->y - w.z*Q->z);
    qdot.x = 0.5*( w.x*Q->w + w.z*Q->y - w.y*Q->z);
    qdot.y = 0.5*( w.y*Q->w - w.z*Q->x + w.x*Q->z);
    qdot.z = 0.5*( w.z*Q->w + w.y*Q->x - w.x*Q->y);

    float accel_norm = 1 / (sqrt(a.x*a.x + a.y*a.y + a.z*a.z));
    a.x *= accel_norm;
    a.y *= accel_norm;
    a.z *= accel_norm;

    float _2q0 = 2.0f*Q->w;
    float _2q1 = 2.0f*Q->x;
    float _2q2 = 2.0f*Q->y;
    float _2q3 = 2.0f*Q->z;
    float _4q0 = 4.0f*Q->w;
    float _4q1 = 4.0f*Q->x;
    float _4q2 = 4.0f*Q->y;
    float _4q3 = 4.0f*Q->z;
    float q0q0 = Q->w*Q->w;
    float q1q1 = Q->x*Q->x;
    float q2q2 = Q->y*Q->y;
    float q3q3 = Q->z*Q->z;
    float q1q1_q2q2 = Q->x*Q->x + Q->y*Q->y;

    float g0 = _4q0*q1q1_q2q2 + _2q2*a.x - _2q1*a.y;
    float g1 = _4q1*q1q1_q2q2 - _2q3*a.x - _2q0*a.y + _4q1*a.z;
    float g2 = _4q2*q1q1_q2q2 + _2q0*a.x - _2q3*a.y + _4q2*a.z;
    float g3 = _4q3*q1q1_q2q2 - _2q1*a.x - _2q2*a.y;

    float g_norm = 1 / (sqrt(g0*g0 + g1*g1 + g2*g2 + g3*g3));
    g0 *= g_norm;
    g1 *= g_norm;
    g2 *= g_norm;
    g3 *= g_norm;

    qdot.w -= beta*g0;
    qdot.x -= beta*g1;
    qdot.y -= beta*g2;
    qdot.z -= beta*g3;

    Q->w += qdot.w*dt;
    Q->x += qdot.x*dt;
    Q->y += qdot.y*dt;
    Q->z += qdot.z*dt;

    float q_norm = 1 / (sqrt(Q->w*Q->w + Q->x*Q->x + Q->y*Q->y + Q->z*Q->z));
    Q->w *= q_norm;
    Q->x *= q_norm;
    Q->y *= q_norm;
    Q->z *= q_norm;

    tf::Quaternion attitude(
          Q->x,
          Q->y,
          Q->z,
          Q->w);
    tf::Matrix3x3(attitude).getRPY(roll,pitch,yaw);
    mag.x = (mag.x - avg_mx)*scale.x;
    mag.y = (mag.y - avg_my)*scale.y;
    mag.z = (mag.z - avg_mz)*scale.z;
    float Xh = mag.x * cos(-pitch) + mag.y * sin(-roll)*sin(-pitch) - mag.z * cos(-roll)*sin(-pitch);
    float Yh = mag.y * cos(-roll) + mag.z * sin(-roll);
    yaw = atan2(Yh,Xh);
    ROS_INFO("euler: %.2f, %.2f, %.2f",roll/pi*180,pitch/pi*180,yaw/pi*180);

    geometry_msgs::Vector3 euler;
    euler.x = roll/pi*180;
    euler.y = pitch/pi*180;
    euler.z = yaw/pi*180;
    return euler;
}
int main(int argc , char **argv)
{

	ros::init(argc , argv , "imutf");	
	ros::NodeHandle n;
  ros::Subscriber mag_sub = n.subscribe<sensor_msgs::MagneticField> ("/mavros/imu/mag",10,calib);
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu> ("/mavros/imu/data_raw",10,compute);
  ros::Publisher rqt_pub = n.advertise<geometry_msgs::Vector3>("/angle", 10);
  ros::Publisher rqt2_pub = n.advertise<geometry_msgs::Vector3>("/angle2", 10);
  ros::Publisher rqt3_pub = n.advertise<geometry_msgs::Vector3>("/angle3", 10);

	ros::Rate loop_rate(50);
  float sum_ax = 0,sum_ay = 0, sum_az = 0, sum_wx = 0,sum_wy = 0, sum_wz = 0;
  int count = 0, i = 0, num = 100 ;
  geometry_msgs::Vector3 max_mag;
  geometry_msgs::Vector3 min_mag;
  geometry_msgs::Vector3 diff;
  float diff_avg = 0;

  Q.x = Q2.x = Q3.x = 0;
  Q.y = Q2.y = Q3.y = 0;
  Q.z = Q2.z = Q3.z = 0;
  Q.w = Q2.w = Q3.w = 1;

  while(ros::ok() && calibrate == false){
//    ROS_WARN("Start Calibration!");
    if(i<num){
      if (imu.linear_acceleration.x !=0 && imu.linear_acceleration.y !=0 && imu.angular_velocity.x !=0 && imu.angular_velocity.y !=0 && imu.angular_velocity.z !=0){
        ROS_INFO("%d",i);
        i += 1;
        sum_ax += imu.linear_acceleration.x;
        sum_ay += imu.linear_acceleration.y;
        sum_az += imu.linear_acceleration.z;
        sum_wx += imu.angular_velocity.x;
        sum_wy += imu.angular_velocity.y;
        sum_wz += imu.angular_velocity.z;

        count += 1;
        avg_ax = (sum_ax / count) - 0;
        avg_ay = (sum_ay / count) - 0;
        avg_az = (sum_az / count) - 9.81;
        avg_wx = sum_wx / count;
        avg_wy = sum_wy / count;
        avg_wz = sum_wz / count;

        if(mag.x>max_mag.x){
          max_mag.x=mag.x;
          }
        if(mag.y>max_mag.y){
          max_mag.y=mag.y;
          }
        if(mag.z>max_mag.z){
          max_mag.z=mag.z;
          }
        if(mag.x<min_mag.x){
          min_mag.x=mag.x;
          }
        if(mag.y<min_mag.y){
          min_mag.y=mag.y;
          }
        if(mag.z<min_mag.z){
          min_mag.z=mag.z;
          }
        avg_mx = 0.5*(max_mag.x + min_mag.x);
        avg_my = 0.5*(max_mag.y + min_mag.y);
        avg_mz = 0.5*(max_mag.z + min_mag.z);
        diff.x = 0.5*(max_mag.x - min_mag.x);
        diff.y = 0.5*(max_mag.y - min_mag.y);
        diff.z = 0.5*(max_mag.z - min_mag.z);
        diff_avg = (diff.x + diff.y + diff.z)/3;
        scale.x = diff_avg/diff.x;
        scale.y = diff_avg/diff.y;
        scale.z = diff_avg/diff.z;
//        ROS_INFO("diff = %f,%f,%f", diff.x, diff.y, diff.z);
//        ROS_INFO("scale = %f,%f,%f", scale.x, scale.y, scale.z);
      }
      ROS_INFO("a = %f, %f, %f", avg_ax, avg_ay, avg_az);
      ROS_INFO("w = %f, %f, %f", avg_wx, avg_wy, avg_wz);
      ROS_INFO("m = %f, %f, %f", avg_mx, avg_my, avg_mz);
    }
    else{
        calibrate = true;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  while (ros::ok() && calibrate == true)
	{
    ros::spinOnce();
    geometry_msgs::Vector3 w;
    geometry_msgs::Vector3 a;

    w.x = imu.angular_velocity.x - avg_wx;
    w.y = imu.angular_velocity.y - avg_wy;
    w.z = imu.angular_velocity.z - avg_wz;
    a.x = imu.linear_acceleration.x - avg_ax;
    a.y = imu.linear_acceleration.y - avg_ay;
    a.z = imu.linear_acceleration.z - avg_az;

    geometry_msgs::Vector3 angle;
    geometry_msgs::Vector3 angle2;
    geometry_msgs::Vector3 angle3;
    angle = result(w,a,&Q,beta);
    angle2 = result(w,a,&Q2,beta2);
    angle3 = result(w,a,&Q3,beta3);

    rqt_pub.publish(angle);
    rqt2_pub.publish(angle2);
    rqt3_pub.publish(angle3);

    loop_rate.sleep();
	}
    return 0;
}


