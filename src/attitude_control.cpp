/**
 * @file offb_main.cpp
 * @author Julian Oes <julian@oes.ch>
 * @license BSD 3-clause
 *
 * @brief ROS node to do offboard control of PX4 through MAVROS.
 *
 * Initial code taken from http://dev.px4.io/ros-mavros-offboard.html
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#define gravity 9.806
#define pi 3.1415926
bool init = false;
bool initial = false;
bool start = false;
int flag=0;
float KPx=10, KPy=10, KPz=0.6;
float KVx=3.3, KVy=3.3, KVz=0.2;
float KIx=0.33, KIy=0.33, KIz=0.05;
float KPyaw = 1;
double roll, pitch, yaw;
float r = 0.5;
float T = 2*pi;
using namespace std;
typedef struct
{
    float roll;
    float x;
    float y;
    float z;
}vir;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
geometry_msgs::PoseStamped host_mocap;
geometry_msgs::PoseStamped initial_pose;
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        host_mocap = *msg;
        if(init==false)
        {
        initial_pose = host_mocap;
        init = true;
        }
        tf::Quaternion Q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

        tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
//        ROS_INFO("angle: %.2f,%.2f, %.2f",roll,pitch,yaw);
//        tf::quaternionTFToMsg(Q,desired.pose.orientation);
//        ROS_INFO("error: %.4f, %.4f, %.4f, %.4f ",desired.pose.orientation.x,desired.pose.orientation.y,desired.pose.orientation.z,desired.pose.orientation.w);

}
float sumx,sumy,sumz;
geometry_msgs::Point last_pos;
geometry_msgs::Point velocity;
void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, mavros_msgs::AttitudeTarget* pose,float vx,float vy,float ax,float ay)
{
float errx, erry, errz, err_roll;
float errvx , errvy ,errvz ;
float ux, uy, uz, uroll, upitch, uyaw;
float ubx, uby;
tf::Quaternion Q;

errx = vir.x - host_mocap.pose.position.x;
erry = vir.y - host_mocap.pose.position.y;
errz = vir.z - host_mocap.pose.position.z;
velocity.x = (host_mocap.pose.position.x-last_pos.x)/0.02;
velocity.y = (host_mocap.pose.position.y-last_pos.y)/0.02;
velocity.z = (host_mocap.pose.position.z-last_pos.z)/0.02;
//errvx = vx - host_mocapvel.twist.linear.x;
//errvy = vy - host_mocapvel.twist.linear.y;
//errvz = 0 - host_mocapvel.twist.linear.z;
errvx = vx - velocity.x;
errvy = vy - velocity.y;
errvz = 0 - velocity.z;
if(start==false){
sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;
}
last_pos = host_mocap.pose.position;
if(sumz>0.2){
  sumz = 0.2; 
}
else if(sumz<-0.2){
  sumz = -0.2;
}
if(sumx>0.6){
  sumx = 0.6;
}
else if(sumx<-0.6){
  sumx = -0.6;
}
if(sumy>0.6){
  sumy = 0.6;
}
else if(sumy<-0.6){
  sumy = -0.6;
}
err_roll = vir.roll - yaw;
if(err_roll>pi)
err_roll = err_roll - 2*pi;
else if(err_roll<-pi)
err_roll = err_roll + 2*pi;

ROS_INFO("err: %.3f,%.3f,%.3f,%.3f",errx,erry,errz,err_roll);
//ROS_INFO("err: %.3f,%.3f,%.3f",errvx,errvy,errvz);

if(start == false){
ux = KPx*errx + KVx*errvx + sumx;
uy = KPy*erry + KVy*errvy + sumy;
uz = KPz*errz + KVz*errvz + sumz;
}
else{
//    ux = KPx*errx + KVx*errvx + sumx + (-(4*pi*pi/T/T)*(4*pi*pi/T/T)*r*cos(2*pi*t/T));
//    uy = KPy*erry + KVy*errvy + sumy + (-(4*pi*pi/T/T)*(4*pi*pi/T/T)*r*sin(2*pi*t/T));
    ux = KPx*errx + KVx*errvx + sumx + ax;
    uy = KPy*erry + KVy*errvy + sumy + ay;
    uz = KPz*errz + KVz*errvz + sumz;
}
ubx = cos(-yaw) * ux - sin(-yaw) * uy;
uby = sin(-yaw) * ux + cos(-yaw) * uy;
uroll = -atan2(uby,gravity);
upitch = atan2(ubx,gravity);
uyaw = KPyaw*err_roll;
//uroll = 0;
//upitch = 0;
//uyaw = 0;
Q.setRPY(uroll, upitch, uyaw);
tf::quaternionTFToMsg(Q,pose->orientation);
pose->thrust = uz + 0.2;
ROS_INFO("u: %.4f,%.4f,%.4f,%.4f",uroll,upitch,uyaw,pose->thrust);

}
/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "track");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/mavros/setpoint_position/local", 10);
    //ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   //("/mavros/mocap/pose", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/mavros/set_mode");
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody7/pose", 10, host_pos);
    ros::Publisher local_vel_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 2);
    //ros::Subscriber host2_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 1, host_vel);
    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(50);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
	//mocap_pos_pub.publish(host_mocap);
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::Point desired_pos;
    geometry_msgs::Point desired_vel;

    vir vir1;
    float r = 0.5;
    float T = 2*pi;
    float dt = 0.02;
    float t = 0;
    vir1.x = 0;
    vir1.y = 0;
    vir1.z = 0.5;
    vir1.roll = 0;

    mavros_msgs::AttitudeTarget pose;
    pose.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE|
                     mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE|
                     mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    pose.thrust = 0;

    //send a few setpoints before starting
   for(int i = 200; ros::ok() && i > 0; --i){
        local_vel_pub.publish(pose);
        //mocap_pos_pub.publish(host_mocap);
        vir1.x = initial_pose.pose.position.x;
        vir1.y = initial_pose.pose.position.y;
        vir1.z = initial_pose.pose.position.z+0.5;
        vir1.roll = yaw;

        ros::spinOnce();
        rate.sleep();
    }

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
                vir1.roll -= 0.03;
                break;
            case 68:    // key CCW(<-)
                vir1.roll += 0.03;
                break;
			case 119:    // key foward
                vir1.x += 0.05;
                break;
            case 120:    // key back
                vir1.x += -0.05;
                break;
            case 97:    // key left
                vir1.y += 0.05;
                break;
            case 100:    // key right
                vir1.y -= 0.05;
                break;
	    	case 115:    // key right
		{
                vir1.x = initial_pose.pose.position.x;
                vir1.y = initial_pose.pose.position.y;
                vir1.z = initial_pose.pose.position.z+0.2;
                break;
		}
		case 108:    // close arming
			{
			offb_set_mode.request.custom_mode = "MANUAL";
			set_mode_client.call(offb_set_mode);
			arm_cmd.request.value = false;
			arming_client.call(arm_cmd);
            break;
			}
            case 116:    // t
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
                if(initial == true){
                initial = false;
                }
                else if(initial == false){
                initial = true;
                }

            break;
            }
            case 63:
                return 0;
                break;
            }
        }
		if(vir1.roll>pi)
		vir1.roll = vir1.roll - 2*pi;
		else if(vir1.roll<-pi)
		vir1.roll = vir1.roll + 2*pi;

        if(initial == true){
        vir1.y = initial_pose.pose.position.y + r*sin(2*pi*t/T);
        t += dt;
        }

//        ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z, vir1.roll/pi*180);
        follow(vir1,host_mocap,&pose,0,0,0,0);

        //mocap_pos_pub.publish(host_mocap);
        local_vel_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

