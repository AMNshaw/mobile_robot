#include "ros/ros.h"
#include "std_msgs/String.h"
#include "getch.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <math.h>

#define LEADER_INIT_X -1.0f
#define LEADER_INIT_Y 0.0f
#define TAKEOFF_SPEED 0.5f
#define TAKEOFF_HEIGHT 1.2f
#define LAND_SPEED 0.8f
#define CONTROL_HZ 100.0f

float trajectory_t; 
/*
float wayPoint_x_final = 0.0;
float wayPoint_y_final = 0.0;
float wayPoint_x_initial;
float wayPoint_y_initial;
*/

geometry_msgs::PoseStamped leader_pose;
geometry_msgs::Point wayPoint_final;
geometry_msgs::Point wayPoint_initial;



enum {
	DISARM,
	HOVERING,
    TAKEOFF,
    LAND,
    TRAJECTORY_FOLLOWING,
	WAYPOINT_FOLLOWING,
	TRACK_RED_POINT,
}LeaderMode;

int leader_mode;
int kill_all_drone = 0;
int start_all_drone = 0;

void start_takeoff(){
	if(leader_mode == TAKEOFF || leader_pose.pose.position.z>0.1 ){
		ROS_WARN("leader already takeoff");
	}
	else{
		leader_mode = TAKEOFF;
		ROS_INFO("leader start takeoff");
	}
}

void start_land(){
	if(leader_mode == LAND || leader_pose.pose.position.z <= 0.01 ){
		ROS_WARN("leader already landing or it's on the land");
	}
	else{
		leader_mode = LAND;
		ROS_INFO("leader start landing");
	}
}

void start_trajectory_following(){
	if(leader_mode == HOVERING){
    	leader_mode = TRAJECTORY_FOLLOWING;
		ROS_INFO("leader start trajectory");
	}
	else{
		ROS_WARN("leader can not start trajectory");
	}
}
void stop_trajectory(){
	if(leader_mode == TRAJECTORY_FOLLOWING){
    	leader_mode = HOVERING;
		ROS_INFO("leader stop trajectory");
	}
	else{
		ROS_WARN("leader not in trajectory");
	}
}

void start_waypoint_following(){
	if(leader_mode == HOVERING){
    	leader_mode = WAYPOINT_FOLLOWING;
		ROS_INFO("leader start waypoint");
	}
	else{
		ROS_WARN("leader can not start waypoint");
	}
}

void stop_waypoint(){
	if(leader_mode == WAYPOINT_FOLLOWING){
    	leader_mode = HOVERING;
		ROS_INFO("leader stop waypoint");
	}
	else{
		ROS_WARN("leader not in waypoint");
	}
}

void start_track_red_point(){
	if(leader_mode == HOVERING){
    	leader_mode = TRACK_RED_POINT;
		ROS_INFO("leader start tracking point");
	}
	else{
		ROS_WARN("leader can not start tracking");
	}
}

void stop_track_red_point(){
	if(leader_mode == TRACK_RED_POINT){
    	leader_mode = HOVERING;
		ROS_INFO("leader stop tracking");
	}
	else{
		ROS_WARN("leader not tracking");
	}
}

void leader_pose_generate(geometry_msgs::PoseStamped *leader_pose){
	if(leader_mode == TAKEOFF){
  		leader_pose->pose.position.z += TAKEOFF_SPEED/CONTROL_HZ;
		if( leader_pose->pose.position.z >= TAKEOFF_HEIGHT ){
			leader_pose->pose.position.z = TAKEOFF_HEIGHT;
			leader_mode = HOVERING;
		}
	}

	if(leader_mode == LAND){
  		leader_pose->pose.position.z -= LAND_SPEED/CONTROL_HZ;
		if( leader_pose->pose.position.z <= 0 ){
			leader_pose->pose.position.z = 0;
			leader_mode = DISARM;
		}
	}

	if(leader_mode == TRAJECTORY_FOLLOWING){
		trajectory_t += 1/CONTROL_HZ;
		leader_pose->pose.position.x = cos(trajectory_t*0.3 + M_PI);
		leader_pose->pose.position.y = sin(trajectory_t*0.3 + M_PI);
	}

	if(leader_mode == WAYPOINT_FOLLOWING || leader_mode == TRACK_RED_POINT){
		trajectory_t += 1/CONTROL_HZ;
		if(abs(leader_pose->pose.position.x -wayPoint_final.x) > 0.01 || abs(leader_pose->pose.position.y - wayPoint_final.y) > 0.01 || abs(leader_pose->pose.position.z - wayPoint_final.z) > 0.01 )
		{
			geometry_msgs::Point err;
			err.x  = wayPoint_final.x - wayPoint_initial.x;
			err.y  = wayPoint_final.y - wayPoint_initial.y;
			err.z  = wayPoint_final.z - wayPoint_initial.z;
		
			float err_norm = sqrt(pow(err.x, 2) + pow(err.y, 2) + pow(err.z, 2));	
			leader_pose->pose.position.x = wayPoint_initial.x + err.x/err_norm*trajectory_t*0.3;
			leader_pose->pose.position.y = wayPoint_initial.y + err.y/err_norm*trajectory_t*0.3;
			leader_pose->pose.position.z = wayPoint_initial.z + err.z/err_norm*trajectory_t*0.3;

		}
		else
		{

		}
	}
}

void waypoint_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	if(leader_mode == WAYPOINT_FOLLOWING)
	{
		trajectory_t = 0;
		if(msg->x > 1.5 || msg->y > 1.5 || msg->z > 2)
			ROS_WARN("Command out of bound");
		else
		{
			wayPoint_initial = leader_pose.pose.position;
			wayPoint_final = *msg;		
		}
		
	}

	else if(leader_mode == TRACK_RED_POINT)
	{
		trajectory_t = 0;
		if(msg->x > 1.5 || msg->y > 1.5)
			ROS_WARN("Command out of bound");
		else
		{
			wayPoint_initial = leader_pose.pose.position;
			wayPoint_final.x = msg->x;
			wayPoint_final.y = msg->y;
		}
	}
	else
		ROS_WARN("leader not in waypoint mode");
}


int main(int argc, char **argv)
{
  leader_mode = DISARM;
  ros::init(argc, argv, "leader_pose_publisher");

  ros::NodeHandle nh;
  ros::Publisher leader_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/leader_pose", 10);
  ros::Publisher uav_killer_pub = nh.advertise<std_msgs::Int32>("/uav_kill", 10);
  ros::Publisher uav_start_pub = nh.advertise<std_msgs::Int32>("/uav_start", 10);
  ros::Subscriber point_sub = nh.subscribe<geometry_msgs::Point>("/waypoint", 10, waypoint_cb);

  ros::Rate loop_rate(100);
  
  leader_pose.header.stamp = ros::Time::now();
  leader_pose.header.frame_id = "map";
  leader_pose.pose.position.x = LEADER_INIT_X;
  leader_pose.pose.position.y = LEADER_INIT_Y;
  leader_pose.pose.position.z = 0.0;
  leader_pose.pose.orientation.x = 0.0;
  leader_pose.pose.orientation.y = 0.0;
  leader_pose.pose.orientation.z = 0.0;
  leader_pose.pose.orientation.w = 1.0;

	ROS_INFO("(t):takeoff\n (l):land\n (e):start_trajectory\n (w):waypoint_mode\n (r):track_red_point\n (p):stop MAV\n (k):kill_all_drone\n (s):start_all_drone \n");
  while (ros::ok())
  {
        //keyboard control
        int c = getch();
        //ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
                case 116:    // (t) takeoff
                    start_takeoff();
                    break;
                case 108:    // (l) land
                    start_land();
                    break;
                case 101:    // (e) start_trajectory_following
                    start_trajectory_following();
                    break;
                case 119:    // (w) start_wayPoint_following
                	wayPoint_initial = leader_pose.pose.position;
					wayPoint_final = leader_pose.pose.position;
                    start_waypoint_following();
                    break;
				case 114:    // (t) start_track_red_point
                	wayPoint_initial = leader_pose.pose.position;
					wayPoint_final = leader_pose.pose.position;
                   	start_track_red_point();
                    break;	
                case 112:    // (p) stop_trajectory_following
                	if(leader_mode == TRAJECTORY_FOLLOWING)
                    	stop_trajectory();
                    else if (leader_mode == WAYPOINT_FOLLOWING)
                    	stop_waypoint();
					else if (leader_mode == TRACK_RED_POINT)
						stop_track_red_point();
                    break;
                case 107:    // (k) uav_kill
					kill_all_drone = 1;
					ROS_WARN("kill all drone");
                    break;
                case 115:    // (s) uav_start
					start_all_drone = 1;
					ROS_INFO("start all drone");
                    break;
			}
        }
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
	leader_pose_generate(&leader_pose);
	
	std_msgs::Int32 kill_msg;
	kill_msg.data=kill_all_drone;
	uav_killer_pub.publish(kill_msg);
	std_msgs::Int32 start_msg;
	start_msg.data=start_all_drone;
	uav_start_pub.publish(start_msg);
    leader_pose_pub.publish(leader_pose);
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
