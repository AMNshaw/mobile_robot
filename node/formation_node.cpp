#include <ros/ros.h>
#include "ros/param.h"
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
#include <queue>

#define gravity 9.806
using namespace std;

bool init = false;
bool start = false;
//set control P-gain
double KPx=1, KPy=1, KPz=1.2;
//float KPx=5, KPy=5, KPz=1.2;
double KPyaw = 1;
double roll = 0, pitch = 0, yaw = 0;


class MAV
{
private:
    geometry_msgs::PoseStamped MAV_pose;
    ros::Subscriber pose_sub;
    queue<geometry_msgs::PoseStamped> pose_queue;
    int id;

public:
    MAV(ros::NodeHandle nh, string subTopic, int ID);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    geometry_msgs::PoseStamped getPose();
    static int UAV_ID;
    static int delay_step;    
};

int MAV::UAV_ID = 0;
int MAV::delay_step = 0;

MAV::MAV(ros::NodeHandle nh, string subTopic, int ID)
{
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(subTopic, 10, &MAV::pose_cb, this);
    id = ID;
}

void MAV::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(id != UAV_ID)
    {
	pose_queue.push(*msg);
	if(pose_queue.size() >= delay_step)
	{
	    MAV_pose = pose_queue.front();
	    pose_queue = queue<geometry_msgs::PoseStamped>();
	}
    }
    else
	MAV_pose = *msg;
}

geometry_msgs::PoseStamped MAV::getPose(){return MAV_pose;}



void laplacian_remap(XmlRpc::XmlRpcValue laplacian_param, bool laplacian_map[][5])
{
    int k = 0;
    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
	    ROS_ASSERT(laplacian_param[k].getType() == XmlRpc::XmlRpcValue::TypeInt);
            int a = laplacian_param[k];
	    laplacian_map[i][j] = a!=0;
	    k++;
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation");
    ros::NodeHandle nh;

    ros::param::get("UAV_ID", MAV::UAV_ID);
    ros::param::get("delay_step", MAV::delay_step);

    //Subscriber
    MAV mav[5] = {MAV(nh, "/leader_pose", 0),
                  MAV(nh, "/vrpn_client_node/MAV1/pose", 1),
                  MAV(nh, "/vrpn_client_node/MAV2/pose", 2),
                  MAV(nh, "/vrpn_client_node/MAV3/pose", 3),
                  MAV(nh, "/vrpn_client_node/MAV4/pose", 4)};

    //Publisher    
    ros::Publisher desired_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("desired_velocity_raw", 100);
    XmlRpc::XmlRpcValue laplacian_param;
    nh.getParam("laplacian", laplacian_param);
    ROS_ASSERT(laplacian_param.getType() == XmlRpc::XmlRpcValue::TypeArray);
    bool laplacian_map[5][5] = {0};
    laplacian_remap(laplacian_param, laplacian_map);


    //float leader_uav_vector_x[5] = {0,0.5,-0.5,-0.5,0.5 };  //vector x from leader to uav
    //float leader_uav_vector_y[5] = {0,0.5,0.5 ,-0.5,-0.5};  //vector y from leader to uav
    float leader_uav_vector_x[5] = {0, 0, -0.5, 0, 0.5 };  //vector x from leader to uav
    float leader_uav_vector_y[5] = {0, 0, -0.5 , -0.5*sqrt(2),-0.5};  //vector y from leader to uav
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
            if(laplacian_map[MAV::UAV_ID][i] == 1){
                desired_vel.twist.linear.x += mav[i].getPose().pose.position.x - mav[MAV::UAV_ID].getPose().pose.position.x + relative_map_x[MAV::UAV_ID][i] ;
                desired_vel.twist.linear.y += mav[i].getPose().pose.position.y - mav[MAV::UAV_ID].getPose().pose.position.y + relative_map_y[MAV::UAV_ID][i] ;
                desired_vel.twist.linear.z += mav[i].getPose().pose.position.z - mav[MAV::UAV_ID].getPose().pose.position.z;
            }
        }
        desired_vel_pub.publish(desired_vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



