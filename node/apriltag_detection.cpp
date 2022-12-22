#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

using namespace std;

class RobotFollower
{
	public:
		RobotFollower(string pubTopic, string subTopic);

		void publish();
		void getsub();
		void cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
	private:
		ros::NodeHandle nh;
		ros::Publisher com_pub;
		ros::Subscriber tag_sub;

		//Tag Position//
		float x;
		float y;
		float z;

		//Check List//
		geometry_msgs::PoseStamped input;
		apriltag_ros::AprilTagDetectionArray tag;
};

RobotFollower::RobotFollower(string pubTopic, string subTopic)
{
	com_pub = nh.advertise<geometry_msgs::PoseStamped>(pubTopic, 100);
	tag_sub = nh.subscribe(subTopic, 100, &RobotFollower::cameraCallback, this);

	x = y = z = 0.0;
}

void RobotFollower::cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	if (msg->detections.size() > 0)
	{
		tag = *msg;
		//id = tag.detections[0].id;
		//header = tag.detections[0].pose.header;
		//cov = tag.detections[0].pose.pose.covariance;
		x = tag.detections[0].pose.pose.pose.position.x;
		y = tag.detections[0].pose.pose.pose.position.y;
		z = tag.detections[0].pose.pose.pose.position.z;
		//i = tag.detections[0].pose.pose.pose.orientation.x;
		//j = tag.detections[0].pose.pose.pose.orientation.y;
		//k = tag.detections[0].pose.pose.pose.orientation.z;
		//w = tag.detections[0].pose.pose.pose.orientation.w;
		
		input.pose.position.x = +z;
		input.pose.position.y = -x;
		input.pose.position.z = -y;
	}
	else
	{
		input.pose.position.x = 0.25;
		input.pose.position.y = 0.0;
		input.pose.position.z = 0.0;
	}
}

void RobotFollower::publish()
{
	com_pub.publish(input);
}

void RobotFollower::getsub()
{

	while (com_pub.getNumSubscribers() == 0)
	{
		ROS_INFO("Waiting for subscribers to connect");
		ros::Duration(0.1).sleep();
	}
	
	ROS_INFO("Connect");
}

int main(int argc, char** argv)
{
	ROS_INFO("Follower Distance");
	ros::init(argc, argv, "DC");
	
	RobotFollower I("/aprilTag_pos", "/tag_detections");
	I.getsub();

	ros::Rate rate(30);

	while(ros::ok())
	{
		ros::spinOnce();
		I.publish();

		rate.sleep();
	}

	ros::shutdown();
	return 0;
}
