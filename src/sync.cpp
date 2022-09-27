#include <ros/ros.h>
#include <mavros_msgs/CamIMUStamp.h>
#include <sensor_msgs/Image.h>
#define period 50000000

bool first = false;
int trigger_time = 0;
int image_time = 0;
int trigger_count = 0;
int trigger_curs = 0;
int trigger_curns = 0;
ros::Publisher pub;
sensor_msgs::Image correct;
mavros_msgs::CamIMUStamp trigger;
void cb(const mavros_msgs::CamIMUStamp::ConstPtr& msg)
{
    trigger = *msg;
    //  ROS_INFO("print: %d,%d",trigger.frame_seq_id,correct.header.seq);
    //  ROS_INFO("trigger_count: %d",trigger_count);

    //ROS trigger time in ns
    trigger_time = trigger.frame_stamp.sec*1000000000 + trigger.frame_stamp.nsec;
    //update when trigger number - image number = 1
    if((trigger.frame_seq_id - correct.header.seq) == (trigger_count + 1)){
        trigger_curs = trigger.frame_stamp.sec;
        trigger_curns = trigger.frame_stamp.nsec;
    }
}
void icb(const sensor_msgs::Image::ConstPtr& msg)
{
    correct = *msg;
    //ROS image time in ns
    image_time = correct.header.stamp.sec*1000000000 + correct.header.stamp.nsec;
    //update offset(trigger_count) between trigger number and image number,
    //before refine image timestamp
    if(first == false){
        //trigger time should happened before image time
        if((image_time - trigger_time) > 0){
            trigger_count = trigger.frame_seq_id - correct.header.seq;
            first = true;
        }
    ROS_INFO("Throw first image!");
    }
    else{
        //in this callback trigger number and image number should be the same
        if((trigger.frame_seq_id - correct.header.seq) == trigger_count){
            int time_offset = (correct.header.stamp.sec - trigger_curs)*1000000000 + (correct.header.stamp.nsec - trigger_curns);
            ROS_INFO("td: %d",time_offset/1000000);
            //refine image timestamp and publish
            correct.header.stamp.sec = trigger_curs;
            correct.header.stamp.nsec = trigger_curns;
            pub.publish(correct);
        }
        else{
            ROS_INFO("Drop Image");
        }
    }
}

//mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 200
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<mavros_msgs::CamIMUStamp>("/mavros/cam_imu_sync/cam_imu_stamp", 1, cb);
    //ros::Subscriber isub = nh.subscribe<sensor_msgs::Image>("/camera/image_raw", 2, icb);
    ros::Subscriber isub = nh.subscribe<sensor_msgs::Image>("/arducam/triggered/camera/image_raw", 1, icb);
    pub = nh.advertise<sensor_msgs::Image>("/sync/image", 2);

    ros::spin();

    return 0;
}

