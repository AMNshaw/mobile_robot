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
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <flight_control/qptrajectory.h>
#include <flight_control/ncrl_tf.h>
#define gravity 9.806
#define Mocap
//#define VIO
#define LIO
using namespace std;

bool init = false;
bool init_mocap = false;
bool start = false;
bool pose_switch = false;
//set control P-gain
double KPx=1, KPy=1, KPz=1.2;
//float KPx=5, KPy=5, KPz=1;
double KPyaw = 1;
double roll = 0, pitch = 0, yaw = 0;

typedef struct
{
    double yaw;
    double x;
    double y;
    double z;
}vir;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped host_mocap;
geometry_msgs::PoseStamped initial_pose;
nav_msgs::Odometry cam2imu;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

#ifdef VIO
void extrinsic(const nav_msgs::Odometry::ConstPtr& msg)
{
    cam2imu = *msg;
}
#endif

#ifdef Mocap
geometry_msgs::PoseStamped mocap_pos;
geometry_msgs::PoseStamped initial_mocap;
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    mocap_pos.header = msg->header;
    mocap_pos.pose.position = msg->pose.position;
    mocap_pos.pose.orientation = msg->pose.orientation;

    // trans2centroid
    ncrl_tf::Trans trans1_, trans2_, trans3_;
    ncrl_tf::setTransFrame(trans1_, "INERTIA", "MOCAP");
    ncrl_tf::setTransFrame(trans2_, "MOCAP", "CENTROID");
    ncrl_tf::setTransFrame(trans3_, "INERTIA", "CENTROID");
    Eigen::Vector3d EX_v(0, 0, -0.1),
                    v_(mocap_pos.pose.position.x,
                       mocap_pos.pose.position.y,
                       mocap_pos.pose.position.z);
    Eigen::Quaterniond EX_q(1, 0, 0, 0),
                       q_(mocap_pos.pose.orientation.w,
                          mocap_pos.pose.orientation.x,
                          mocap_pos.pose.orientation.y,
                          mocap_pos.pose.orientation.z);
    ncrl_tf::setTrans(trans1_, q_, v_);
    ncrl_tf::setTrans(trans2_, EX_q, EX_v);
    ncrl_tf::accumTrans(trans3_, trans1_, trans2_);
    mocap_pos.pose.position.x = trans3_.v.x();
    mocap_pos.pose.position.y = trans3_.v.y();
    mocap_pos.pose.position.z = trans3_.v.z();
    mocap_pos.pose.orientation.x = trans3_.q.x();
    mocap_pos.pose.orientation.y = trans3_.q.y();
    mocap_pos.pose.orientation.z = trans3_.q.z();
    mocap_pos.pose.orientation.w = trans3_.q.w();

    //store intial pose for first callback
    if(init_mocap == false){
        initial_mocap = mocap_pos;
        init_mocap = true;
    }
    mocap_pos.pose.position.x -= initial_mocap.pose.position.x;
    mocap_pos.pose.position.y -= initial_mocap.pose.position.y;
    mocap_pos.pose.position.z -= initial_mocap.pose.position.z;
}
#endif

void host_pos(const nav_msgs::Odometry::ConstPtr& msg){
    //store odometry into global variable
    host_mocap.header = msg->header;
    host_mocap.pose.position = msg->pose.pose.position;
    host_mocap.pose.orientation = msg->pose.pose.orientation;

    // trans2centroid
    ncrl_tf::Trans trans1_, trans2_, trans3_;
    ncrl_tf::setTransFrame(trans1_, "INERTIA", "MOCAP");
    ncrl_tf::setTransFrame(trans2_, "MOCAP", "CENTROID");
    ncrl_tf::setTransFrame(trans3_, "INERTIA", "CENTROID");
    Eigen::Vector3d EX_v(0, 0, -0.1),
                    v_(host_mocap.pose.position.x,
                       host_mocap.pose.position.y,
                       host_mocap.pose.position.z);
    Eigen::Quaterniond EX_q(1, 0, 0, 0),
                       q_(host_mocap.pose.orientation.w,
                          host_mocap.pose.orientation.x,
                          host_mocap.pose.orientation.y,
                          host_mocap.pose.orientation.z);
    ncrl_tf::setTrans(trans1_, q_, v_);
    ncrl_tf::setTrans(trans2_, EX_q, EX_v);
    ncrl_tf::accumTrans(trans3_, trans1_, trans2_);
    host_mocap.pose.position.x = trans3_.v.x();
    host_mocap.pose.position.y = trans3_.v.y();
    host_mocap.pose.position.z = trans3_.v.z();
    host_mocap.pose.orientation.x = trans3_.q.x();
    host_mocap.pose.orientation.y = trans3_.q.y();
    host_mocap.pose.orientation.z = trans3_.q.z();
    host_mocap.pose.orientation.w = trans3_.q.w();

    //store intial pose for first callback
    if(init == false){
        initial_pose = host_mocap;
        init = true;
    }
    host_mocap.pose.position.x -= initial_pose.pose.position.x;
    host_mocap.pose.position.y -= initial_pose.pose.position.y;
    host_mocap.pose.position.z -= initial_pose.pose.position.z;

    //transfer quartenion to roll, pitch, yaw
    tf::Quaternion Q(
        host_mocap.pose.orientation.x,
        host_mocap.pose.orientation.y,
        host_mocap.pose.orientation.z,
        host_mocap.pose.orientation.w);

    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motive");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
//    ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 2);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
#ifdef Mocap
    ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody7/pose", 10, mocap_cb);
#endif
#ifdef VIO
    ros::Subscriber host_sub = nh.subscribe<nav_msgs::Odometry> ("/vins_estimator/imu_propagate",2, host_pos);
    ros::Subscriber ex_sub = nh.subscribe<nav_msgs::Odometry> ("/vins_estimator/extrinsic",2, extrinsic);
#endif
#ifdef LIO
    ros::Subscriber host_sub = nh.subscribe<nav_msgs::Odometry> ("/estimator/imu_propagate",2, host_pos);
#endif
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

    double max = 0;
    double count = 0;
    qptrajectory plan;
    path_def path;
    trajectory_profile p1,p2,p3,p4,p5;
    std::vector<trajectory_profile> data;
    p1.pos << 0,0,0;
    p1.vel << 0.0,0.0,0;
    p1.acc << 0.00,-0.0,0;
    p1.yaw = 0;

    p2.pos<< 0.4,-0.4,0;
    p2.vel<< 0,0,0;
    p2.acc<< 0,0,0;
    p2.yaw = 0;

    p3.pos<< 0,0.4,0;
    p3.vel<< 0,0,0;
    p3.acc<< 0,0,0;
    p3.yaw = 0;

    p4.pos << -0.4,-0.4,0;
    p4.vel << 0,0,0;
    p4.acc << 0,0,0;
    p4.yaw = 0;

    p5.pos << 0,0,0;
    p5.vel << 0.0,0.0,0;
    p5.acc << 0.00,-0.0,0;
    p5.yaw = 0;

    path.push_back(segments(p1,p2,4));
    path.push_back(segments(p2,p3,4));
    path.push_back(segments(p3,p4,4));
    path.push_back(segments(p4,p5,4));
    data = plan.get_profile(path ,path.size(),dt);
    max = data.size();

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(vs);
        //mocap_pos_pub.publish(host_mocap);
        vir1.x = 0;
        vir1.y = 0;
        vir1.z = 0.5;
        vir1.yaw = yaw;
        ROS_INFO("initial_mocap: %3f, %3f, %3f", initial_mocap.pose.position.x, initial_mocap.pose.position.y, initial_mocap.pose.position.z);
        ROS_INFO("initial_pose: %3f, %3f, %3f", initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z);
//        ROS_WARN("extrinsic: %.2f, %.2f, %.2f", cam2imu.pose.pose.position.x, cam2imu.pose.pose.position.y, cam2imu.pose.pose.position.z);

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
                    if(pose_switch == true){
                    pose_switch = false;
                    }
                    else if(pose_switch == false){
                    pose_switch = true;
                    }
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

        ROS_INFO("%d, %2f, %2f", start, count, max);
        if(start == true){
            //QP trajectory
            geometry_msgs::Point pos;
            geometry_msgs::Point vel;
            pos.x = data[count].pos[0];
            pos.y = data[count].pos[1];
            pos.z = data[count].pos[2];

            vel.x = data[count].vel[0];
            vel.y = data[count].vel[1];
            vel.z = data[count].vel[2];

            vir1.x = pos.x;
            vir1.y = pos.y;
            desired_vel.twist.linear.x = vel.x;
            desired_vel.twist.linear.y = vel.y;

            count++;
            //if trajectory is done, STOP!
            if(count >=max){
                start = false;
                count = 0;
            }
        }
        else{
            desired_vel.twist.linear.x = 0;
            desired_vel.twist.linear.y = 0;
            desired_vel.twist.linear.z = 0;
        }

        ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z, vir1.yaw/M_PI*180);
        //input desired position and measurement, plus feedforward velocity
        //output control input vs
        if(pose_switch)
          follow(vir1, mocap_pos, &vs, desired_vel.twist.linear.x, desired_vel.twist.linear.y, 0, 0);
        else
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



