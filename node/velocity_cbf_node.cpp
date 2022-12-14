#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

using namespace std;

class Track_CBF
{
private:
    geometry_msgs::PoseStamped aprilTag_pos;
    geometry_msgs::PoseStamped self_pos;
    ros::Subscriber aprilTag_pos_sub;
    float distance_track;
    float distance_safe;
    float gamma;

public:
    Track_CBF();
    Track_CBF(ros::NodeHandle nh, string aprilTag_subTopic);
    void aprilTag_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void setCBFparam(float dis_t, float dis_s, float gamma);
    float getTrackDistance();
    float getSafeDistance();
    float getGamma();
    geometry_msgs::PoseStamped getTagPose();

    int QPsolve_vel(geometry_msgs::TwistStamped desired_vel_raw, geometry_msgs::TwistStamped* desired_vel);
};

Track_CBF::Track_CBF(ros::NodeHandle nh, string aprilTag_subTopic)
{
    aprilTag_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(aprilTag_subTopic, 10, &Track_CBF::aprilTag_pose_cb, this);
    distance_safe = distance_track = gamma = 0.5;
}

void Track_CBF::aprilTag_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    aprilTag_pos = *msg;
}

void Track_CBF::setCBFparam(float dis_t, float dis_s, float gm)
{
    distance_track = dis_t;
    distance_safe = dis_s;
    gamma = gm;

}

float Track_CBF::getTrackDistance(){ return distance_track;}
float Track_CBF::getSafeDistance(){ return distance_safe;}
float Track_CBF::getGamma(){ return gamma;}
geometry_msgs::PoseStamped Track_CBF::getTagPose(){ return aprilTag_pos;}

int Track_CBF::QPsolve_vel(geometry_msgs::TwistStamped desired_vel_raw, geometry_msgs::TwistStamped* desired_vel)
{
    Eigen::SparseMatrix<double> hessian_Matrix;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    hessian_Matrix.resize(2,2);
    hessian_Matrix.insert(0,0) = 1;
    hessian_Matrix.insert(1,0) = 0;
    hessian_Matrix.insert(0,1) = 0;
    hessian_Matrix.insert(1,1) = 1;

    gradient.resize(2);
    gradient << - desired_vel_raw.twist.linear.x , - desired_vel_raw.twist.linear.y;

    upperBound.resize(1);
    lowerBound.resize(1);
    linearMatrix.resize(1, 2);

    linearMatrix.insert(0, 0) = 2*(self_pos.pose.position.x - aprilTag_pos.pose.position.x);
    linearMatrix.insert(0, 1) = 2*(self_pos.pose.position.y - aprilTag_pos.pose.position.y);
    upperBound(0) = gamma*(pow(distance_track, 2)
                                -pow(self_pos.pose.position.x - aprilTag_pos.pose.position.x, 2)
                                -pow(self_pos.pose.position.y - aprilTag_pos.pose.position.y, 2));
    lowerBound(0) = -gamma*(pow(self_pos.pose.position.x - aprilTag_pos.pose.position.x, 2)
                                +pow(self_pos.pose.position.y - aprilTag_pos.pose.position.y, 2)
                                -pow(distance_safe, 2));


    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(1);

    if(!solver.data()->setHessianMatrix(hessian_Matrix)) return 1;
    if(!solver.data()->setGradient(gradient)) return 1;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
    if(!solver.data()->setLowerBound(lowerBound)) return 1;
    if(!solver.data()->setUpperBound(upperBound)) return 1;
    if(!solver.initSolver()) return 1;
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

    Eigen::VectorXd QPSolution;
    QPSolution = solver.getSolution();
    *desired_vel = desired_vel_raw;
    desired_vel->twist.linear.x = QPSolution(0);
    desired_vel->twist.linear.y = QPSolution(1);
    if(desired_vel->twist.linear.x > 100 || desired_vel->twist.linear.y > 100)
    {
        desired_vel->twist.linear.x = 0;
        desired_vel->twist.linear.y = 0;
    }


    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_cbf");
    ros::NodeHandle nh;

    ros::Publisher track_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/vel/track", 2);

    ros::Rate rate(100);

    Track_CBF cbf(nh, "/aprilTag_pos");
    cbf.setCBFparam(0.40, 0.30, 0.3); // track_distance, safe_distance, gamma
    geometry_msgs::TwistStamped desired_vel;
    geometry_msgs::TwistStamped desired_vel_raw;
    desired_vel.twist.linear.x = 0;
    desired_vel.twist.linear.y = 0;
    desired_vel_raw.twist.linear.x = 0;
    desired_vel_raw.twist.linear.y = 0;

    
    while(ros::ok())
    {
        cbf.QPsolve_vel(desired_vel_raw , &desired_vel);
        cout << desired_vel << endl;

        track_vel_pub.publish(desired_vel);

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
