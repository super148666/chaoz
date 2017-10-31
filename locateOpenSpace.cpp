//
// Created by chaoz on 23/10/17.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

double g_scan[181];
geometry_msgs::Twist g_msg;
bool g_leftOpen = false;
bool g_rightOpen = false;
bool g_frontOpen = false;
nav_msgs::Odometry::_pose_type::_pose_type g_currentPose;
double g_linear;
double g_angular;
double g_closestDist;
ros::Publisher pub;

int SearchOpenSpace(double distThres, double countThres, int angle){
    int count = 0;
    int followCount = 0;
    int mid = -1;
    int followMid = -1;
    double sumRange = 0.0;
    double halfSumRange = 0.0;
    int i;
    for (i = 0; i < 181; i++) {
        if (g_scan[i] > distThres) {
            if (count == 0) mid = -1;
            count++;
            sumRange += g_scan[i];
        }
        if (g_scan[i] <= distThres) {
            if (count > countThres) {
                halfSumRange = 0;
//                for (int j = i - count; j < i; j++) {
//                    halfSumRange += scan[j];
//                    if (halfSumRange > sumRange / 2) {
//                        mid = j;
//                        break;
//                    }
//                }
                mid = i - count / 2;
                if (followMid == -1) {
                    followMid = mid;
                    followCount = count;
                }
                if (abs(mid - angle) < abs(followMid - angle)) {
                    followMid = mid;
                    followCount = count;
                }
            }
            count = 0;
            sumRange = 0;

        }
    }
    if (count > countThres && followMid == -1) {
        followMid = i - count / 2;
        followCount = count;
    }
//    g_msg.linear.z = followCount/90.0;
    g_msg.linear.z = abs(90-followMid)/45.0;
    if(followMid == -1){
        g_msg.linear.z = 0;
    }
    if(g_msg.linear.z>1) {
        g_msg.linear.z = 1;
    }

    return followMid;
}

void GetObjectiveValue() {
    int followMid = SearchOpenSpace(g_closestDist+1500,15,90);
    if(g_rightOpen) {
        if (followMid > 100) {
            followMid = SearchOpenSpace(g_closestDist+1500, 15, 0);
        }
    }
    if(g_frontOpen){
        followMid = SearchOpenSpace(g_closestDist+1500,15,90);
    }
    if(g_leftOpen) {
        if(followMid < 80 && followMid!=-1){
            followMid = SearchOpenSpace(g_closestDist+1500, 15, 180);
        }
    }
    g_msg.linear.x = 0.5;
    g_msg.angular.z = followMid-90;
    if(g_msg.angular.z>20){
        g_msg.linear.x = 0.4;
        g_msg.angular.z = 20;
    }
    if(g_msg.angular.z<-20){
        g_msg.linear.x = 0.4;
        g_msg.angular.z = -20;
    }
    std::cout<<g_msg.linear.z<<std::endl;
}

void laserMessageReceived(const sensor_msgs::LaserScan &msg) {
    int index = 0;
    g_closestDist = 20000.1;
    for (int i = 90; i < 451; i = i + 2) {
        index = (i - 90) / 2;
        g_scan[index] = msg.ranges[i] * 1000.0;
        if(g_closestDist>g_scan[index]){
            g_closestDist = g_scan[index];
        }
    }
    g_rightOpen = false;
    g_leftOpen = false;
    g_frontOpen = false;
    for (int i = 0; i < 60; i++) {
        if (g_scan[i] > g_closestDist+1500) {
            g_rightOpen = true;
            break;
        }
    }
    for (int i = 60; i < 120; i++) {
        if (g_scan[i] > g_closestDist+1500) {
            g_frontOpen = true;
            break;
        }
    }
    for (int i = 120; i < 181; i++) {
        if (g_scan[i] > g_closestDist+1500) {
            g_leftOpen = true;
            break;
        }
    }

}

void poseMessageReceived(const nav_msgs::Odometry &msg) {
    g_currentPose = msg.pose.pose;
    g_linear = msg.twist.twist.linear.x;
    g_angular = msg.twist.twist.angular.z;
    g_currentPose.orientation.w = atan2(g_currentPose.orientation.z, g_currentPose.orientation.w) * 2 / M_PI * 180;
    if (g_currentPose.orientation.w < -180) {
        g_currentPose.orientation.w += 360;
    }
    if (g_currentPose.orientation.w > 180) {
        g_currentPose.orientation.w -= 360;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "locateOpenSpace");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("offer2", 1000);
    ros::Subscriber laser;
    ros::Subscriber pose;
    laser = nh.subscribe("RosAria/sim_lms1xx_1_laserscan", 1000, &laserMessageReceived);
    pose = nh.subscribe("RosAria/pose", 1000, &poseMessageReceived);
    std::cout << "subscriber to laserscan done" << std::endl;
    g_msg.linear.x = 0.0;
    g_msg.linear.y = 0.0;
    g_msg.linear.z = 0.0;
    g_msg.angular.x = 0.0;
    g_msg.angular.y = 0.0;
    g_msg.angular.z = 0.0;
    pub.publish(g_msg);
    while(ros::ok()){
        ros::spinOnce();
        GetObjectiveValue();
        pub.publish(g_msg);
    }return (0);
}
