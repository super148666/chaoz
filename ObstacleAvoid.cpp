//
// Created by chaoz on 23/10/17.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

double g_scan[181];
nav_msgs::Odometry::_pose_type::_pose_type g_currentPose;
geometry_msgs::Twist g_msg;
double g_linear;
double g_angular;
ros::Publisher pub;

double GetObjectiveValue(){
    bool leftObstacle = false;
    bool rightObstacle = false;
    bool frontObstacle = false;
    double leftClosestDist = 20000.1;
    double rightClosestDist = 20000.1;
    double frontClosestDist = 20000.1;
    double leftObjective;
    double rightObjective;
    double frontObjective;
    for(int i=0;i<60;i++){
        if(g_scan[i]<rightClosestDist){
            rightClosestDist = g_scan[i];
            rightObstacle = true;
        }
    }
    for(int i=60;i<121;i++){
        if(g_scan[i]<frontClosestDist){
            frontClosestDist = g_scan[i];
            frontObstacle = true;
        }
    }
    for(int i=121;i<181;i++){
        if(g_scan[i]<leftClosestDist){
            leftClosestDist = g_scan[i];
            leftObstacle = true;
        }
    }


}

void laserMessageReceived(const sensor_msgs::LaserScan &msg) {
    int index = 0;
    int count;
    for (int i = 90; i < 451; i = i + 2) {
        index = (i - 90) / 2;
        g_scan[index] = msg.ranges[i] * 1000.0;
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
    ros::init(argc, argv, "obstacleAvoid");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("obstacleAvoid/offer", 1);
    geometry_msgs::Twist msg;
    std::cout << "publisher to cmd_vel done" << std::endl;
    ros::Subscriber laser;
    ros::Subscriber pose;
    laser = nh.subscribe("RosAria/sim_lms1xx_1_laserscan", 1, &laserMessageReceived);
    pose = nh.subscribe("RosAria/pose", 1000, &poseMessageReceived);
    std::cout << "subscriber to laserscan done" << std::endl;
    pub.publish(g_msg);
    ros::spin();
    return (0);
}
