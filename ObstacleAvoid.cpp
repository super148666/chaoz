//
// Created by chaoz on 23/10/17.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

double g_scan[181];
nav_msgs::Odometry::_pose_type::_pose_type g_currentPose;
geometry_msgs::Twist g_msg;
double g_linear;
double g_angular;
ros::Publisher pub;

void GetObjectiveValue(){
    double leftClosestDist = 20000.1;
    double rightClosestDist = 20000.1;
    double frontClosestDist = 20000.1;
    double dist;
    for(int i=0;i<60;i++){
        dist = g_scan[i]*cos(i * M_PI/180);
        if(dist<rightClosestDist){
            rightClosestDist = dist;
        }
    }
    for(int i=60;i<121;i++){
        dist = g_scan[i] * cos(abs(90-i)*M_PI/180);
        if(dist<frontClosestDist){
            frontClosestDist = dist;
        }
    }
    for(int i=121;i<181;i++){
        dist = g_scan[i]*cos((180-i) * M_PI/180);
        if(dist<leftClosestDist){
            leftClosestDist = dist;
        }
    }
    g_msg.linear.z = 0;
    if(rightClosestDist<350){
        g_msg.angular.z = 90;
        g_msg.linear.x = g_linear / 2;
        g_msg.linear.z = 1.1;
    }
    if(leftClosestDist<350){
        g_msg.angular.z = -90;
        g_msg.linear.x = g_linear / 2;
        g_msg.linear.z = 1.1;
    }
    if(rightClosestDist<350&&leftClosestDist<350){
        g_msg.linear.x = 0;
        g_msg.angular.z = g_scan[0]<g_scan[180]? 90:-90;
        g_msg.linear.z = 1.1;
    }
    if(frontClosestDist<600){
        g_msg.linear.x = 0;
        g_msg.angular.z = g_scan[0]<g_scan[180]? 90:-90;
        g_msg.linear.z = 1.1;
    }
}

void laserMessageReceived(const sensor_msgs::LaserScan &msg) {
    int index = 0;
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

    pub = nh.advertise<geometry_msgs::Twist>("offer1", 1000);
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
