//
// Created by chaoz on 29/10/17.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <cstdlib>


geometry_msgs::Twist g_msg;
ros::Publisher pub;

void GetObjectiveValue(){

    g_msg.linear.x = 0.5;
    g_msg.angular.z = 0;
    g_msg.linear.z = 0.01;

}

int main(int argc, char **argv) {
    srand(time(NULL));

    ros::init(argc, argv, "driveStraight");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("offer3", 1);
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

