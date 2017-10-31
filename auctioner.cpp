//
// Created by chaoz on 30/10/17.
//

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>

#define MAX_BEHAVIOR 6

geometry_msgs::Twist g_behavior[MAX_BEHAVIOR];
bool g_activated[MAX_BEHAVIOR];
geometry_msgs::Twist g_msg;
ros::Publisher pub;

void GetWinner(){
    int winner = -1;
    double highestPrice = 0;
    for(int i=0;i<MAX_BEHAVIOR;i++){
        if(g_activated[i]){
            if(highestPrice < g_behavior[i].linear.z){
                highestPrice = g_behavior[i].linear.z;
                winner = i;
            }
            g_activated[i] = false;
        }
    }
    pub.publish(g_behavior[winner]);
    std::cout<<"winner:"<<winner<<std::endl;
}

void offer1MessageReceived(const geometry_msgs::Twist &msg) {
    g_activated[0] = true;
    g_behavior[0] = msg;
}

void offer2MessageReceived(const geometry_msgs::Twist &msg) {
    g_activated[1] = true;
    g_behavior[1] = msg;
}

void offer3MessageReceived(const geometry_msgs::Twist &msg) {
    g_activated[2] = true;
    g_behavior[2] = msg;
}

void offer4MessageReceived(const geometry_msgs::Twist &msg) {
    g_activated[3] = true;
    g_behavior[3] = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "auctioner");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
    std::cout << "publisher to cmd_vel done" << std::endl;
    ros::Subscriber offer1;
    offer1 = nh.subscribe("offer1", 1000, &offer1MessageReceived);
    ros::Subscriber offer2;
    offer2 = nh.subscribe("offer2", 1000, &offer2MessageReceived);
    ros::Subscriber offer3;
    offer3 = nh.subscribe("offer3", 1000, &offer3MessageReceived);
    ros::Subscriber offer4;
    offer4 = nh.subscribe("offer3", 1000, &offer4MessageReceived);
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
        GetWinner();
        ros::Duration(0.1).sleep();
    }return (0);
}
