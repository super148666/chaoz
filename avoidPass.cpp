//
// Created by chaoz on 29/10/17.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>

#define SCALE_FACTOR 100


nav_msgs::Odometry::_pose_type::_pose_type g_currentPose;
double g_linear;
double g_angular;
double g_scan[181];
int g_unknown[181];
geometry_msgs::Twist g_msg;
ros::Publisher pub;
cv::Mat visited;
cv::Point currentPoint;

void PrintVisited(){
    cv::Mat display = visited.clone();
    cv::circle(display,currentPoint,5,cv::Scalar(200),-1);
    cv::imshow("visited",display);
}

void GetObjectiveValue(){
    int leftUnknow = 0;
    int rightUnknow = 0;
    for(int i=0;i<90;i++){
        rightUnknow+=g_unknown[i];
    }
    for(int i=91;i<181;i++){
        leftUnknow+=g_unknown[i];
    }
    if(rightUnknow>leftUnknow){
        g_msg.angular.z = -90;
    }
    else{
        g_msg.angular.z = 90;
    }
    g_msg.linear.x = 0.5;
    g_msg.linear.z = abs(rightUnknow-leftUnknow)/500;
}

void laserMessageReceived(const sensor_msgs::LaserScan &msg) {
    int index = 0;
    double rad;
    double dist;
    cv::Point laserPoint;
    cv::Point pointNow;
    cv::Point pointLast;
    for (int i = 90; i < 451; i = i + 2) {
        index = (i - 90) / 2;
        g_unknown[index]=0;
        rad = (index-90+g_currentPose.orientation.w) /180*M_PI;
        g_scan[index] = msg.ranges[i] * 1000.0;
        dist = g_scan[index];
        if(g_scan[index]>1500){
            dist=1500;
        }
        laserPoint.x = (int) ((dist * cos(rad) + g_currentPose.position.x * 1000.0)/ SCALE_FACTOR);
        laserPoint.y = (int) ((dist * sin(rad) + g_currentPose.position.y * 1000.0)/ SCALE_FACTOR);
        cv::line(visited,laserPoint,currentPoint,cv::Scalar(255));
        visited.at<uchar>(laserPoint) = 0;
        if(g_scan[index]<=1500){
            continue;
        }
        pointLast = cv::Point((int) ((g_scan[index] * cos(rad) + g_currentPose.position.x * 1000.0)/ SCALE_FACTOR),(int) ((g_scan[index] * sin(rad) + g_currentPose.position.y * 1000.0)/ SCALE_FACTOR));
        if(visited.at<uchar>(pointLast)){
            g_unknown[index]++;
        }
        for(int j = (int) g_scan[index]-1;j>1500;j--){
            pointNow = cv::Point((int) ((j * cos(rad) + g_currentPose.position.x * 1000.0)/ SCALE_FACTOR),(int) ((j * sin(rad) + g_currentPose.position.y * 1000.0)/ SCALE_FACTOR));
            if(pointNow.x==pointLast.x&&pointNow.y==pointLast.y){
                continue;
            }
            if(visited.at<uchar>(pointNow)==127){
                g_unknown[index]++;
            }
            pointLast.x=pointNow.x;
            pointLast.y=pointNow.y;
        }
    }
    PrintVisited();

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
    currentPoint=cv::Point(static_cast<int>(g_currentPose.position.x / SCALE_FACTOR * 1000.0),
                           static_cast<int>(g_currentPose.position.y / SCALE_FACTOR * 1000.0));
}

int main(int argc, char **argv) {
    cv::namedWindow("visited", CV_GUI_NORMAL | CV_WINDOW_AUTOSIZE);
    ros::init(argc, argv, "avoidPass");
    ros::NodeHandle nh;
    visited = cv::Mat(100,
                  200, CV_8UC1, cv::Scalar(127));

    pub = nh.advertise<geometry_msgs::Twist>("offer4", 1000);
    ros::Subscriber laser;
    ros::Subscriber pose;
    pose = nh.subscribe("RosAria/pose", 1000, &poseMessageReceived);
    laser = nh.subscribe("RosAria/sim_lms1xx_1_laserscan", 1000, &laserMessageReceived);

    g_msg.linear.x = 0.0;
    g_msg.linear.y = 0.0;
    g_msg.linear.z = 0.0;
    g_msg.angular.x = 0.0;
    g_msg.angular.y = 0.0;
    g_msg.angular.z = 0.0;
    pub.publish(g_msg);
    currentPoint.x = 90;
    currentPoint.y = 50;
    g_currentPose.orientation.w = 90;
    while(ros::ok()){
        ros::spinOnce();
        GetObjectiveValue();
        pub.publish(g_msg);
        cv::waitKey(10);
    }return (0);
}