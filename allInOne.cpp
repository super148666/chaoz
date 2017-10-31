//
// Created by chaoz on 31/10/17.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>

#define SCALE_FACTOR 100
#define MAX_BEHAVIOR 6

nav_msgs::Odometry::_pose_type::_pose_type g_currentPose;
double g_linear;
double g_angular;
double g_scan[181];
double g_closestDist;
int g_unknown[181];
geometry_msgs::Twist g_behavior[MAX_BEHAVIOR];
bool g_activated[MAX_BEHAVIOR];
bool g_leftOpen = false;
bool g_rightOpen = false;
bool g_frontOpen = false;
geometry_msgs::Twist g_msg;
ros::Publisher pub;
cv::Mat visited;
cv::Point currentPoint;

void PrintVisited(){
    cv::Mat display = visited.clone();
    cv::circle(display,currentPoint,5,cv::Scalar(200),-1);
    cv::imshow("visited",display);
}

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

void GetObjectiveValue3(){
    g_activated[3]=true;
    int leftUnknow = 0;
    int rightUnknow = 0;
    for(int i=0;i<90;i++){
        rightUnknow+=g_unknown[i];
    }
    for(int i=91;i<181;i++){
        leftUnknow+=g_unknown[i];
    }
    if(rightUnknow>leftUnknow){
        g_behavior[3].angular.z = -90;
    }
    else{
        g_behavior[3].angular.z = 90;
    }
    g_behavior[3].linear.x = 0.5;
    g_behavior[3].linear.z = abs(rightUnknow-leftUnknow)/1000;
    if(g_behavior[3].linear.z>1){
        g_behavior[3].linear.z=1;
    }
    std::cout<<"offer3:"<<g_behavior[3].linear.z<<std::endl;
}

void GetObjectiveValue2(){
    g_activated[2]=true;
    g_behavior[2].linear.x = 0.5;
    g_behavior[2].angular.z = 0;
    g_behavior[2].linear.z = 0.01;
    std::cout<<"offer2:"<<g_behavior[2].linear.z<<std::endl;
}

void GetObjectiveValue1() {
    g_activated[1]=true;
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
    g_behavior[1].linear.x = 0.5;
    g_behavior[1].angular.z = followMid-90;
    if(g_behavior[1].angular.z>20){
        g_behavior[1].linear.x = 0.4;
        g_behavior[1].angular.z = 20;
    }
    if(g_behavior[1].angular.z<-20){
        g_behavior[1].linear.x = 0.4;
        g_behavior[1].angular.z = -20;
    }
    std::cout<<"offer1:"<<g_behavior[1].linear.z<<std::endl;
}

void GetObjectiveValue0(){
    g_activated[0]=true;
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
    g_behavior[0].linear.z = 0;
    if(rightClosestDist<350){
        g_behavior[0].angular.z = 90;
        g_behavior[0].linear.x = g_linear / 2;
        g_behavior[0].linear.z = 1.1;
    }
    if(leftClosestDist<350){
        g_behavior[0].angular.z = -90;
        g_behavior[0].linear.x = g_linear / 2;
        g_behavior[0].linear.z = 1.1;
    }
    if(rightClosestDist<350&&leftClosestDist<350){
        g_behavior[0].linear.x = 0;
        g_behavior[0].angular.z = g_scan[0]<g_scan[180]? 90:-90;
        g_behavior[0].linear.z = 1.1;
    }
    if(frontClosestDist<600){
        g_behavior[0].linear.x = 0;
        g_behavior[0].angular.z = g_scan[0]<g_scan[180]? 90:-90;
        g_behavior[0].linear.z = 1.1;
    }
    std::cout<<"offer0:"<<g_behavior[0].linear.z<<std::endl;
}

void laserMessageReceived(const sensor_msgs::LaserScan &msg) {
    int index = 0;
    double rad;
    double dist;
    g_closestDist = 20000.1;
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

    PrintVisited();
    cv::waitKey(10);

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


int main(int argc, char **argv) {
    cv::namedWindow("visited", CV_GUI_NORMAL | CV_WINDOW_AUTOSIZE);
    ros::init(argc, argv, "allInOne");
    ros::NodeHandle nh;
    visited = cv::Mat(100,
                      200, CV_8UC1, cv::Scalar(127));

    pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
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
        std::cout<<"----------"<<std::endl;
        GetObjectiveValue3();
        GetObjectiveValue2();
        GetObjectiveValue1();
        GetObjectiveValue0();
        GetWinner();
    }return (0);
}
