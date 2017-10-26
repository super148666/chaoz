//
// Created by chaoz on 22/10/17.
//

#ifndef ROSARIA_STATE_DISPLAY_H
#define ROSARIA_STATE_DISPLAY_H
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"

#include <iostream>
#include <string>

#define RAD_TO_DEGREE 57.29577951
#define DEGREE_TO_RAD 0.01745329252
#define COLOR_RED 2
#define COLOR_GREEN 1
#define COLOR_BLUE 0
#define TIME_STEP 0.05

using namespace cv;
using namespace std;

class StateDisplay{
private:
    Mat MyObject;
    Mat MyImage;
    Mat MyBackground;
    Size MySize;
    string MyWindowName;
    int MyScaleX;
    int MyScaleY;
    double MyLaserOffset;
    Point MyRobotPosition;
    Point MyLaserPosition;
    double MyLaserMaxRange;
    Vec3b colorBlack;
    Vec3b colorOrange;
    Vec3b colorRed;
    Vec3b colorBlue;
    Vec3b colorGreen;
    int MyFrontLength;
    int MyHalfWidth;
    int minHessian;
    double tt;
public:
    explicit StateDisplay();
    void DisplayImage();
    void DisplayBackground();
    void UpdateSurrounding(double* scan);
    int SearchFreeSpace(double* scan, double distThres, int countThres, double angle);
    void AddWayPoint(double dist, double ang, Scalar color);
    void AddLaserPoint(double dist, double ang, Scalar color);
    void Clear();
    void SaveImage(string name);
};

#endif //ROSARIA_STATE_DISPLAY_H
