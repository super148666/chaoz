//
// Created by chaoz on 22/10/17.
//

#include "state_display.h"
//screen.cpp

geometry_msgs::PoseWithCovarianceStamped::_pose_type::_pose_type g_currentPose;
double g_scan[181];
double g_tansVel = 0.0;
double g_rotVel = 0.0;

StateDisplay::StateDisplay() {
    colorBlack.val[COLOR_BLUE] = 0;
    colorBlack.val[COLOR_GREEN] = 0;
    colorBlack.val[COLOR_RED] = 0;
    colorOrange.val[COLOR_BLUE] = 0;
    colorOrange.val[COLOR_GREEN] = 140;
    colorOrange.val[COLOR_RED] = 255;
    colorRed.val[COLOR_BLUE] = 0;
    colorRed.val[COLOR_GREEN] = 0;
    colorRed.val[COLOR_RED] = 255;
    colorBlue.val[COLOR_BLUE] = 255;
    colorBlue.val[COLOR_GREEN] = 0;
    colorBlue.val[COLOR_RED] = 0;
    colorGreen.val[COLOR_BLUE] = 0;
    colorGreen.val[COLOR_GREEN] = 255;
    colorGreen.val[COLOR_RED] = 0;

    MyLaserMaxRange = 5000;
    int scaleFactor = 15;
    MyScaleX = MyScaleY = scaleFactor;
    MyFrontLength /= scaleFactor;
    MyHalfWidth /= scaleFactor;
    MyRobotPosition = Point((int) (MyLaserMaxRange / MyScaleX),
                            (int) ceil((MyLaserMaxRange + MyLaserOffset) / MyScaleY));
    MyLaserPosition = MyRobotPosition;
    MyLaserPosition.y -= MyLaserOffset / MyScaleY;
    MyBackground = Mat(MyRobotPosition.y+1,
                       (int) (2 * MyLaserMaxRange / MyScaleX)+1, CV_8UC3, Scalar(255, 255, 255));
    rectangle(MyBackground, Point(MyRobotPosition.x - MyHalfWidth, MyRobotPosition.y),
              Point(MyRobotPosition.x + MyHalfWidth, MyRobotPosition.y - MyFrontLength), Scalar(0, 200, 200), -1);
    circle(MyBackground, MyLaserPosition, MyLaserPosition.x, Scalar(0, 255, 0), 1);
    putText(MyBackground, "Max Laser Range", Point(2, 10), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
    putText(MyBackground, "FreeSpace", Point(2, 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
    putText(MyBackground, "Max Laser Range", Point(2, 30), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));

    MyBackground.copyTo(MyImage);
    MySize.width = (int) (2 * MyLaserMaxRange / MyScaleX);
    MySize.height = (int) (MyLaserMaxRange / MyScaleY);
    namedWindow(MyWindowName, CV_GUI_NORMAL | WINDOW_AUTOSIZE);
}

void StateDisplay::DisplayImage() {
    imshow(MyWindowName, MyImage);
    waitKey(10);
    MyImage = MyBackground.clone();
}

void StateDisplay::DisplayBackground() {
    imshow(MyWindowName, MyBackground);
    waitKey(10);
}

void StateDisplay::UpdateSurrounding(double* scan) {
    Point obstacle[181];
    double radToDegree = RAD_TO_DEGREE;
    double rad = 0.0;
    for (int i = 0; i < 181; i++) {
        rad = -i * radToDegree;
        if (scan[i] > MyLaserMaxRange) {
            scan[i] = MyLaserMaxRange;
        }
        obstacle[i].x = ((int) (scan[i] * cos(rad) / MyScaleX) + MyLaserPosition.x);
        obstacle[i].y = ((int) (scan[i] * sin(rad) / MyScaleY) + MyLaserPosition.y);


        if (scan[i] < 500) {
            line(MyImage, MyLaserPosition, obstacle[i], Scalar(0, 0, 255));
        }
        MyImage.at<Vec3b>(obstacle[i]) = colorBlack;

        if (i > 0) {
            if (i < 45 || i > 135) {
                if (norm(obstacle[i - 1] - obstacle[i]) > 100 / MyScaleY) {
                    line(MyImage, obstacle[i - 1], obstacle[i], Scalar(0, 0, 255));
                    continue;
                }
            }
            line(MyImage, obstacle[i - 1], obstacle[i], Scalar(0, 0, 0));
        }
    }
    double angleStep = g_rotVel * TIME_STEP * DEGREE_TO_RAD;
    double lengthStep = g_tansVel * TIME_STEP;
    Point estimatePath;
    double angle, length;
    for (int i = 1; i < 100; i++) {
        angle = angleStep * i;
        length = lengthStep * i;
        estimatePath = Point(static_cast<int>(sin(-angle) * length / MyScaleX + MyRobotPosition.x),
                             static_cast<int>(-cos(angle) * length / MyScaleY + MyRobotPosition.y));
        if (estimatePath.y > MyRobotPosition.y) break;
        MyImage.at<Vec3b>(estimatePath) = colorBlue;
    }
    rectangle(MyImage, Point(MyRobotPosition.x - MyHalfWidth, MyRobotPosition.y),
              Point(MyRobotPosition.x + MyHalfWidth, MyRobotPosition.y - MyFrontLength), Scalar(0, 200, 200), -1);


}

int StateDisplay::SearchFreeSpace(double* scan, double distThres, int countThres) {
    int count = 0;
    int mid = -1;
    int leftMid = -1;
    double radToDegree = RAD_TO_DEGREE;
    double rad;
    int i;
    Point midPoint, freeSpacePoint;
    for (i = 0; i < 181; i++) {
        if (scan[i] > distThres) {
            if (count == 0) mid = -1;
            count++;
            rad = -i * radToDegree;
            freeSpacePoint.x = ((int) (scan[i] * cos(rad) / MyScaleX) + MyLaserPosition.x);
            freeSpacePoint.y = ((int) (scan[i] * sin(rad) / MyScaleY) + MyLaserPosition.y);
            if (scan[i] > distThres) {
                line(MyImage, MyLaserPosition, freeSpacePoint, Scalar(0, 255, 255));
            }
        }
        if (scan[i] <= distThres) {
            if (count > countThres) {
                mid = i - count / 2;
                rad = -mid * radToDegree;
                if (scan[i] > MyLaserMaxRange) {
                    scan[i] = MyLaserMaxRange;
                }
                midPoint = Point(((int) (scan[i] * cos(rad) / MyScaleX) + MyLaserPosition.x),
                                 ((int) (scan[i] * sin(rad) / MyScaleY) + MyLaserPosition.y));
                line(MyImage, MyLaserPosition, midPoint, Scalar(0, 255, 0));
                if (mid > leftMid) {
                    leftMid = mid;
                }
            }
            count = 0;
        }
    }

    if (count > countThres && leftMid == -1) {
        leftMid = i - count / 2;
        if (scan[i] > MyLaserMaxRange) {
            scan[i] = MyLaserMaxRange;
        }
    }
    if (leftMid >= 0) {
        rad = -leftMid * radToDegree;
        midPoint = Point(((int) (scan[leftMid] * cos(rad) / MyScaleX) + MyLaserPosition.x),
                         ((int) (scan[leftMid] * sin(rad) / MyScaleY) + MyLaserPosition.y));
        line(MyImage, MyLaserPosition, midPoint, Scalar(0, 140, 250));
    }
    return leftMid;
}

void poseMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    g_currentPose = msg.pose.pose;
//    cout << "pose received" << endl;
}

void laserMessageReceived(const sensor_msgs::LaserScan &msg) {
    int index = 0;
    for (int i = 90; i < 451; i=i+2) {
        index = (i - 90) / 2;
        g_scan[index] = msg.ranges[i] * 1000.0;
    }
}

void velMessageReceived(const nav_msgs::Odometry &msg){
    g_tansVel = msg.twist.twist.linear.x * 1000.0;
    g_rotVel = msg.twist.twist.angular.z /M_PI*180;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_display");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
    geometry_msgs::Twist msg;
    cout << "publisher to cmd_vel done" << endl;
    ros::Subscriber vel;
    vel = nh.subscribe("RosAria/pose",1000,&velMessageReceived);
    ros::Subscriber pose;
    pose = nh.subscribe("robot_pose_ekf/odom_combined", 1000, &poseMessageReceived);
    cout << "subscriber to odom_combined done" << endl;
    ros::Subscriber laser;
    laser = nh.subscribe("RosAria/sim_lms1xx_1_laserscan", 1000, &laserMessageReceived);
    cout << "subscriber to laserscan done" << endl;
    StateDisplay displayer;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    pub.publish(msg);
    cout << "configure state display done" << endl;
    int mid;
    while (waitKey(20)!=27) {
        ros::spinOnce();
        displayer.UpdateSurrounding(g_scan);
        mid = displayer.SearchFreeSpace(g_scan,2000,13);
//        cout<<mid<<endl;
        displayer.DisplayImage();
        msg.linear.x = 0.4;
        for(int i=45;i<135;i++){
            if(g_scan[i]<700){
                msg.linear.x = 0;
                break;
            }
        }
        if(mid==-1){
            msg.angular.z = -90;
            pub.publish(msg);
            continue;
        }
        mid = (mid-90);
        msg.angular.z = mid;
        pub.publish(msg);


    }

    return (0);
}