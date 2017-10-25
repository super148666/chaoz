//
// Created by chaoz on 22/10/17.
//

#include "tourGuide.h"

#define NUM_WAYPOINT 25
#define DIST_THRES 1000

geometry_msgs::PoseWithCovarianceStamped::_pose_type::_pose_type g_currentPose;
//nav_msgs::Odometry::_pose_type::_pose_type g_currentPose;
StateDisplay displayer;
geometry_msgs::Pose::_position_type g_waypoint[NUM_WAYPOINT];
double g_scan[181];
double g_tansVel = 0.0;
double g_rotVel = 0.0;
ros::Publisher pub;
//perfect with no wheel error - from mobile sim
//void InitGlobalVariables() {
//    for (int i = 0; i < NUM_WAYPOINT; i++) {
//        g_waypoint[i].z = 0.0;
//    }
//    g_waypoint[0].x = 0.0;
//    g_waypoint[0].y = 0.0;
//
//    g_waypoint[1].x = 1188.11;
//    g_waypoint[1].y = -4654.41;
//
//    g_waypoint[2].x = 6076.21;
//    g_waypoint[2].y = -4658.01;
//
//    g_waypoint[3].x = 11599.7;
//    g_waypoint[3].y = -4374.04;
//
//    g_waypoint[4].x = 16916.6;
//    g_waypoint[4].y = -4345.23;
//
//    g_waypoint[5].x = 19250.6;
//    g_waypoint[5].y = -4330.14;
//
//    g_waypoint[6].x = 22167.5;
//    g_waypoint[6].y = -4315.79;
//
//    g_waypoint[7].x = 25720.5;
//    g_waypoint[7].y = -4297.87;
//
//    g_waypoint[8].x = 25762.7;
//    g_waypoint[8].y = -5942.66;
//
//    g_waypoint[9].x = 25768.7;
//    g_waypoint[9].y = -8260.66;
//
//    g_waypoint[10].x = 25775.8;
//    g_waypoint[10].y = -10942.7;
//
//    g_waypoint[11].x = 25779.3;
//    g_waypoint[11].y = -12290.7;
//
//    g_waypoint[12].x = 29497.7;
//    g_waypoint[12].y = -12342.8;
//
//    g_waypoint[13].x = 31455.8;
//    g_waypoint[13].y = -12539.8;
//
//    g_waypoint[14].x = 36012.5;
//    g_waypoint[14].y = -12480.3;
//
//    g_waypoint[15].x = 38177.5;
//    g_waypoint[15].y = -12463.2;
//
//    g_waypoint[16].x = 41265.4;
//    g_waypoint[16].y = -12437.8;
//
//    g_waypoint[17].x = 25476.2;
//    g_waypoint[17].y = 10562.0;
//
//    g_waypoint[18].x = 25308.5;
//    g_waypoint[18].y = 17285.2;
//
//    g_waypoint[19].x = 25540.1;
//    g_waypoint[19].y = 25375.6;
//
//    g_waypoint[20].x = 25167.8;
//    g_waypoint[20].y = 32743.6;
//
//    g_waypoint[21].x = 25118.6;
//    g_waypoint[21].y = 37065.4;
//
//    g_waypoint[22].x = 25076.0;
//    g_waypoint[22].y = 38784.2;
//
//}

//with imu & ekf
//void InitGlobalVariables() {
//    for(int i=0;i<NUM_WAYPOINT;i++){
//        g_waypoint[i].z = 0;
//    }
//    g_waypoint[0].x = 0;
//    g_waypoint[0].y = 0;
//    g_waypoint[1].x = 2.34865*1000;
//    g_waypoint[1].y = -0.134793*1000;
//    g_waypoint[2].x = 5.90467*1000;
//    g_waypoint[2].y = -0.33409*1000;
//    g_waypoint[3].x = 11.098*1000;
//    g_waypoint[3].y = -0.507425*1000;
//    g_waypoint[4].x = 16.8485*1000;
//    g_waypoint[4].y = -1.03163*1000;
//    g_waypoint[5].x = 19.0892*1000;
//    g_waypoint[5].y = -1.21065*1000;
//    g_waypoint[6].x = 22.4892*1000;
//    g_waypoint[6].y = -1.47782*1000;
//    g_waypoint[7].x = 26.1104*1000;
//    g_waypoint[7].y = -1.94202*1000;
//    g_waypoint[8].x = 25.7968*1000;
//    g_waypoint[8].y = -4.14788*1000;
//    g_waypoint[9].x = 25.4624*1000;
//    g_waypoint[9].y = -6.47065*1000;
//    g_waypoint[10].x = 24.9693*1000;
//    g_waypoint[10].y = -9.18086*1000;
//    g_waypoint[11].x = 24.766*1000;
//    g_waypoint[11].y = -10.4076*1000;
//    g_waypoint[12].x = 28.8565*1000;
//    g_waypoint[12].y = -11.2251*1000;
//    g_waypoint[13].x = 31.1937*1000;
//    g_waypoint[13].y = -11.68*1000;
//    g_waypoint[14].x = 35.6165*1000;
//    g_waypoint[14].y = -12.4605*1000;
//    g_waypoint[15].x = 37.6738*1000;
//    g_waypoint[15].y = -12.7547*1000;
//    g_waypoint[16].x = 25.295*1000;
//    g_waypoint[16].y = -9.45628*1000;
//    g_waypoint[17].x = 27.6272*1000;
//    g_waypoint[17].y = -1.00099*1000;
//    g_waypoint[18].x = 31.8136*1000;
//    g_waypoint[18].y = 8.75242*1000;
//    g_waypoint[19].x = 33.5139*1000;
//    g_waypoint[19].y = 13.1856*1000;
//    g_waypoint[20].x = 36.2642*1000;
//    g_waypoint[20].y = 20.2039*1000;
//    g_waypoint[21].x = 39.8409*1000;
//    g_waypoint[21].y = 28.2846*1000;
//    g_waypoint[22].x = 43.8101*1000;
//    g_waypoint[22].y = 35.1068*1000;
//    g_waypoint[23].x = 46.4566*1000;
//    g_waypoint[23].y = 39.3946*1000;
//    g_waypoint[24].x = 47.2396*1000;
//    g_waypoint[24].y = 40.3156*1000;
//    g_waypoint[25].x = 48.5556*1000;
//    g_waypoint[25].y = 42.0615*1000;
//    g_waypoint[26].x = 39.4277*1000;
//    g_waypoint[26].y = 28.5384*1000;
//}

//with imu & ekf with same linear speed
void InitGlobalVariables() {
    g_waypoint[0].x = 0;
    g_waypoint[0].y = 0;
    g_waypoint[1].x = 2.80797;
    g_waypoint[1].y = -0.0161297;
    g_waypoint[2].x = 5.723;
    g_waypoint[2].y = 0.0469143;
    g_waypoint[3].x = 11.1869;
    g_waypoint[3].y = 0.342565;
    g_waypoint[4].x = 16.9077;
    g_waypoint[4].y = 0.132684;
    g_waypoint[5].x = 19.358;
    g_waypoint[5].y = 0.287418;
    g_waypoint[6].x = 22.178;
    g_waypoint[6].y = 0.335201;
    g_waypoint[7].x = 25.3527;
    g_waypoint[7].y = 0.495888;
    g_waypoint[8].x = 25.4903;
    g_waypoint[8].y = -1.95358;
    g_waypoint[9].x = 25.6442;
    g_waypoint[9].y = -4.41105;
    g_waypoint[10].x = 25.9217;
    g_waypoint[10].y = -7.71921;
    g_waypoint[11].x = 30.3027;
    g_waypoint[11].y = -7.96171;
    g_waypoint[12].x = 32.6672;
    g_waypoint[12].y = -7.80257;
    g_waypoint[13].x = 37.0167;
    g_waypoint[13].y = -7.61941;
    g_waypoint[14].x = 39.4157;
    g_waypoint[14].y = -7.70562;
    g_waypoint[15].x = 42.636;
    g_waypoint[15].y = -7.96717;
    g_waypoint[16].x = 25.6802;
    g_waypoint[16].y = -7.02406;
    g_waypoint[17].x = 32.7937;
    g_waypoint[17].y = 11.687;
    g_waypoint[18].x = 34.2015;
    g_waypoint[18].y = 18.0466;
    g_waypoint[19].x = 35.7156;
    g_waypoint[19].y = 24.0776;
    g_waypoint[20].x = 37.7418;
    g_waypoint[20].y = 32.6346;
    g_waypoint[21].x = 39.8222;
    g_waypoint[21].y = 40.5733;
    g_waypoint[22].x = 40.9479;
    g_waypoint[22].y = 44.38;
    g_waypoint[23].x = 41.5406;
    g_waypoint[23].y = 46.3659;
    g_waypoint[24].x = 36.7649;
    g_waypoint[24].y = 31.739;
    for (int i = 0; i < NUM_WAYPOINT; i++) {
        g_waypoint[i].z = 0;
        g_waypoint[i].x *= 1000.0;
        g_waypoint[i].y *= 1000.0;
    }
}

//void InitGlobalVariables() {
//    g_waypoint[0].x = 0;
//    g_waypoint[0].y = 0;
//    g_waypoint[1].x = 1.797;
//    g_waypoint[1].y = 0;
//    g_waypoint[2].x = 4.499;
//    g_waypoint[2].y = 0.127;
//    g_waypoint[3].x = 9.762;
//    g_waypoint[3].y = 0.268;
//    g_waypoint[4].x = 15.451;
//    g_waypoint[4].y = 0.305;
//    g_waypoint[5].x = 17.389;
//    g_waypoint[5].y = 0.309;
//    g_waypoint[6].x = 20.627;
//    g_waypoint[6].y = 0.315;
//    g_waypoint[7].x = 23.465;
//    g_waypoint[7].y = 0.32;
//    g_waypoint[8].x = 23.467;
//    g_waypoint[8].y = -1.884;
//    g_waypoint[9].x = 23.469;
//    g_waypoint[9].y = -4.16;
//    g_waypoint[10].x = 23.471;
//    g_waypoint[10].y = -7.004;
//    g_waypoint[11].x = 27.881;
//    g_waypoint[11].y = -8.101;
//    g_waypoint[12].x = 30.189;
//    g_waypoint[12].y = -8.01;
//    g_waypoint[13].x = 34.408;
//    g_waypoint[13].y = -8.002;
//    g_waypoint[14].x = 36.873;
//    g_waypoint[14].y = -7.997;
//    g_waypoint[15].x = 39.756;
//    g_waypoint[15].y = -7.992;
//    g_waypoint[16].x = 23.686;
//    g_waypoint[16].y = -7.717;
//    g_waypoint[17].x = 23.989;
//    g_waypoint[17].y = 10.496;
//    g_waypoint[18].x = 23.756;
//    g_waypoint[18].y = 15.693;
//    g_waypoint[19].x = 23.55;
//    g_waypoint[19].y = 22.716;
//    g_waypoint[20].x = 23.72;
//    g_waypoint[20].y = 30.899;
//    g_waypoint[21].x = 23.406;
//    g_waypoint[21].y = 38.486;
//    g_waypoint[22].x = 23.365;
//    g_waypoint[22].y = 42.497;
//    g_waypoint[23].x = 23.329;
//    g_waypoint[23].y = 46.106;
//    g_waypoint[24].x = 23.591;
//    g_waypoint[24].y = 29.231;
//    for (int i = 0; i < NUM_WAYPOINT; i++) {
//        g_waypoint[i].z = 0;
//        g_waypoint[i].x *= 1000.0;
//        g_waypoint[i].y *= 1000.0;
//    }
//}

double GetDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

double GetAngle(double x1, double y1, double x2, double y2, double ori2) {
    double ang = atan2(y1 - y2, x1 - x2) * RAD_TO_DEGREE;
//    cout<<"atan2:"<<ang;
    ang -= ori2;
//    cout<<" -"<<ori2<<":"<<ang;
    if (ang > 180) ang -= 360;
    if (ang < -180) ang += 360;
//    cout<<" final:"<<ang<<endl;
    return ang;
}

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
    MyWindowName = "ChaoZ";
    MyLaserOffset = 125;
    MyLaserMaxRange = 5000;
    int scaleFactor = 15;
    MyScaleX = MyScaleY = scaleFactor;
    MyFrontLength = 313 / scaleFactor;
    MyHalfWidth = 253 / scaleFactor;
    MyRobotPosition = Point((int) (MyLaserMaxRange / MyScaleX),
                            (int) ceil((MyLaserMaxRange + MyLaserOffset) / MyScaleY));
    MyLaserPosition = MyRobotPosition;
    MyLaserPosition.y -= MyLaserOffset / MyScaleY;
    MyBackground = Mat(MyRobotPosition.y + 1,
                       (int) (2 * MyLaserMaxRange / MyScaleX) + 1, CV_8UC3, Scalar(255, 255, 255));
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
}

void StateDisplay::DisplayBackground() {
    imshow(MyWindowName, MyBackground);
    waitKey(10);
}

void StateDisplay::UpdateSurrounding(double *scan) {
    Point obstacle[181];
    double radToDegree = DEGREE_TO_RAD;
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

int StateDisplay::SearchFreeSpace(double *scan, double distThres, int countThres, double angle) {
    angle += 90;
    int count = 0;
    int mid = -1;
    int leftMid = -1;
    double radToDegree = DEGREE_TO_RAD;
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
                if (abs(mid - angle) < abs(leftMid - angle)) {
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

void StateDisplay::AddWayPoint(double dist, double ang) {
    Point waypoint;
    double rad;
    rad = ang * DEGREE_TO_RAD;
    if (dist > MyLaserMaxRange) {
        dist = MyLaserMaxRange-15;
    }
    waypoint.x = ((int) (dist * - sin(rad) / MyScaleX) + MyRobotPosition.x);
    waypoint.y = ((int) (dist * - cos(rad) / MyScaleY) + MyRobotPosition.y);
    if(waypoint.y>MyRobotPosition.y){
        waypoint.y = MyRobotPosition.y;
    }
    circle(MyImage,waypoint,10,Scalar(0,0,255),-1);
}

void StateDisplay::Clear() {
    MyImage = MyBackground.clone();
}

void StateDisplay::SaveImage(string name) {
    imwrite(name,MyImage);
}
//void poseMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg) {
//    g_currentPose = msg.pose.pose;
////    cout << "pose received" << endl;
//}

geometry_msgs::Twist *DriveFreeSpace() {
    geometry_msgs::Twist *msg = new geometry_msgs::Twist;
    msg->linear.y = msg->linear.z = msg->linear.x = 0.0;
    msg->angular.x = msg->angular.y = msg->angular.z = 0.0;
    ros::spinOnce();
    displayer.UpdateSurrounding(g_scan);
    int mid = displayer.SearchFreeSpace(g_scan, 2000, 13, 90.0);
    displayer.DisplayImage();
    msg->linear.x = 0.4;
    for (int i = 45; i < 135; i++) {
        if (g_scan[i] < 500) {
            msg->linear.x = 0;
            break;
        }
    }
    if (mid == -1) {
        msg->angular.z = -90;
        return msg;
    }
    mid = (mid - 90);
    msg->angular.z = mid;

    return msg;
}

geometry_msgs::Twist *DriveWayPoint() {
    static int currentWayPoint = 0;
    static bool reach = false;
    ros::spinOnce();
    geometry_msgs::Twist *msg = new geometry_msgs::Twist;
    msg->linear.y = msg->linear.z = msg->linear.x = 0.0;
    msg->angular.x = msg->angular.y = msg->angular.z = 0.0;
    if (reach) {
        sleep(3);
        // Play Sound or Image!!
        // tourGuide!!
        cout << "reach point " << currentWayPoint << endl;
        reach = false;
        currentWayPoint++;
    }
    if (currentWayPoint >= NUM_WAYPOINT) {
        cout << "reach final" << endl;
        exit(0);
    }
    double dist = GetDistance(g_waypoint[currentWayPoint].x, g_waypoint[currentWayPoint].y,
                              g_currentPose.position.x * 1000.0, g_currentPose.position.y * 1000.0);
    if (dist < DIST_THRES) {
        reach = true;
        stringstream sstm;
        sstm<<"waypoint"<<currentWayPoint<<".jpg";
        displayer.SaveImage(sstm.str());
        return msg;
    }
    double angle = GetAngle(g_waypoint[currentWayPoint].x, g_waypoint[currentWayPoint].y,
                            g_currentPose.position.x * 1000.0, g_currentPose.position.y * 1000.0,
                            g_currentPose.orientation.w);
    displayer.Clear();
    displayer.UpdateSurrounding(g_scan);
    int mid = displayer.SearchFreeSpace(g_scan, 2000, 15, 0);
    displayer.AddWayPoint(dist, angle);
    displayer.DisplayImage();
    msg->linear.x = 0.4;
    cout << "dist to waypoint " << currentWayPoint << " is " << dist << "mm with " << angle << "degree" << endl;
    if (currentWayPoint < 3) {
        if (angle == 0) {
            msg->angular.z = 0;
        }
        if (angle > 0) {
            msg->angular.z = 30;
        }
        if (angle < 0) {
            msg->angular.z = -30;
        }
        if (abs(angle) > 10) {
            msg->linear.x = 0;
        }
        return msg;
    }
    for (int i = 0; i < 181; i++) {
        if (g_scan[i] < 400) {
            msg->linear.x = 0;
            break;
        }
    }
    if(abs(angle)>90){
        msg->linear.x = 0;
        msg->angular.z = angle > 0 ? 30 : -30;
        return msg;
    }
    if (abs(angle) > 45 && g_scan[(int)angle+90]>dist) {
        msg->linear.x = 0;
        msg->angular.z = angle > 0 ? 30 : -30;
        return msg;
    }
    if (mid == -1) {
        msg->angular.z = -30;
        return msg;
    }
    mid = (mid - 90);
    msg->angular.z = mid;

    return msg;
}


void poseMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    g_currentPose = msg.pose.pose;
    g_currentPose.orientation.w = atan2(g_currentPose.orientation.z, g_currentPose.orientation.w) * 2 / M_PI * 180;
    if (g_currentPose.orientation.w < -180) {
        g_currentPose.orientation.w += 360;
    }
    if (g_currentPose.orientation.w > 180) {
        g_currentPose.orientation.w -= 360;
    }
    pub.publish(*DriveWayPoint());
}

void laserMessageReceived(const sensor_msgs::LaserScan &msg) {
    int index = 0;
    for (int i = 90; i < 451; i = i + 2) {
        index = (i - 90) / 2;
        g_scan[index] = msg.ranges[i] * 1000.0;
    }
}

void velMessageReceived(const nav_msgs::Odometry &msg) {
    g_tansVel = msg.twist.twist.linear.x * 1000.0;
    g_rotVel = msg.twist.twist.angular.z / M_PI * 180;
}


int main(int argc, char **argv) {
    InitGlobalVariables();

    ros::init(argc, argv, "state_display");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
    geometry_msgs::Twist msg;
    cout << "publisher to cmd_vel done" << endl;
    ros::Subscriber vel;
    vel = nh.subscribe("RosAria/pose", 1000, &velMessageReceived);
    ros::Subscriber pose;
    pose = nh.subscribe("robot_pose_ekf/odom_combined", 1000, &poseMessageReceived);
//    pose = nh.subscribe("RosAria/pose", 1000, &poseMessageReceived);
    cout << "subscriber to pose done" << endl;
    ros::Subscriber laser;
    laser = nh.subscribe("RosAria/lms1xx_1_laserscan", 1000, &laserMessageReceived);
    cout << "subscriber to laserscan done" << endl;

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    pub.publish(msg);
    cout << "configure state display done" << endl;


    ros::spin();


    return (0);
}

