//
// Created by chaoz on 26/10/17.
//

#include <Aria.h>
#include <fstream>
#include <signal.h>
#include <termios.h>
#include "QRDetect.cpp"
#include "StateDisplay.h"

#define NUM_WAYPOINT 25
#define DIST_THRES 1000
#define DELTA_SCAN 200.0
#define NUM_STOP 1

//geometry_msgs::PoseWithCovarianceStamped::_pose_type::_pose_type g_currentPose;
nav_msgs::Odometry::_pose_type::_pose_type g_currentPose;
StateDisplay displayer;
//geometry_msgs::Pose::_position_type g_waypoint[NUM_WAYPOINT];
double g_scan[181];
ros::Publisher pub;
double g_linear = 0.0;
double g_angular = 0.0;
bool g_leftOpen = false;
bool g_rightOpen = false;
bool g_frontOpen = false;
QRDetector g_giveMeCode(1);

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
//void InitGlobalVariables() {
//    g_waypoint[0].x = 0;
//    g_waypoint[0].y = 0;
//    g_waypoint[1].x = 2.80797;
//    g_waypoint[1].y = -0.0161297;
//    g_waypoint[2].x = 5.723;
//    g_waypoint[2].y = 0.0469143;
//    g_waypoint[3].x = 11.1869;
//    g_waypoint[3].y = 0.342565;
//    g_waypoint[4].x = 16.9077;
//    g_waypoint[4].y = 0.132684;
//    g_waypoint[5].x = 19.358;
//    g_waypoint[5].y = 0.287418;
//    g_waypoint[6].x = 22.178;
//    g_waypoint[6].y = 0.335201;
//    g_waypoint[7].x = 25.3527;
//    g_waypoint[7].y = 0.495888;
//    g_waypoint[8].x = 25.4903;
//    g_waypoint[8].y = -1.95358;
//    g_waypoint[9].x = 25.6442;
//    g_waypoint[9].y = -4.41105;
//    g_waypoint[10].x = 25.9217;
//    g_waypoint[10].y = -7.71921;
//    g_waypoint[11].x = 30.3027;
//    g_waypoint[11].y = -7.96171;
//    g_waypoint[12].x = 32.6672;
//    g_waypoint[12].y = -7.80257;
//    g_waypoint[13].x = 37.0167;
//    g_waypoint[13].y = -7.61941;
//    g_waypoint[14].x = 39.4157;
//    g_waypoint[14].y = -7.70562;
//    g_waypoint[15].x = 42.636;
//    g_waypoint[15].y = -7.96717;
//    g_waypoint[16].x = 25.6802;
//    g_waypoint[16].y = -7.02406;
//    g_waypoint[17].x = 32.7937;
//    g_waypoint[17].y = 11.687;
//    g_waypoint[18].x = 34.2015;
//    g_waypoint[18].y = 18.0466;
//    g_waypoint[19].x = 35.7156;
//    g_waypoint[19].y = 24.0776;
//    g_waypoint[20].x = 37.7418;
//    g_waypoint[20].y = 32.6346;
//    g_waypoint[21].x = 39.8222;
//    g_waypoint[21].y = 40.5733;
//    g_waypoint[22].x = 40.9479;
//    g_waypoint[22].y = 44.38;
//    g_waypoint[23].x = 41.5406;
//    g_waypoint[23].y = 46.3659;
//    g_waypoint[24].x = 36.7649;
//    g_waypoint[24].y = 31.739;
//    for (int i = 0; i < NUM_WAYPOINT; i++) {
//        g_waypoint[i].z = 0;
//        g_waypoint[i].x *= 1000.0;
//        g_waypoint[i].y *= 1000.0;
//    }
//}

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

double GetDistance(double x1, double y1, double x2, double y2);

double GetAngle(double x1, double y1, double x2, double y2, double ori2);

geometry_msgs::Twist *DriveFreeSpace();

geometry_msgs::Twist *DriveWayPoint();

//void poseMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg) {
//    g_currentPose = msg.pose.pose;
//    g_currentPose.orientation.w = atan2(g_currentPose.orientation.z, g_currentPose.orientation.w) * 2 / M_PI * 180;
//    if (g_currentPose.orientation.w < -180) {
//        g_currentPose.orientation.w += 360;
//    }
//    if (g_currentPose.orientation.w > 180) {
//        g_currentPose.orientation.w -= 360;
//    }
//    pub.publish(*DriveWayPoint());
//}

void poseMessageReceived(const nav_msgs::Odometry &msg);

void laserMessageReceived(const sensor_msgs::LaserScan &msg);

int LeftDetect();

int RightDetect();

int main(int argc, char **argv) {
//    InitGlobalVariables();

    ros::init(argc, argv, "tourGuide");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
    geometry_msgs::Twist msg;
    cout << "publisher to cmd_vel done" << endl;
    ros::Subscriber vel;
    ros::Subscriber pose;
//    pose = nh.subscribe("robot_pose_ekf/odom_combined", 1000, &poseMessageReceived);
    pose = nh.subscribe("RosAria/pose", 1000, &poseMessageReceived);
    cout << "subscriber to pose done" << endl;
    ros::Subscriber laser;
    laser = nh.subscribe("RosAria/lms1xx_1_laserscan", 1000, &laserMessageReceived);
//    laser = nh.subscribe("RosAria/sim_lms1xx_1_laserscan", 1000, &laserMessageReceived);
    cout << "subscriber to laserscan done" << endl;

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    pub.publish(msg);
    cout << "configure state display done" << endl;

//    while(waitKey(10)!=27){
//        displayer.DisplayBackground();
//    }
//    ros::spin();
    displayer.Clear();
    displayer.UpdateSurrounding(g_scan);
    while(waitKey(10)!=27) {
        pub.publish(*DriveFreeSpace());
    }
    return (0);
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
//    pub.publish(*DriveWayPoint());
}

void laserMessageReceived(const sensor_msgs::LaserScan &msg) {
    static
    int index = 0;
    int count;
    for (int i = 90; i < 451; i = i + 2) {
        index = (i - 90) / 2;
        g_scan[index] = msg.ranges[i] * 1000.0;
    }
    count = 0;
    for (int i = 0; i < 60; i++) {
        if (g_scan[i] > 5000) {
            count++;
        }
    }
    g_rightOpen = count > 15;
    count = 0;
    for (int i = 60; i < 120; i++) {
        if (g_scan[i] > 5000) {
            count++;
        }
    }
    g_frontOpen = count > 15;
    count = 0;
    for (int i = 120; i < 181; i++) {
        if (g_scan[i] > 5000) {
            count++;
        }
    }
    g_leftOpen = count > 15;
}

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


geometry_msgs::Twist *DriveFreeSpace() {
    int mid;
    static set<string> visited;
    geometry_msgs::Twist *msg = new geometry_msgs::Twist;
    msg->linear.y = msg->linear.z = msg->linear.x = 0.0;
    msg->angular.x = msg->angular.y = msg->angular.z = 0.0;
    ros::spinOnce();
    displayer.Clear();
    displayer.UpdateSurrounding(g_scan);
    displayer.MotionEstimate(g_linear,g_angular);
    if (g_rightOpen) {
        cout<<"right"<<endl;
        mid = displayer.SearchFreeSpace(g_scan, 2500, 12, 90);
        if(mid>120)
        mid = displayer.SearchFreeSpace(g_scan, 2500, 12, 0);
    } else if (g_frontOpen) {
        cout<<"front"<<endl;
        mid = displayer.SearchFreeSpace(g_scan, 2500, 12, 90);
    } else if (g_leftOpen) {
        cout<<"left"<<endl;
        mid = displayer.SearchFreeSpace(g_scan, 2500, 12, 180);
    } else {
        cout<<"front"<<endl;
        mid = displayer.SearchFreeSpace(g_scan, 2500, 12, 90);
    }
    displayer.DisplayImage();
    for(int i=0;i<1;i++) {
        string QRMessage = g_giveMeCode.readQR();
        if (!QRMessage.empty()) {
            //check if it's in the visited set
            if(!(visited.find(QRMessage) != visited.end())){
              visited.insert(QRMessage);
              pub.publish(*msg);
              while (g_linear > 100) {
                  ros::spinOnce();
              }
              system("play -q beep.wav");
              cout << "get code " << QRMessage << endl;
              sleep(5);
            }
        }
    }
    msg->linear.x = 0.4;
    for (int i = 75; i < 105; i++) {
        if (g_scan[i] < 650) {
            msg->linear.x = 0;
            break;
        }
    }
    if (mid == -1) {
        msg->linear.x = 0;
        msg->angular.z = g_linear < 0.1 ? 90 : 0;
        return msg;
    }
    mid = (mid - 90);
    msg->angular.z = mid;
    if (mid > 0) {
        for (int i = 180; i > 90; i--) {
            if (g_scan[i] < 500) {
                msg->linear.x = 0;
            }
        }
    }
    if (mid > 30) {
        msg->linear.x /= 0.40;
    }
    if (mid > 60) {
        msg->linear.x /= 0.35;
    }

    if (mid < 0) {
        for (int i = 45; i < 90; i++) {
            if (g_scan[i] < 400) {
                msg->linear.x = 0;
            }
        }
    }
    if (mid < -30) {
        msg->linear.x /= 0.4;
    }
    if (mid < -60) {
        msg->linear.x /= 0.35;
    }
    for (int i = 45; i < 135; i++) {
        if (g_scan[i] < 400) {
            msg->linear.x = 0;
            break;
        }
    }
    msg->angular.z = (msg->linear.x == 0) && (g_linear != 0) ? 0 : msg->angular.z;
    return msg;
}

//geometry_msgs::Twist *DriveWayPoint() {
//    static int currentWayPoint = 0;
//    static bool reach = false;
//    ros::spinOnce();
//    geometry_msgs::Twist *msg = new geometry_msgs::Twist;
//    msg->linear.y = msg->linear.z = msg->linear.x = 0.0;
//    msg->angular.x = msg->angular.y = msg->angular.z = 0.0;
//    if (reach) {
//        sleep(3);
//        // Play Sound or Image!!
//        // tourGuide!!
//        cout << "reach point " << currentWayPoint << endl;
//        reach = false;
//        currentWayPoint++;
//    }
//
//    // passing first narrow door;
//    if (currentWayPoint == 1) {
//        double dist = GetDistance(g_waypoint[currentWayPoint].x, g_waypoint[currentWayPoint].y,
//                                  g_currentPose.position.x * 1000.0, g_currentPose.position.y * 1000.0);
//        double angle = GetAngle(g_waypoint[currentWayPoint].x, g_waypoint[currentWayPoint].y,
//                                g_currentPose.position.x * 1000.0, g_currentPose.position.y * 1000.0,
//                                g_currentPose.orientation.w);
//        displayer.Clear();
//        displayer.UpdateSurrounding(g_scan);
//        int mid = displayer.SearchFreeSpace(g_scan, 5000, 5, 0);
//        displayer.AddWayPoint(dist, angle, Scalar(0, 0, 255));
//        displayer.DisplayImage();
//        if (mid == -1) {
//            msg->angular.z = -30;
//            return msg;
//        }
//        mid = (mid - 90);
//        msg->angular.z = mid;
//
//        return msg;
//    }
//
//    if (currentWayPoint >= NUM_WAYPOINT) {
//        cout << "reach final" << endl;
//        exit(0);
//    }
//
//    double dist = GetDistance(g_waypoint[currentWayPoint].x, g_waypoint[currentWayPoint].y,
//                              g_currentPose.position.x * 1000.0, g_currentPose.position.y * 1000.0);
//    double angle = GetAngle(g_waypoint[currentWayPoint].x, g_waypoint[currentWayPoint].y,
//                            g_currentPose.position.x * 1000.0, g_currentPose.position.y * 1000.0,
//                            g_currentPose.orientation.w);
//    if (dist < DIST_THRES) {
//        reach = true;
//        stringstream sstm;
//        sstm << "waypoint" << currentWayPoint << ".jpg";
//        displayer.SaveImage(sstm.str());
//        return msg;
//    }
//
//
//    displayer.Clear();
//    displayer.UpdateSurrounding(g_scan);
//    int mid = displayer.SearchFreeSpace(g_scan, 2000, 15, 0);
//    displayer.AddWayPoint(dist, angle, Scalar(0, 0, 255));
//    displayer.DisplayImage();
//    msg->linear.x = 0.4;
//    cout << "dist to waypoint " << currentWayPoint << " is " << dist << "mm with " << angle << "degree" << endl;
//
//    if (currentWayPoint < 3) {
//        if (angle == 0) {
//            msg->angular.z = 0;
//        }
//        if (angle > 0) {
//            msg->angular.z = 30;
//        }
//        if (angle < 0) {
//            msg->angular.z = -30;
//        }
//        if (abs(angle) > 10) {
//            msg->linear.x = 0;
//        }
//        return msg;
//    }
//    if (currentWayPoint > 3) {
//
//        for (int i = 0; i < 181; i++) {
//            if (g_scan[i] < 400) {
//                msg->linear.x = 0;
//                break;
//            }
//        }
//    }
//    if (abs(angle) > 90) {
//        msg->linear.x = 0;
//        msg->angular.z = angle > 0 ? 30 : -30;
//        return msg;
//    }
//    if (abs(angle) > 45 && g_scan[(int) angle + 90] > dist) {
//        msg->linear.x = 0;
//        msg->angular.z = angle > 0 ? 30 : -30;
//        return msg;
//    }
//    if (mid == -1) {
//        msg->angular.z = -30;
//        return msg;
//    }
//    mid = (mid - 90);
//    msg->angular.z = mid;
//
//    return msg;
//}

void Detect() {
    ros::spinOnce();
    displayer.Clear();
    displayer.UpdateSurrounding(g_scan);
    bool doorLeft = false;
    bool doorRight = false;
    //left
    doorLeft = LeftDetect();
    //right
    doorRight = RightDetect();

}

int LeftDetect() {
    double inputData[45];
    double maximumScan = 0.0;
    double minimumScan = 20000.0;
    double averageScan = 0.0;
    double sumScan = 0.0;
    int count = 0;
    for (int i = 180; i > 135; i--) {
        inputData[180 - i] = (g_scan[i] * cos((180 - i) * M_PI / 180));
        sumScan += inputData[180 - i];
        if (maximumScan < inputData[180 - i]) maximumScan = inputData[180 - i];
        if (minimumScan > inputData[180 - i]) {
            minimumScan = inputData[180 - i];
        }
    }
    averageScan = sumScan / 45;

    if (maximumScan - minimumScan < 80 || maximumScan - minimumScan > minimumScan) {
        displayer.DisplayImage();
        return 0;
    }

    if (minimumScan < 0.9 * inputData[0]) {
        displayer.DisplayImage();
        return 0;
    }
    if (maximumScan - minimumScan > 100) {
        for (int i = 180; i > 135; i--) {
            if (inputData[180 - i] > averageScan) {
                displayer.AddLaserPoint(g_scan[i], i - 90, Scalar(0, 0, 255));
                count++;
            }
        }
        displayer.DisplayImage();
        return count;
    }
}

int RightDetect() {
    int count = 0;
    double inputData[45];
    double maximumScan = 0.0;
    double minimumScan = 20000.0;
    double averageScan = 0.0;
    double sumScan = 0.0;
    for (int i = 0; i < 45; i++) {
        inputData[i] = (g_scan[i] * cos((i) * M_PI / 180));
        sumScan += inputData[i];
        if (maximumScan < inputData[i]) maximumScan = inputData[i];
        if (minimumScan > inputData[i]) {
            minimumScan = inputData[i];
        }
    }
    averageScan = sumScan / 45;

    if (maximumScan - minimumScan < 80 || maximumScan - minimumScan > minimumScan) {
        displayer.DisplayImage();
        return 0;
    }

    if (minimumScan < 0.9 * inputData[0]) {
        displayer.DisplayImage();
        return 0;
    }

    if (maximumScan - minimumScan > 100) {
        for (int i = 0; i < 45; i++) {
            if (inputData[i] > averageScan) {
                displayer.AddLaserPoint(g_scan[i], i - 90, Scalar(0, 0, 255));
                count++;
            }
        }
        displayer.DisplayImage();
        return count;
    }
}

//
//int main (int argc, const char * argv[])
//{
//    VideoCapture cap=VideoCapture(0);
//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
//    if (!cap.isOpened())
//        return -1;
//
//    Mat frame;
//    HOGDescriptor hog;
//    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
//
//    namedWindow("video capture", CV_WINDOW_AUTOSIZE);
//    while (true)
//    {
//        cap >> frame;
//        if (!frame.data)
//            continue;
//        vector<Rect> detected, detected_filtered;
//        hog.detectMultiScale(frame, detected, 0, Size(8,8), Size(16,16), 1.06, 2);
//        size_t i, j;
//        /*checking for the distinctly detected human in a frame*/
//        for (i=0; i<detected.size(); i++)
//        {
//            Rect r = detected[i];
//            for (j=0; j<detected.size(); j++)
//                if (j!=i && (r & detected[j]) == r)
//                    break;
//            if (j== detected.size())
//                detected_filtered.push_back(r);
//        }
//        /*for each distinctly detected human draw rectangle around it*/
//        for (i=0; i<detected_filtered.size(); i++)
//        {
//            Rect r = detected_filtered[i];
//            r.x += cvRound(r.width*0.1);
//            r.width = cvRound(r.width*0.8);
//            r.y += cvRound(r.height*0.07);
//            r.height = cvRound(r.height*0.8);
//            rectangle(frame, r.tl(), r.br(), Scalar(0,0,255), 2);
//        }        imshow("video capture", frame);
//        if (waitKey(10) >= 0)
//            break;
//    }
//    return 0;
//}
