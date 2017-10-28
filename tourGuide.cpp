//
// Created by chaoz on 26/10/17.
//

#include <Aria.h>
#include <fstream>
#include <signal.h>
#include <termios.h>
//#include "QRDetect.cpp"
#include "StateDisplay.h"

#define KEYCODE_D 100
#define KEYCODE_A 97
#define KEYCODE_W 119
#define KEYCODE_S 115
#define KEYCODE_Q 113
#define KEYCODE_P 112
#define KEYCODE_SPACE 32
#define KEYCODE_R 114
#define KEYCODE_T 116

#define SUBTRACT_HISTORY 50 // compared histroy of image in subtractor
#define SUBTRACT_THRESHOLD 5 // threshold value for subtractor

#define NUM_WAYPOINT 25
#define DIST_THRES 1000
#define DELTA_SCAN 200.0
#define NUM_STOP 1

nav_msgs::Odometry::_pose_type::_pose_type g_currentPose;
StateDisplay displayer(0);
double g_scan[181];
ros::Publisher pub;
double g_linear = 0.0;
double g_angular = 0.0;
bool g_leftOpen = false;
bool g_rightOpen = false;
bool g_frontOpen = false;
int kfd = 0;
struct termios cooked, raw;
VideoCapture g_cap;

class MotionDetector {
public:
    MotionDetector(int history = 500, double varThreshold = 16,
                   bool detectShadows = true) {
        bgSubtractor = BackgroundSubtractorMOG2(history, varThreshold, detectShadows);
    }

    void imShowForeground(int windowSizeX = 0, int windowSizeY = 0) {

        if ((windowSizeX == 0) & (windowSizeY == 0)) {
            namedWindow("foreground");
        } else {
            namedWindow("foreground", WINDOW_NORMAL);
        }

        resizeWindow("foreground", windowSizeX, windowSizeY);
        imshow("foreground", foreground);
    }

    void imShowBackground(int windowSizeX = 0, int windowSizeY = 0) {

        if ((windowSizeX == 0) & (windowSizeY == 0)) {
            namedWindow("background");
        } else {
            namedWindow("background", WINDOW_NORMAL);
        }

        resizeWindow("background", windowSizeX, windowSizeY);
        imshow("background", background);
    }

    int Detect(Mat* frame) {
        int count=0;
        this->bgSubtractor.operator()(*frame,this->foreground);
        this->bgSubtractor.getBackgroundImage(this->background);
        erode(this->foreground, this->foreground, Mat());
        dilate(this->foreground, this->foreground, Mat());
        findContours(this->foreground, this->contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        drawContours(*frame, this->contours, -1, Scalar(0, 0, 255), 2);
        for(vector<vector<Point> >::iterator it = this->contours.begin();it!=this->contours.end();it++){
            count += (*it).size();
        }
//        cout<<count<<endl;
        return count;
    }


private:
    BackgroundSubtractorMOG2 bgSubtractor;
    Mat foreground;
    Mat background;
    vector<vector<Point> > contours;
};

void keyLoop();

geometry_msgs::Twist *DriveFreeSpace();

void poseMessageReceived(const nav_msgs::Odometry &msg);

void laserMessageReceived(const sensor_msgs::LaserScan &msg);

int LeftDetect();

int RightDetect();

int main(int argc, char **argv) {
//    InitGlobalVariables();
    if (!g_cap.open(1)) {
        cout << "Unable to open webcam\n";
        exit(EXIT_FAILURE);
    }
    ros::init(argc, argv, "tourGuide");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
    geometry_msgs::Twist msg;
    cout << "publisher to cmd_vel done" << endl;
    ros::Subscriber vel;
    ros::Subscriber pose;
    pose = nh.subscribe("RosAria/pose", 1000, &poseMessageReceived);
    cout << "subscriber to pose done" << endl;
    ros::Subscriber laser;
//    laser = nh.subscribe("RosAria/lms1xx_1_laserscan", 1000, &laserMessageReceived);
    laser = nh.subscribe("RosAria/sim_lms1xx_1_laserscan", 1000, &laserMessageReceived);
    cout << "subscriber to laserscan done" << endl;

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    pub.publish(msg);
    cout << "configure state display done" << endl;



    keyLoop();

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
    int index = 0;
    for (int i = 90; i < 451; i = i + 2) {
        index = (i - 90) / 2;
        g_scan[index] = msg.ranges[i] * 1000.0;
    }
    for (int i = 0; i < 60; i++) {
        if (g_scan[i] > 8000) {
            g_rightOpen = true;
            break;
        }
    }
    for (int i = 60; i < 120; i++) {
        if (g_scan[i] > 8000) {
            g_frontOpen = true;
            break;
        }
    }
    for (int i = 120; i < 181; i++) {
        if (g_scan[i] > 8000) {
            g_leftOpen = true;
            break;
        }
    }
    LeftDetect();
    RightDetect();
}

geometry_msgs::Twist *DriveFreeSpace() {
    int mid;
    static set <string> visited;
    geometry_msgs::Twist *msg = new geometry_msgs::Twist;
    msg->linear.y = msg->linear.z = msg->linear.x = 0.0;
    msg->angular.x = msg->angular.y = msg->angular.z = 0.0;
    ros::spinOnce();
    displayer.Clear();
    displayer.UpdateSurrounding(g_scan);
    mid = displayer.SearchFreeSpace(g_scan, 2000, 20, 90);
    if (g_rightOpen) {
//        cout<<"right"<<endl;
        if(mid>100)
        mid = displayer.SearchFreeSpace(g_scan, 2000, 20, 0);
    } else if (g_leftOpen) {
//        cout<<"left"<<endl;
        if(mid<80)
        mid = displayer.SearchFreeSpace(g_scan, 2000, 20, 180);
    }
    displayer.MotionEstimate(g_linear,g_angular);
    for(int i=0;i<1;i++) {
        string QRMessage = displayer.readQR();
        if (!QRMessage.empty()) {
            //check if it's in the visited set
            if(!(visited.find(QRMessage) != visited.end())){
                visited.insert(QRMessage);
                pub.publish(*msg);
                ros::spinOnce();
                displayer.Clear();
                displayer.UpdateSurrounding(g_scan);
                displayer.MotionEstimate(g_linear,g_angular);
                displayer.AddRoomText(QRMessage);
                displayer.DisplayImage();
                system("play -q /usr/local/Aria/examples/sound-computers.wav");
                cout << "get code: " << QRMessage << endl;
                sleep(2);
            }
        }
    }
    displayer.DisplayImage();
    msg->linear.x = 0.25;
//    for (int i = 75; i < 105; i++) {
//        if (g_scan[i] < 650) {
//            msg->linear.x = 0;
//            break;
//        }
//    }
    if (mid == -1) {
        msg->linear.x = 0;
        msg->angular.z = g_linear < 0.1 ? 90 : 0;
        return msg;
    }
    mid = (mid - 90);
    msg->angular.z = mid;
    if (mid > 0) {
//        for (int i = 180; i > 90; i--) {
//            if (g_scan[i] < 500) {
//                msg->linear.x = 0;
//            }
//        }
    }
    if (mid > 30) {
        msg->linear.x /= 0.20;
    }
    if (mid > 60) {
        msg->linear.x /= 0.15;
    }

    if (mid < 0) {
//        for (int i = 45; i < 90; i++) {
//            if (g_scan[i] < 400) {
//                msg->linear.x = 0;
//            }
//        }
    }
    if (mid < -30) {
        msg->linear.x /= 0.2;
    }
    if (mid < -60) {
        msg->linear.x /= 0.15;
    }
//    for (int i = 45; i < 135; i++) {
//        if (g_scan[i] < 400) {
//            msg->linear.x = 0;
//            break;
//        }
//    }
    msg->angular.z = (msg->linear.x == 0) && (g_linear != 0) ? 0 : msg->angular.z;
    return msg;
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
        return 0;
    }

    if (minimumScan < 0.9 * inputData[0]) {
        return 0;
    }
    if (maximumScan - minimumScan > 100) {
        for (int i = 180; i > 135; i--) {
            if (inputData[180 - i] > averageScan) {
                displayer.AddLaserPoint(g_scan[i], i - 90, Scalar(0, 0, 255));
                count++;
            }
        }
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
        return 0;
    }

    if (minimumScan < 0.9 * inputData[0]) {
        return 0;
    }

    if (maximumScan - minimumScan > 100) {
        for (int i = 0; i < 45; i++) {
            if (inputData[i] > averageScan) {
                displayer.AddLaserPoint(g_scan[i], i - 90, Scalar(0, 0, 255));
                count++;
            }
        }
        return count;
    }
}

void keyLoop() {
    int c;
    bool deadManSwitch = true;
    bool dirty=false;
    bool motion = false;
    bool firstStart = true;
    geometry_msgs::Twist twist;
    Mat frame;
    g_cap>>frame;
    Size frameSize(40, 30);
    MotionDetector motionDetector = MotionDetector(SUBTRACT_HISTORY, SUBTRACT_THRESHOLD);
    // get the console in raw mode
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");
    puts("Press the space bar to stop the robot.");
    puts("Press q to stop the program");
    for(;;)
    {
        ros::spinOnce();
        c=waitKey(1);
        if(deadManSwitch) {
            do{
                c = waitKey(3);
                displayer.Clear();
                ros::spinOnce();
                displayer.UpdateSurrounding(g_scan);
                displayer.MotionEstimate(g_linear,g_angular);
                displayer.DisplayImage();
            }while(c==-1);
            switch (c) {
                case KEYCODE_A:
                    ROS_DEBUG("LEFT");
                    twist.angular.z = 50;
                    twist.linear.x = g_linear;
                    dirty = true;
                    break;
                case KEYCODE_D:
                    ROS_DEBUG("RIGHT");
                    twist.angular.z = -50;
                    twist.linear.x = g_linear;
                    dirty = true;
                    break;
                case KEYCODE_W:
                    ROS_DEBUG("UP");
                    twist.angular.z = 0;
                    twist.linear.x = 0.5;
                    dirty = true;
                    break;
                case KEYCODE_S:
                    ROS_DEBUG("DOWN");
                    twist.angular.z = 0;
                    twist.linear.x = -0.5;
                    dirty = true;
                    break;
                case KEYCODE_SPACE:
                    ROS_DEBUG("STOP");
                    twist.angular.z = 0;
                    twist.linear.x = 0;
                    dirty = true;
                    break;
                case KEYCODE_Q:
                    ROS_DEBUG("QUIT");
                    ROS_INFO_STREAM("You quit the program successfully");
                    return;
                    break;
                case KEYCODE_T:
                    ROS_INFO("DEAD_MAN_SWITCH : OFF");
                    displayer.AddRoomText(" ",true);
                    deadManSwitch = false;
                    firstStart = true;
                    break;
                default:
                    break;
            }
            if (dirty == true) {
                pub.publish(twist);
                dirty = false;
            }
        }else{
            if(firstStart){
                namedWindow("webcam", WINDOW_NORMAL);
                resizeWindow("webcam", frame.cols, frame.rows);
                do {
                    imshow("webcam", frame);
                    g_cap >> frame;
                    if (frame.empty()) break; // end of video stream
                    resize(frame, frame, frameSize);
                }while(motionDetector.Detect(&frame)>10);
                firstStart = false;
            }
            while (!motion) {
                g_cap >> frame;
                if (frame.empty()) break; // end of video stream
                resize(frame, frame, frameSize);
                if(motionDetector.Detect(&frame)>50) motion = true;
                imshow("webcam", frame);
                waitKey(1);
                if(motion){
                    motion = false;
                    system("play -q /usr/local/Aria/examples/sound-computers.wav");
                    // play sound to ask
                    cout<<"first motion"<<endl;
                    for(int i=0;i<50;i++){
                        g_cap >> frame;
                        if (frame.empty()) break; // end of video stream
                        resize(frame, frame, frameSize);
                        motionDetector.Detect(&frame);
                        imshow("webcam", frame);
                        waitKey(1);
                    }
                    while (!motion) {
                        g_cap >> frame;
                        if (frame.empty()) break; // end of video stream
                        resize(frame, frame, frameSize);
                        if(motionDetector.Detect(&frame)>15) motion = true;
                        imshow("webcam", frame);
                        waitKey(1);
                    }
                    cout<<"second motion"<<endl;
                    destroyWindow("webcam");
                }
            }
            do{
                c = waitKey(3);
                pub.publish(*DriveFreeSpace());
            }while(c==-1);
            switch(c)
            {
                case KEYCODE_T:
                    ROS_INFO("DEAD_MAN_SWITCH : ON");
                    deadManSwitch = true;
                    break;
                case KEYCODE_Q:
                    ROS_DEBUG("QUIT");
                    ROS_INFO_STREAM("You quit the program successfully");
                    return;
                    break;
                default:
                    break;
            }
        }
    }
}