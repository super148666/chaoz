//
// Created by chaoz on 24/10/17.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <termios.h>

#define KEYCODE_R 100
#define KEYCODE_L 97
#define KEYCODE_U 119
#define KEYCODE_D 115
#define KEYCODE_Q 113
#define KEYCODE_P 112
#define KEYCODE_SPACE 32
#define KEYCODE_RACE 114

geometry_msgs::PoseWithCovarianceStamped::_pose_type::_pose_type g_currentPose;
void positionMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg){
    g_currentPose = msg.pose.pose;
}

//nav_msgs::Odometry::_pose_type::_pose_type g_currentPose;
//void positionMessageReceived(const nav_msgs::Odometry &msg){
//    g_currentPose = msg.pose.pose;
//}

void MoveForward() {
    geometry_msgs::Twist msg;
    msg.linear.x = 250;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    pub.publish(msg);
}

void MoveBackward() {
    geometry_msgs::Twist msg;
    msg.linear.x = -250;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    pub.publish(msg);
}

void TurnRight() {
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = -25;
    pub.publish(msg);
}

void TurnLeft() {
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 25;
    pub.publish(msg);
}

void Stop() {
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    pub.publish(msg);
}

void Train() {
    static int count = 0;
    cout << "training " << count << endl;
    count++;
    ros::spinOnce();
    double maximumScan = 0;
    double inputData[91];
    double trainResult[90];
    for (int i = 0; i < 45; i++) {
        displayer.UpdateSurrounding(g_scan);
        displayer.DisplayImage();
        displayer.AddWayPoint(g_scan[i], i - 90, Scalar(0, 0, 255));
        displayer.DisplayImage();
        inputData[i] = g_scan[i];
        if (maximumScan < g_scan[i]) maximumScan = g_scan[i];
        cout << "1 for door/ 0 for not door:";
        cin >> trainResult[i];
    }
    for (int i = 180; i > 135; i--) {
        displayer.UpdateSurrounding(g_scan);
        displayer.DisplayImage();
        displayer.AddWayPoint(g_scan[i], i - 90, Scalar(0, 0, 255));
        displayer.DisplayImage();
        inputData[180 - i] = g_scan[i];
        if (maximumScan < g_scan[i]) maximumScan = g_scan[i];
        cout << "1 for door/ 0 for not door:";
        cin >> trainResult[180 - i];
    }
    for (int i = 0; i < 90; i++) {
        inputData[i] /= maximumScan;
    }
    inputData[91] = 1.0;
    backprop(inputData,trainResult);
    //train
}

void Detect() {
    double maximumScan = 0;
    double inputData[90];
    for (int i = 0; i < 45; i++) {
        inputData[i] = g_scan[i];
        if (maximumScan < g_scan[i]) maximumScan = g_scan[i];
    }
    for (int i = 180; i > 135; i--) {
        inputData[180 - i] = g_scan[i];
        if (maximumScan < g_scan[i]) maximumScan = g_scan[i];
    }
    for (int i = 0; i < 90; i++) {
        inputData[i] /= maximumScan;
    }
    inputData[91] = 1.0;
    // FeedForward
    feedforward(inputData);
    displayer.Clear();
    displayer.UpdateSurrounding(g_scan);
    displayer.DisplayImage();
    for (int i = 0; i < 45; i++) {
        if (g_out[i] > 0.5) {
            displayer.AddWayPoint(g_scan[i], i - 90, Scalar(0, 0, 255));
//            displayer.DisplayImage();
        }
    }
    for (int i = 180; i > 135; i--) {
        if (g_out[180 - i] > 0.5) {
            displayer.AddWayPoint(g_scan[i], i - 90, Scalar(0, 0, 255));
//            displayer.DisplayImage();
        }
    }
    displayer.DisplayImage();
}

void Print() {
    //Print weight or save in file;
    ofstream fs;
    fs.open("weight1.csv");
    for(int i=0;i<NN_NUM_INPUT;i++){
        for(int j=0;j<NN_NUM_HIDEN_NN;j++){
            fs<<g_weight1[i][j]<<endl;
        }
    }
    fs.close();
    fs.open("weight2.csv");
    for(int i=0;i<NN_NUM_HIDEN_NN;i++){
        for(int j=0;j<NN_NUM_OUTPUT;j++){
            fs<<g_weight2[i][j]<<endl;
        }
    }
    fs.close();
    cout<<"print done"<<endl;

}

class TeleopRosAria
{
public:
    TeleopRosAria();
    void keyLoop();
private:
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    ros::Publisher twist_pub_;
    ros::Subscriber pose_sub_;
};
TeleopRosAria::TeleopRosAria():
        linear_(0),
        angular_(0),
        l_scale_(2.0),
        a_scale_(2.0)
{
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
    pose_sub_ = nh_.subscribe("robot_pose_ekf/odom_combined",1,&positionMessageReceived);
//    pose_sub_ = nh_.subscribe("RosAria/pose",1,&positionMessageReceived);
}
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}
void TeleopRosAria::keyLoop()
{
    static bool raceMode = false;
    l_scale_ = 5000;
    a_scale_ = 500;
    if(raceMode){
        l_scale_ = 10000;
        a_scale_ = 1000;
    }

    static int count = 0;
    char c;
    bool dirty=false;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");
    puts("Press the space bar to stop the robot.");
    puts("Press q to stop the program");
    for(;;)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        linear_=angular_=0;
        ROS_DEBUG("value: 0x%02X\n", c);
        switch(c)
        {
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                angular_ = 0.1;
                linear_ = 0;
                dirty = true;
                break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                angular_ = -0.1;
                linear_ = 0;
                dirty = true;
                break;
            case KEYCODE_U:
                ROS_DEBUG("UP");
                linear_ = 0.1;
                angular_ = 0;
                dirty = true;
                break;
            case KEYCODE_D:
                ROS_DEBUG("DOWN");
                linear_ = -0.1;
                angular_ = 0;
                dirty = true;
                break;
            case KEYCODE_SPACE:
                ROS_DEBUG("STOP");
                linear_ = 0;
                angular_ = 0;
                dirty = true;
                break;
            case KEYCODE_Q:
                ROS_DEBUG("QUIT");
                ROS_INFO_STREAM("You quit the teleop successfully");
                return;
                break;
            case KEYCODE_P:
                ros::spinOnce();
                std::cout<<"g_waypoint["<<count<<"].x = "<<g_currentPose.position.x<<";"<<std::endl;
                std::cout<<"g_waypoint["<<count<<"].y = "<<g_currentPose.position.y<<";"<<std::endl;
                count++;
                break;
            case KEYCODE_RACE:
                if(!raceMode){
                    raceMode = true;
                    std::cout<<"Race Mode ON!!"<<std::endl;
                }else{
                    raceMode = false;
                    std::cout<<"Race Mode OFF."<<std::endl;
                }
                break;
        }
        geometry_msgs::Twist twist;
        twist.angular.z = a_scale_*angular_;
        twist.linear.x = l_scale_*linear_;
        if(dirty ==true)
        {
            twist_pub_.publish(twist);
            dirty=false;
        }
    }
    return;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_RosAria");
    TeleopRosAria teleop_RosAria;
    signal(SIGINT,quit);
    teleop_RosAria.keyLoop();
    return(0);
}
