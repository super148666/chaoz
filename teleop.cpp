//
// Created by chaoz on 24/10/17.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <signal.h>
#include <termios.h>

#define KEYCODE_R 100
#define KEYCODE_L 97
#define KEYCODE_U 119
#define KEYCODE_D 115
#define KEYCODE_Q 0x71
#define KEYCODE_P 0x70
#define KEYCODE_SPACE 0x20

geometry_msgs::PoseWithCovarianceStamped::_pose_type::_pose_type g_currentPose;
void positionMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg){
    g_currentPose = msg.pose.pose;
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
}
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_RosAria");
    TeleopRosAria teleop_RosAria;
    signal(SIGINT,quit);
    teleop_RosAria.keyLoop();
    return(0);
}
void TeleopRosAria::keyLoop()
{
    l_scale_ = 5000;
    a_scale_ = 500;
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
                std::cout<<"g_wayPoint["<<count<<"].x = "<<g_currentPose.position.x<<";"<<std::endl;
                std::cout<<"g_wayPoint["<<count<<"].y = "<<g_currentPose.position.y<<";"<<std::endl;
                count++;
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