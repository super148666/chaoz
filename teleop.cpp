//
// Created by chaoz on 24/10/17.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <termios.h>



class TeleopRosAria
{
public:
    TeleopRosAria();
    void keyLoop();
private:
    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
    ros::Subscriber pose_sub_;
};
TeleopRosAria::TeleopRosAria()
{
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
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
            case KEYCODE_A:
                ROS_DEBUG("LEFT");
                angular_ = 0.1;
                linear_ = 0;
                dirty = true;
                break;
            case KEYCODE_D:
                ROS_DEBUG("RIGHT");
                angular_ = -0.1;
                linear_ = 0;
                dirty = true;
                break;
            case KEYCODE_W:
                ROS_DEBUG("UP");
                linear_ = 0.1;
                angular_ = 0;
                dirty = true;
                break;
            case KEYCODE_S:
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
                break;
            case KEYCODE_R:
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
//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "teleop_RosAria");
//    TeleopRosAria teleop_RosAria;
//    signal(SIGINT,quit);
//    teleop_RosAria.keyLoop();
//    return(0);
//}
