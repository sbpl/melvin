#ifndef SERVO_NODE_H
#define SERVO_NODE_H

#include <string>
#include <vector>

#include <segbot_msgs/ServoCmd.h>
#include <ros/ros.h>

#include <servo_node/Dynamixel.h>
#include <servo_node/Timer.h>

class ServoNode
{
public:

    ServoNode();
    ~ServoNode();
    int initialize();
    void cmdCallback(const segbot_msgs::ServoCmdConstPtr& msg);
    int updateServo();

private:

    std::vector<ros::Subscriber> cmd_sub;
    std::vector<ros::Publisher> angle_pub;
    ros::NodeHandle nh;

    std::string dev;
    int baudRate;
    std::vector<int> moduleId;

    std::vector<int> state;
    std::vector<float> minAngle;  //degrees
    std::vector<float> maxAngle;  //degrees
    std::vector<float> vel;   //degrees per second
    std::vector<float> desAngle;
    std::vector<int> dir;
    std::vector<bool> scan;
    float reversePoint; //degrees away from the goal to start reversing

    std::vector<Upenn::Dynamixel> dynamixel;
    Upenn::Timer cmdTimer;
    std::vector<float> position;
};

#endif
