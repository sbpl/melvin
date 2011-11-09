#ifndef SERVO_NODE_H
#define SERVO_NODE_H

#include <segbot_msgs/ServoCmd.h>
#include <segbot_msgs/ServoAngle.h>
#include <ros/ros.h>

using namespace std;
using namespace ros;

class ServoNode{
  public:
    ServoNode();
    ~ServoNode();
    int initialize();
    void cmdCallback(const segbot_msgs::ServoCmdConstPtr& msg);
    int updateServo();
  private:
    vector<ros::Subscriber> cmd_sub;
    vector<ros::Publisher> angle_pub;
    ros::NodeHandle nh;

    char* dev;
    int baudRate;
    vector<int> moduleId;

    vector<int> state;
    vector<float> minAngle;  //degrees
    vector<float> maxAngle;  //degrees
    vector<float> vel;   //degrees per second
    vector<float> desAngle;
    vector<int> dir;
    vector<bool> scan;
    float reversePoint; //degrees away from the goal to start reversing

    vector<Dynamixel> dynamixel;
    Upenn::Timer cmdTimer;
    vector<float> position;

};

#endif

