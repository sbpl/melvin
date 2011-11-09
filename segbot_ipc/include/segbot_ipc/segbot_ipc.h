#ifndef SEGBOT_IPC_H
#define SEGBOT_IPC_H

#include <segbot_ipc/DpiDataTypes.hh>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class SegbotIPC{
  public:
    SegbotIPC();
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
  private:
    VehicleState pose_msg;
    ros::Subscriber pose_sub;
};

#endif
