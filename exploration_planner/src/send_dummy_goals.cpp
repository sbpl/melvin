#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <stdio.h>
#include <tf/tf.h>
#include <actionlib_msgs/GoalStatusArray.h>

using namespace std;
using namespace ros;
using namespace nav_msgs;

Publisher pub;
double last_pub_time = 0;

void sendGoal(){
  geometry_msgs::PoseStamped goal;

  goal.pose.position.x = 0;
  goal.pose.position.y = 0;
  goal.pose.position.z = 0;
  tf::Quaternion temp;
  temp.setEulerZYX(0,0,0);
  goal.pose.orientation.x = temp.getX();
  goal.pose.orientation.y = temp.getY();
  goal.pose.orientation.z = temp.getZ();
  goal.pose.orientation.w = temp.getW();
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "/map";
  pub.publish(goal);
}

void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArrayConstPtr& msg){
  //printf("size=%d\n",msg->status_list.size());
  if(!msg->status_list.empty() && msg->status_list.back().status == msg->status_list.back().SUCCEEDED){
    //printf("we are at the goal!\n");
    sendGoal();
    printf("\nsent dummy goal (finished the last one)\n");
    last_pub_time = ros::Time::now().toSec();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "send_dummy_goals");

  pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("/goal", 1);
  Subscriber moveBaseStatus_sub = ros::NodeHandle().subscribe("move_base/status", 1, &moveBaseStatusCallback);

  //ros::Rate loop_rate(1.0/15);
  while (ros::ok()){
    double curr_time = ros::Time::now().toSec();
    if(curr_time-last_pub_time > 10.0){
      sendGoal();
      printf("\nsent dummy goal (on timeout)\n");
      last_pub_time = curr_time;
    }
    else
      ros::Duration(curr_time-last_pub_time).sleep();
    ros::spinOnce();
    //loop_rate.sleep();
  }
}

