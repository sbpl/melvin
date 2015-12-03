#include <ipc.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <segbot_ipc/DpiDataTypes.hh>

int main(int argc, char** argv){
  ros::init(argc, argv, "segbot_ipc");

  //init ipc
  IPC_connect("Segbot");
  VehicleState pose_msg;
  IPC_defineMsg("segbot/pose", IPC_VARIABLE_LENGTH, pose_msg.getIPCFormat());

  ros::NodeHandle node;
  tf::TransformListener listener;
  geometry_msgs::TransformStamped geo_pose;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    transformStampedTFToMsg(transform, geo_pose);

    pose_msg.x = geo_pose.transform.translation.x;
    pose_msg.y = geo_pose.transform.translation.y;
    pose_msg.z = 1;
    pose_msg.yaw = tf::getYaw(geo_pose.transform.rotation);

    //printf("\n\ntf %f %f %f\n\n", pose_msg.x,pose_msg.y,pose_msg.yaw);

    IPC_publishData("segbot/pose", &pose_msg); 

    rate.sleep();
  }
  return 0;
}

