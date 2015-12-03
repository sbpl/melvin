#include <servo_node/Dynamixel.h>
#include <iostream>
#include <servo_node/Timer.h>
#include <servo_node/ErrorMessage.h>
#include <servo_node/servo_node.h>
#include <ros/ros.h>

#define PI 3.14159

//states for the built-in mini-state machine
enum { DYNAMIXEL_STATE_UNINITIALIZED, 
       DYNAMIXEL_STATE_INITIALIZED,
       DYNAMIXEL_STATE_MOVE_CMD_SENT,    
       DYNAMIXEL_STATE_STOPPED
     };

ServoNode::ServoNode() : nh("~"){

// dev = (char*)"/dev/ttyUSB0";
//  baudRate = 57600;

  std::string device_name;
  nh.getParam("device_name", dev);
  nh.getParam("baud_rate", baudRate);
  ROS_INFO("connecting to device %s with baud rate %d\n", dev.c_str(), baudRate); 

  std::vector<double> temp;

  //Get servo IDs
  nh.getParam("servoID", temp);
  if(temp.size() == 0){
    moduleId.push_back(1);
    moduleId.push_back(2);
  }
  else{
    for(int i=0; i<temp.size(); i++) {
      moduleId.push_back(temp[i]);
    }
  }  

  //Get min angle for each servo
  nh.getParam("minAngle", temp);
  if(temp.size() == 0){
    //for(unsigned int i=0; i<moduleId.size(); i++)
      minAngle.push_back(0.0);
      minAngle.push_back(-10.0);//was -30
  }
  else if(temp.size() != (int)moduleId.size()){
    ROS_ERROR("Number of minAngle parameters does not match number of servo IDs!");
    exit(1);
  }
  else{
    for(int i=0; i<temp.size(); i++) {
      minAngle.push_back(static_cast<double>(temp[i]));
    }
  }

  //Get max angle for each servo
  nh.getParam("maxAngle", temp);
  //ROS_ASSERT(temp.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if(temp.size() == 0){
    //for(unsigned int i=0; i<moduleId.size(); i++)
      maxAngle.push_back(0.0);
      maxAngle.push_back(10.0);//was 0
  }
  else if(temp.size() != (int)moduleId.size()){
    ROS_ERROR("Number of maxAngle parameters does not match number of servo IDs!");
    exit(1);
  }
  else{
    for(int i=0; i<temp.size(); i++) {
      maxAngle.push_back(static_cast<double>(temp[i]));
    }
  }

  //Get velocity for each servo
  nh.getParam("velocity", temp);
  if(temp.size() == 0){
    for(unsigned int i=0; i<moduleId.size(); i++)
      vel.push_back(90.0);
  }
  else if(temp.size() != (int)moduleId.size()){
    ROS_ERROR("Number of velocity parameters does not match number of servo IDs!");
    exit(1);
  }
  else{
    for(int i=0; i<temp.size(); i++) {
      vel.push_back(static_cast<double>(temp[i]));
    }
  }

  ROS_INFO("Loaded parameters...");

  for(size_t i = 0; i < moduleId.size(); i++)
  {
    ROS_INFO("%zu: id %d min angle %f max angle %f velocity %f\n", i,moduleId[i], minAngle[i], maxAngle[i], vel[i]); 
  }

  reversePoint = 5; //degrees away from the goal to start reversing
  for(unsigned int i=0; i<moduleId.size(); i++){
    desAngle.push_back(0);
    dir.push_back(1);
    scan.push_back(minAngle != maxAngle);
    state.push_back(DYNAMIXEL_STATE_INITIALIZED);
    position.push_back(0);
    Dynamixel d;
    dynamixel.push_back(d);

    char topic1[16];
    char topic2[16];
    sprintf(topic1, "/servoCmd%d",moduleId[i]);
    sprintf(topic2, "/servoAngle%d",moduleId[i]);
    cmd_sub.push_back(nh.subscribe<segbot_msgs::ServoCmd>(topic1, 1, &ServoNode::cmdCallback, this));
    angle_pub.push_back(nh.advertise<segbot_msgs::ServoAngle>(topic2, 3));
  }
  
}

int ServoNode::initialize(){
  ROS_INFO("Begin init...");
  for(unsigned int i=0; i<moduleId.size(); i++){
    //open the serial port
    if (dynamixel[i].Connect(dev.c_str(),baudRate,moduleId[i])){
      ROS_ERROR("servo_node: module %d could not connect\n",i);
      return -1;
    }
    
    //check to see if the servo is hooked up
    if (dynamixel[i].StartDevice()){
      ROS_ERROR("servo_node: module %d could not get status\n",i);
      return -1;
    }

    ROS_INFO("servo_node: module %d connected\n",i);

    if (dynamixel[i].MoveToPos(0,50)){
      ROS_ERROR("servo_node: module %d could not send move cmd\n",i);
      return -1;
    }
    ROS_INFO("servo_node: module %d sent move cmd\n",i);
  }
  ROS_INFO("Init complete!");
  return 0;
}


ServoNode::~ServoNode(){}

void ServoNode::cmdCallback(const segbot_msgs::ServoCmdConstPtr& msg){
  int i = msg->id;

  minAngle[i] = msg->minAngle*180/PI;
  maxAngle[i] = msg->maxAngle*180/PI;
  vel[i] = msg->velocity*180/PI;

  scan[i] = minAngle != maxAngle;

  state[i] = DYNAMIXEL_STATE_INITIALIZED;
}

int ServoNode::updateServo(){
  for(unsigned int i=0; i<moduleId.size(); i++){
    if(scan[i]){
      //servo back and forth
      switch (state[i]){

        case DYNAMIXEL_STATE_INITIALIZED:

        case DYNAMIXEL_STATE_STOPPED:
ROS_INFO("stopped");
        
          desAngle[i] = dir[i] > 0 ? maxAngle[i] : minAngle[i];
        
          if (dynamixel[i].MoveToPos(desAngle[i],vel[i])){
            ROS_ERROR("servo_node: module %d could not send MoveToPos command\n",i);
            return -1;
          }
        
          state[i] = DYNAMIXEL_STATE_MOVE_CMD_SENT;
          cmdTimer.Tic();
          
          //fall through here to get the position

        case DYNAMIXEL_STATE_MOVE_CMD_SENT:
ROS_INFO("sent");

          if (dynamixel[i].GetPosition(position[i]) == 0){
            ROS_INFO("module %d: angle = %f\n",i,position[i]);
            //if we are close to getting to the desired angle, switch the direction without getting there
            if ((dir[i]>0) && (position[i] > ( desAngle[i] - reversePoint))){
              state[i] = DYNAMIXEL_STATE_STOPPED;
              dir[i]*=-1;
            }

            else if ((dir[i]<0) && (position[i] < (desAngle[i] + reversePoint))){
              state[i] = DYNAMIXEL_STATE_STOPPED;
              dir[i]*=-1;
            }
            break;
          }
          else{
            ROS_ERROR( "servo_node: module %d could not get position from the servo\n",i);
            return -1;
          }

        default:
          ROS_ERROR("servo_node: module %d unknown state\n",i);
          return -1;
      }
    }
    else{
      switch(state[i]){
        case DYNAMIXEL_STATE_INITIALIZED:
          if (dynamixel[i].MoveToPos(maxAngle[i],vel[i])){
            ROS_ERROR("servo_node: module %d could not send MoveToPos command\n",i);
            return -1;
          }
          state[i] = DYNAMIXEL_STATE_MOVE_CMD_SENT;
          cmdTimer.Tic();
          //fall through here to get the position

        case DYNAMIXEL_STATE_MOVE_CMD_SENT:
          if (dynamixel[i].GetPosition(position[i]) == 0){
            ROS_INFO("module %d: angle = %f\n",i,position[i]);
            break;
          }
          else{
            ROS_ERROR( "servo_node: module %d could not get position from the servo\n",i);
            return -1;
          }

        default:
          ROS_ERROR("servo_node: module %d unknown state\n",i);
          return -1;
      }
    }

    segbot_msgs::ServoAngle msg;
    msg.header.stamp = ros::Time::now();
    msg.angle = position[i]*PI/180;
    msg.id = i;
    angle_pub[i].publish(msg);
  }

  return 0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "servo_node");
  ServoNode s;
  if(s.initialize())
    return 1;

  //set up periodic call
  while(ros::ok()){
    if(s.updateServo())
      return 1;
    //ros::spinOnce();
  }

  return 0;
}

