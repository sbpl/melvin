#include <servo_controller/servo_controller.h>
#include <pluginlib/class_list_macros.h>

using namespace servo_controller_ns;

/// Controller initialization in non-realtime
bool ServoController::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n){
  std::string base_servo_name;
  if (!n.getParam("base_servo_joint", base_servo_name)){
    ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  base_servo_joint = robot->getJointState(base_servo_name);
  if (!base_servo_joint){
    ROS_ERROR("ServoController could not find joint named '%s'", base_servo_name.c_str());
    return false;
  }
  std::string baseServoAngleName;
  n.getParam("base_servo_angle_name", baseServoAngleName);
  baseServoAngle_sub = nh_.subscribe<segbot_msgs::ServoAngle>(baseServoAngleName, 1, &ServoController::baseCmdCallback, this);
  baseAngle = 0;

  std::string tilt_servo_name;
  if (!n.getParam("tilt_servo_joint", tilt_servo_name)){
    ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  tilt_servo_joint = robot->getJointState(tilt_servo_name);
  if (!tilt_servo_joint){
    ROS_ERROR("ServoController could not find joint named '%s'", tilt_servo_name.c_str());
    return false;
  }
  std::string tiltServoAngleName;
  n.getParam("tilt_servo_angle_name", tiltServoAngleName);
  tiltServoAngle_sub = nh_.subscribe<segbot_msgs::ServoAngle>(tiltServoAngleName.c_str(), 1, &ServoController::tiltCmdCallback, this);
  tiltAngle = 0;
  return true;
}


/// Controller startup in realtime
void ServoController::starting(){
}


/// Controller update loop in realtime
void ServoController::update(){
  //printf("update!\n");
  base_servo_joint->position_ = -baseAngle;
  tilt_servo_joint->position_ = tiltAngle+M_PI/180*4;
}


/// Controller stopping in realtime
void ServoController::stopping(){
}

void ServoController::baseCmdCallback(const segbot_msgs::ServoAngleConstPtr& msg)
{
  baseAngle = msg->angle;
}

void ServoController::tiltCmdCallback(const segbot_msgs::ServoAngleConstPtr& msg)
{
  tiltAngle = msg->angle;
}

/// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS(ServoControllerPlugin, 
                         servo_controller_ns::ServoController, 
                         pr2_controller_interface::Controller)

