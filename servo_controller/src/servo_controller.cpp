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
  baseServoAngle_sub = nh_.subscribe<segbot_msgs::ServoAngle>("/servoAngle1", 1, &ServoController::cmdCallback, this);
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
  tiltServoAngle_sub = nh_.subscribe<segbot_msgs::ServoAngle>("/servoAngle2", 1, &ServoController::cmdCallback, this);
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

// Callback for joystick subscription
void ServoController::cmdCallback(const segbot_msgs::ServoAngleConstPtr& msg){
  if(msg->id == 0)
    baseAngle = msg->angle;
  else
    tiltAngle = msg->angle;
}


/// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS(ServoControllerPlugin, 
                         servo_controller_ns::ServoController, 
                         pr2_controller_interface::Controller)

