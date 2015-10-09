#include <servo_sim_controller/servo_sim_controller.h>
#include <pluginlib/class_list_macros.h>

using namespace servo_sim_controller_ns;

#define MAX(a,b) (a > b ? a : b)
#define MIN(a,b) (a < b ? a : b)
#define SIGN(v) (v > 0 ? 1 : (v < 0 ? -1 : 0))

/// Controller initialization in non-realtime
bool ServoSimController::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n){
  std::string base_servo_name;
  if (!n.getParam("base_servo_joint", base_servo_name)){
    ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  base_servo_joint = robot->getJointState(base_servo_name);
  if (!base_servo_joint){
    ROS_ERROR("ServoSimController could not find joint named '%s'", base_servo_name.c_str());
    return false;
  }
  if (!n.getParam("base_servo_min_ang", base_min)){
    ROS_ERROR("No min angle given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("base_servo_max_ang", base_max)){
    ROS_ERROR("No max angle given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("base_servo_vel", base_desired_vel)){
    ROS_ERROR("No velocity given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }

  std::string tilt_servo_name;
  if (!n.getParam("tilt_servo_joint", tilt_servo_name)){
    ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  tilt_servo_joint = robot->getJointState(tilt_servo_name);
  if (!tilt_servo_joint){
    ROS_ERROR("ServoSimController could not find joint named '%s'", tilt_servo_name.c_str());
    return false;
  }
  if (!n.getParam("tilt_servo_min_ang", tilt_min)){
    ROS_ERROR("No min angle given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("tilt_servo_max_ang", tilt_max)){
    ROS_ERROR("No max angle given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("tilt_servo_vel", tilt_desired_vel)){
    ROS_ERROR("No velocity given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }

  base_desired = base_min;
  tilt_desired = tilt_min;
  last_base_effort = 0;
  last_tilt_effort = 0;
  last_base_pos = 0;
  last_tilt_pos = 0;
  last_base_vel = 0;
  last_tilt_vel = 0;

  return true;
}


/// Controller startup in realtime
void ServoSimController::starting(){
}


/// Controller update loop in realtime
void ServoSimController::update(){

  ros::Time curr_t = ros::Time::now();
  double dt = curr_t.toSec() - last_t;
  if(dt < 0.01){
    base_servo_joint->commanded_effort_ = last_base_effort;
    tilt_servo_joint->commanded_effort_ = last_tilt_effort;
    return;
  }
  last_t = curr_t.toSec();

  double p_pos = 0.8;
  double d_pos = 0.1;
  double p_vel = 0.0005;
  double d_vel = 0.0001;
  double angle_padding = 5*M_PI/180;
  double max_torque = 2.97;

  //base servo
  double base_pos = base_servo_joint->position_;
  double base_vel = base_servo_joint->velocity_;
  if(base_max==base_min){
    double base_effort = MAX(MIN((base_desired-base_pos)*p_pos + (base_pos-last_base_pos)/dt*d_pos, max_torque), -max_torque);
    base_servo_joint->commanded_effort_ = base_effort;
    //printf("base desired=%f base_actual=%f base_effort=%f\n",base_desired,base_pos,base_effort);
  }
  else{
    if(base_pos >= base_max-angle_padding)
      base_desired = base_min;
    if(base_pos <= base_min+angle_padding)
      base_desired = base_max;
    double base_velocity = base_desired_vel*SIGN(base_desired-base_pos);
    double base_effort = MAX(MIN(last_base_effort + (base_velocity-base_vel)*p_vel + (base_vel-last_base_vel)/dt*d_vel, max_torque), -max_torque);
    base_servo_joint->commanded_effort_ = base_effort;
    last_base_effort = base_effort;
  }
  
  //tilt servo
  double tilt_pos = tilt_servo_joint->position_;
  double tilt_vel = tilt_servo_joint->velocity_;
  if(tilt_max==tilt_min){
    double tilt_effort = MAX(MIN((tilt_desired-tilt_pos)*p_pos + (tilt_pos-last_tilt_pos)/dt*d_pos, max_torque), -max_torque);
    tilt_servo_joint->commanded_effort_ = tilt_effort;
  }
  else{
    if(tilt_pos >= tilt_max-angle_padding)
      tilt_desired = tilt_min;
    if(tilt_pos <= tilt_min+angle_padding)
      tilt_desired = tilt_max;
    double tilt_velocity = tilt_desired_vel*SIGN(tilt_desired-tilt_pos);
    double tilt_effort = MAX(MIN(last_tilt_effort + (tilt_velocity-tilt_vel)*p_vel + (tilt_vel-last_tilt_vel)/dt*d_vel, max_torque), -max_torque);
    tilt_servo_joint->commanded_effort_ = tilt_effort;
    last_tilt_effort = tilt_effort;
    //printf("tilt velocity=%f tilt effort=%f\n",tilt_velocity,tilt_effort);
  }

  last_base_pos = base_pos;
  last_tilt_pos = tilt_pos;
  last_base_vel = base_vel;
  last_tilt_vel = tilt_vel;
}


/// Controller stopping in realtime
void ServoSimController::stopping(){
}

/// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS(ServoSimControllerPlugin, 
                         servo_sim_controller_ns::ServoSimController, 
                         pr2_controller_interface::Controller)

