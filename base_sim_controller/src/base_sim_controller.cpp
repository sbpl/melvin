#include <base_sim_controller/base_sim_controller.h>
#include <pluginlib/class_list_macros.h>

using namespace base_sim_controller_ns;

#define MAX(a,b) (a > b ? a : b)
#define MIN(a,b) (a < b ? a : b)

double pose_cov[36] = {1,0,0,0,0,0,
                       0,1,0,0,0,0,
                       0,0,1,0,0,0,
                       0,0,0,1,0,0,
                       0,0,0,0,1,0,
                       0,0,0,0,0,1};
double twist_cov[36] = {1,0,0,0,0,0,
                        0,1,0,0,0,0,
                        0,0,1,0,0,0,
                        0,0,0,1,0,0,
                        0,0,0,0,1,0,
                        0,0,0,0,0,1};

/// Controller initialization in non-realtime
bool BaseSimController::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n){
  std::string left_name, right_name;
  if (!n.getParam("left_wheel_joint", left_name)){
    ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }

  left_wheel_joint = robot->getJointState(left_name);
  if (!left_wheel_joint){
    ROS_ERROR("BaseSimController could not find joint named '%s'", left_name.c_str());
    return false;
  }

  if (!n.getParam("right_wheel_joint", right_name)){
    ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }

  right_wheel_joint = robot->getJointState(right_name);
  if (!right_wheel_joint){
    ROS_ERROR("BaseSimController could not find joint named '%s'", right_name.c_str());
    return false;
  }

  cmd_sub = nh_.subscribe<geometry_msgs::Twist>("base_controller/command", 1, &BaseSimController::cmdCallback, this);
  linearVel = 0;
  angularVel = 0;

  error_sum_right = 0;
  error_sum_left = 0;
  last_t = ros::Time::now().toSec();
  last_error_right = 0;
  last_error_left = 0;
  last_left_torque = 0;
  last_right_torque = 0;

  odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);
  x = 0.0;
  y = 0.0;
  theta = 0.0;

  return true;
}


/// Controller startup in realtime
void BaseSimController::starting(){
}


/// Controller update loop in realtime
void BaseSimController::update(){
  double kp = 100;
  double kd = 0;
  double ki = 0;
  double ki_clamp = 0;
  double max_torque = 122;

  double wheel_radius = 0.24;
  double robot_width = 0.53;

  ros::Time curr_t = ros::Time::now();
  double dt = curr_t.toSec() - last_t;

  if(dt < 0.01){
    right_wheel_joint->commanded_effort_ = -last_right_torque;
    left_wheel_joint->commanded_effort_ = last_left_torque;
    return;
  }

  //double desired_ang_vel_right = (2.0 * linearVel + angularVel * wheel_radius) / (2.0 * robot_width);
  double desired_ang_vel_right = 2.0*(2.0 * linearVel + 4.0*angularVel * wheel_radius) / (2.0 * robot_width);
  double actual_ang_vel_right = -right_wheel_joint->velocity_;
  double error_right = desired_ang_vel_right - actual_ang_vel_right;
  double d_error_right = (error_right - last_error_right)/dt;
  error_sum_right = MAX(MIN(error_sum_right + error_right*dt, ki_clamp), -ki_clamp);
  double right_torque = MAX(MIN(error_right*kp + d_error_right*kd + error_sum_right*ki, max_torque), -max_torque);
  right_wheel_joint->commanded_effort_ = -right_torque;

  //double desired_ang_vel_left = (2.0 * linearVel - angularVel * wheel_radius) / (2.0 * robot_width);
  double desired_ang_vel_left = 2.0*(2.0 * linearVel - 4.0*angularVel * wheel_radius) / (2.0 * robot_width);
  double actual_ang_vel_left = left_wheel_joint->velocity_;
  double error_left = desired_ang_vel_left - actual_ang_vel_left;
  double d_error_left = (error_left - last_error_left)/dt;
  error_sum_left = MAX(MIN(error_sum_left + error_left*dt, ki_clamp), -ki_clamp);
  double left_torque = MAX(MIN(error_left*kp + d_error_left*kd + error_sum_left*ki, max_torque), -max_torque);
  left_wheel_joint->commanded_effort_ = left_torque;

  /*
  //printf("dt = %f\n", dt);
  printf("v=%f w=%f\n",linearVel,angularVel);
  printf("right desired=%f actual=%f error=%f\n",desired_ang_vel_right,actual_ang_vel_right,error_right);
  printf("left desired=%f actual=%f error=%f\n",desired_ang_vel_left,actual_ang_vel_left,error_left);
  printf("right_torque=%f left_torque=%f\n",right_torque,left_torque);
  */

  last_t = curr_t.toSec();
  last_error_right = error_right;
  last_error_left = error_left;
  last_right_torque = right_torque;
  last_left_torque = left_torque;


  //compute odometry
  double d_dist = (actual_ang_vel_right + actual_ang_vel_left)*wheel_radius/2.0;
  double d_ang = (actual_ang_vel_right - actual_ang_vel_left)*wheel_radius/robot_width;
  double vx = d_dist*cos(theta);
  double vy = d_dist*sin(theta);
  double vtheta = d_ang;
  x += vx*dt;
  y += vy*dt;
  theta += vtheta*dt;
  //printf("actual v=%f w=%f\n",d_dist,d_ang);

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

  
  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = curr_t;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);
  

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = curr_t;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "segway_base";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vtheta;

  for(int i=0; i<36; i++){
    odom.pose.covariance[i] = pose_cov[i];
    odom.twist.covariance[i] = twist_cov[i];
  }

  //publish the message
  odom_pub.publish(odom);
}


/// Controller stopping in realtime
void BaseSimController::stopping(){
}

// Callback for joystick subscription
void BaseSimController::cmdCallback(const geometry_msgs::TwistConstPtr& twist){
  linearVel = twist->linear.x;
  angularVel = twist->angular.z;
}


/// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS(BaseSimControllerPlugin, 
                         base_sim_controller_ns::BaseSimController, 
                         pr2_controller_interface::Controller)

