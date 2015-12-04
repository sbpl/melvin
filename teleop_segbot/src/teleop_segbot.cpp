/*
 * teleop_segbot
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

///\author Bradford Neuman
///\brief Converts joystick commands on /joy to commands to segbot

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "topic_tools/MuxSelect.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"

#include "actionlib_msgs/GoalID.h"

const int PUBLISH_FREQ = 20;

using namespace std;

class TeleopSegbot
{
   public:
  geometry_msgs::Twist cmd;
  double min_torso, max_torso;
  double req_torso_vel, torso_step;
  //joy::Joy joy;
  double req_vx, req_vy, req_vw, req_torso, req_pan, req_tilt;
  double req_tilt_vel, req_pan_vel;
  double max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run;
  double max_pan, max_tilt, min_tilt, pan_step, tilt_step;
  int axis_vx, axis_vy, axis_vw, axis_pan, axis_tilt;
  int deadman_button, run_button, send_goal_button, cancel_goal_button;
  bool deadman_no_publish_, torso_publish_, head_publish_;
  
  double prev_vx;
  double prev_vy;
  double prev_vw;
  double acc_trans_limit;
  double acc_rot_limit;
  double acc_timeout_limit;

  bool deadman_, cmd_head;
  bool use_mux_, last_deadman_;
  std::string last_selected_topic_;

  bool cancel_goal_, send_goal_;
  geometry_msgs::PoseStamped next_goal_;
  bool has_next_goal;

  ros::Time last_recieved_joy_message_time_;
  ros::Time last_recieved_deadman_time_;
  ros::Duration joy_msg_timeout_;
  ros::Duration joy_deadman_dt_;

  ros::NodeHandle n_, n_private_;
  ros::Publisher vel_pub_;
  ros::Publisher true_goal_pub_;
  ros::Publisher goal_cancel_pub_;
  ros::Publisher dpad_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber next_goal_sub_;
  ros::ServiceClient mux_client_;

  TeleopSegbot(bool deadman_no_publish = false) :
    max_vx(0.6), max_vy(0.6), max_vw(0.8),
    max_vx_run(0.6), max_vy_run(0.6), max_vw_run(0.8),
    max_pan(2.7), max_tilt(1.4), min_tilt(-0.4),
    pan_step(0.02), tilt_step(0.015),
    //deadman_no_publish_(deadman_no_publish), 
    deadman_no_publish_(true),  //HACK!!! TODO: fix
    acc_trans_limit(100.0), acc_rot_limit(100.0), acc_timeout_limit(0.25),
    deadman_(false), cmd_head(false), 
    use_mux_(false), last_deadman_(false),
    cancel_goal_(false), send_goal_(false),
    has_next_goal(false), n_private_("~")
  { }

  void init()
  {
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    req_pan = req_tilt = 0;
    req_torso = 0.0;
    req_torso_vel = 0.0;

    //parameters for interaction with a mux on cmd_vel topics
    n_private_.param("use_mux", use_mux_, false);

    n_private_.param("max_vx", max_vx, max_vx);
    n_private_.param("max_vy", max_vy, max_vy);
    n_private_.param("max_vw", max_vw, max_vw);

    // Set max speed while running
    n_private_.param("max_vx_run", max_vx_run, max_vx_run);
    n_private_.param("max_vy_run", max_vy_run, max_vy_run);
    n_private_.param("max_vw_run", max_vw_run, max_vw_run);

    n_private_.param("axis_vx", axis_vx, 3);
    n_private_.param("axis_vw", axis_vw, 0);
    n_private_.param("axis_vy", axis_vy, 2);

    n_private_.param("deadman_button", deadman_button, 0);
    n_private_.param("run_button", run_button, 0);
    n_private_.param("send_goal_button", send_goal_button, 0);
    n_private_.param("cancel_goal_button", cancel_goal_button, 0);
    
    n_private_.param("acc_trans_limit", acc_trans_limit, acc_trans_limit);
    n_private_.param("acc_rot_limit", acc_rot_limit, acc_rot_limit);
    n_private_.param("acc_timeout_limit", acc_timeout_limit, 0.25);

    double joy_msg_timeout;
    n_private_.param("joy_msg_timeout", joy_msg_timeout, 0.5); //default to 0.5 seconds timeout
    if (joy_msg_timeout <= 0)
      {
	joy_msg_timeout_ = ros::Duration().fromSec(9999999);//DURATION_MAX;
	ROS_DEBUG("joy_msg_timeout <= 0 -> no timeout");
      }
    else
      {
	joy_msg_timeout_.fromSec(joy_msg_timeout);
	ROS_DEBUG("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
      }

    ROS_DEBUG("max_vx: %.3f m/s\n", max_vx);
    ROS_DEBUG("max_vy: %.3f m/s\n", max_vy);
    ROS_DEBUG("max_vw: %.3f deg/s\n", max_vw*180.0/M_PI);

    ROS_DEBUG("max_vx_run: %.3f m/s\n", max_vx_run);
    ROS_DEBUG("max_vy_run: %.3f m/s\n", max_vy_run);
    ROS_DEBUG("max_vw_run: %.3f deg/s\n", max_vw_run*180.0/M_PI);

    ROS_DEBUG("axis_vx: %d\n", axis_vx);
    ROS_DEBUG("axis_vy: %d\n", axis_vy);
    ROS_DEBUG("axis_vw: %d\n", axis_vw);

    ROS_DEBUG("deadman_button: %d\n", deadman_button);
    ROS_DEBUG("run_button: %d\n", run_button);
    ROS_DEBUG("send_goal_button: %d\n", send_goal_button);
    ROS_DEBUG("cancel_goal_button: %d\n", cancel_goal_button);

    ROS_DEBUG("joy_msg_timeout: %f\n", joy_msg_timeout);

    vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    true_goal_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/goal", 1);
    goal_cancel_pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    dpad_pub_ = n_.advertise<std_msgs::Int16>("dpad", 10);

    joy_sub_ = n_.subscribe("joy", 10, &TeleopSegbot::joy_cb, this);

    next_goal_sub_ = n_.subscribe("next_goal", 10, &TeleopSegbot::next_goal_cb, this);

    //if we're going to use the mux, then we'll subscribe to state changes on the mux
    if(use_mux_){
      ros::NodeHandle mux_nh("mux");
      mux_client_ = mux_nh.serviceClient<topic_tools::MuxSelect>("select");
    }
    
    //set previous velocities to 0
    prev_vx = 0;
    prev_vy = 0;
    prev_vw = 0;
    
    ROS_DEBUG("done controller init!");
  }

  ~TeleopSegbot() { }

  /** Callback for next_goal. Retreives a goal and then sends it to /goal when the send_goal_button is pressed **/
  void next_goal_cb(const geometry_msgs::PoseStamped& next_goal) {
    next_goal_ = next_goal;
    has_next_goal = true;
    ROS_INFO("Got a next_goal");
  }

  /** Callback for joy topic **/
  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {

    deadman_ = (((unsigned int)deadman_button < joy_msg->buttons.size()) && joy_msg->buttons[deadman_button]);
    
    //Record time between deadman presses
    if(deadman_)
    {
      joy_deadman_dt_ = ros::Time::now() -  last_recieved_deadman_time_;
      last_recieved_deadman_time_= ros::Time::now();
    }
    
    //Record this message reciept
    last_recieved_joy_message_time_ = ros::Time::now();

    // send or cancel goal even without deadman switch
    // check for next goal
    //if( has_next_goal && (((unsigned int)send_goal_button < joy_msg->buttons.size()) && joy_msg->buttons[send_goal_button]) ) {
    //  ROS_INFO("sending stored goal!");
    //  //has_next_goal = false;
    //  true_goal_pub_.publish(next_goal_);
    //}

    //if((((unsigned int)cancel_goal_button < joy_msg->buttons.size()) && joy_msg->buttons[cancel_goal_button]) ) {
    //  ROS_INFO("canceling goal!");
    //  actionlib_msgs::GoalID gid;
    //  gid.stamp = ros::Time::now();
    //  //gid.id = "";
    //  goal_cancel_pub_.publish(gid);
    //}

    // dpad: urdl = 4 5 6 7
    if((((unsigned int)4 < joy_msg->buttons.size()) && joy_msg->buttons[4]) ) {
      std_msgs::Int16 m;
      m.data = 0;
      dpad_pub_.publish(m);
    }
    if((((unsigned int)5 < joy_msg->buttons.size()) && joy_msg->buttons[5]) ) {
      std_msgs::Int16 m;
      m.data = 1;
      dpad_pub_.publish(m);
    }
    if((((unsigned int)6 < joy_msg->buttons.size()) && joy_msg->buttons[6]) ) {
      std_msgs::Int16 m;
      m.data = 2;
      dpad_pub_.publish(m);
    }
    if((((unsigned int)7 < joy_msg->buttons.size()) && joy_msg->buttons[7]) ) {
      std_msgs::Int16 m;
      m.data = 3;
      dpad_pub_.publish(m);
    }

    if (!deadman_)
      return;
    
    // send or cancel goal only with deadman switch
    // check for next goal
    if( has_next_goal && (((unsigned int)send_goal_button < joy_msg->buttons.size()) && joy_msg->buttons[send_goal_button]) ) {
      ROS_INFO("sending stored goal!");
      //has_next_goal = false;
      true_goal_pub_.publish(next_goal_);
    }

    if((((unsigned int)cancel_goal_button < joy_msg->buttons.size()) && joy_msg->buttons[cancel_goal_button]) ) {
      ROS_INFO("canceling goal!");
      actionlib_msgs::GoalID gid;
      gid.stamp = ros::Time::now();
      //gid.id = "";
      goal_cancel_pub_.publish(gid);
    }

    // Base
    bool running = (((unsigned int)run_button < joy_msg->buttons.size()) && joy_msg->buttons[run_button]);
    double vx = running ? max_vx_run : max_vx;
    double vy = running ? max_vy_run : max_vy;
    double vw = running ? max_vw_run : max_vw;

    if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy_msg->axes.size()) && !cmd_head)
      req_vx = joy_msg->axes[axis_vx] * vx;
    else
      req_vx = 0.0;
    if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy_msg->axes.size()) && !cmd_head)
      req_vy = joy_msg->axes[axis_vy] * vy;
    else
      req_vy = 0.0;
    if((axis_vw >= 0) && (((unsigned int)axis_vw) < joy_msg->axes.size()) && !cmd_head)
      req_vw = joy_msg->axes[axis_vw] * vw;
    else
      req_vw = 0.0;

    // Enforce max/mins for velocity
    // Joystick should be [-1, 1], but it might not be
    req_vx = max(min(req_vx, vx), -vx);
    req_vy = max(min(req_vy, vy), -vy);
    req_vw = max(min(req_vw, vw), -vw);

    //reset prev commanded velocities if timeout exceeded
    if(joy_deadman_dt_.toSec() >= acc_timeout_limit)
    {
      prev_vx = 0;
      prev_vy = 0;
      prev_vw = 0;
      return;
    }
    
    //Restrain velocities with respect to acceleration limits
    restrain_velocities_from_acceleration_limits(req_vx,req_vy,req_vw);

    //save previously commanded velocities
    prev_vx = req_vx;
    prev_vy = req_vy;
    prev_vw = req_vw;

  }


  void send_cmd_vel()
  {
    if(deadman_  &&
       last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
    {
      //check if we need to switch the mux to our topic for teleop
      if(use_mux_ && !last_deadman_){
        topic_tools::MuxSelect select_srv;
        select_srv.request.topic = vel_pub_.getTopic();
        if(mux_client_.call(select_srv)){
          last_selected_topic_ = select_srv.response.prev_topic;
          ROS_DEBUG("Setting mux to %s for teleop", select_srv.request.topic.c_str());
        }
        else{
          ROS_ERROR("Failed to call select service %s on mux. Are you sure that it is up and connected correctly to the teleop node?", mux_client_.getService().c_str());
        }
      }

      // Base
      cmd.linear.x = req_vx;
      cmd.linear.y = req_vy;
      cmd.angular.z = req_vw;
      ROS_DEBUG("base: %f %f %f", req_vx, req_vy, req_vw);
      vel_pub_.publish(cmd);

      fprintf(stdout,"teleop_base:: %f, %f, %f. Head:: %f, %f\n",
	      cmd.linear.x ,cmd.linear.y, cmd.angular.z, req_pan, req_tilt);
    }
    else
    {
      //make sure to set the mux back to whatever topic it was on when we grabbed it if the deadman has just toggled
      if(use_mux_ && last_deadman_){
        topic_tools::MuxSelect select_srv;
        select_srv.request.topic = last_selected_topic_;
        if(mux_client_.call(select_srv)){
          ROS_DEBUG("Setting mux back to %s", last_selected_topic_.c_str());
        }
        else{
          ROS_ERROR("Failed to call select service %s on mux. Are you sure that it is up and connected correctly to the teleop node?", mux_client_.getService().c_str());
        }
      }

      // Publish zero commands iff deadman_no_publish is false
      cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
      if (!deadman_no_publish_)
      {
        // Base
        vel_pub_.publish(cmd);
      }
    }

    //make sure we store the state of our last deadman
    last_deadman_ = deadman_;
  }
  
  void restrain_velocities_from_acceleration_limits(double & vx, double & vy, double & vw)
  {  
    // get differences of velocites and time
    double vx_diff = vx - prev_vx;
    double vy_diff = vy - prev_vy;
    double vw_diff = vw - prev_vw;

    double t_diff = joy_deadman_dt_.toSec();
    double v_trans_diff = sqrt(vx_diff*vx_diff + vy_diff*vy_diff);
    double v_rot_diff = vw_diff;
    double vx_diff_rat = vx_diff / v_trans_diff;
    double vy_diff_rat = vy_diff / v_trans_diff;
    double v_rot_diff_sign = get_sign(v_rot_diff);
 
    // calculate linear and angular accelerations
    double current_a_trans = v_trans_diff / t_diff;
    double current_a_rot = v_rot_diff / t_diff;
    
    // Restrain translational velocity if acceleration above limit
    if(abs(current_a_trans) > acc_trans_limit)
    {
	double v_trans = acc_trans_limit * t_diff;
	vx = vx_diff_rat * v_trans + prev_vx;
	vy = vy_diff_rat * v_trans + prev_vy;
    }
   
    // Restrain rotational velocity if acceleration above limit
    if(abs(current_a_rot) > acc_rot_limit)
    {
	double v_rot = acc_rot_limit * t_diff;
	vw = v_rot_diff_sign * v_rot + prev_vw;
    }    
  }
  
  template<typename T>
  T get_sign(T value)
  {
      return (value < 0) ? -1 : 1;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_segbot");
  const char* opt_no_publish    = "--deadman_no_publish";

  bool no_publish = false;
  for(int i=1;i<argc;i++)
  {
    if(!strncmp(argv[i], opt_no_publish, strlen(opt_no_publish)))
      no_publish = true;
  }

  TeleopSegbot teleop_segbot(no_publish);
  teleop_segbot.init();

  ros::Rate pub_rate(PUBLISH_FREQ);

  while (teleop_segbot.n_.ok())
  {
    ros::spinOnce();
    teleop_segbot.send_cmd_vel();
    pub_rate.sleep();
  }

  exit(0);
  return 0;
}

