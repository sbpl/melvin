/*
 * command_velocity_filter.cpp
 *
 *  Created on: Mar 9, 2015
 *      Author: bmacallister
 */

#include <command_velocity_filter/command_velocity_filter.h>

command_velocity_filter_node::command_velocity_filter_node() :
		n(), pn("~")
{
	initialize();
}

command_velocity_filter_node::~command_velocity_filter_node()
{
}

void command_velocity_filter_node::initialize()
{
	reset_latest_velocities();
	setup_ros();
}

void command_velocity_filter_node::setup_ros()
{

	//load private params
	pn.getParam("frame_name", frame_name);
	pn.getParam("input_command_vel_topic_name", received_cmd_vel_topic_name);
	pn.getParam("command_vel_topic_name", publish_cmd_vel_topic_name);
	pn.getParam("command_vel_pub_rate", cmd_vel_pub_rate);
	pn.getParam("linear_acc_limit", acc_linear_limit);
	pn.getParam("angular_acc_limit", acc_angular_limit);

	ROS_INFO("subscribed to %s and publish as %s\n", received_cmd_vel_topic_name.c_str(), publish_cmd_vel_topic_name.c_str());
	ROS_INFO("linear acc limit %f  , angular acc limit %f \n", acc_linear_limit, acc_angular_limit);

	//setup subscribers
	cmd_vel_sub = n.subscribe(received_cmd_vel_topic_name, 1, &command_velocity_filter_node::cmd_vel_callback, this);

	//setup timers
	cmd_vel_publish_timer = n.createTimer(ros::Duration(1.0 / cmd_vel_pub_rate), &command_velocity_filter_node::timer_callback, this);

	//setup publishers
	cmd_vel_pub = pn.advertise<geometry_msgs::Twist>(publish_cmd_vel_topic_name, 1);

}

void command_velocity_filter_node::cmd_vel_callback(const geometry_msgs::TwistConstPtr& msg)
{
	set_latest_velocities(*msg);
}

void command_velocity_filter_node::set_latest_velocities(const geometry_msgs::Twist& vel)
{
	latest_recv_velocities_ = vel;
}

void command_velocity_filter_node::reset_latest_velocities()
{
	//Set values of velocities to zero
	latest_recv_velocities_ = geometry_msgs::Twist();
}

void command_velocity_filter_node::timer_callback(const ros::TimerEvent& event)
{
	//Get current time and dt
	ros::Time current_time = ros::Time::now();
	ros::Duration dur = current_time - last_cmd_time_;
	double dt = dur.toSec();

	//Filter velocities
	geometry_msgs::Twist cmd_vel;
	filter_velocities(latest_recv_velocities_, prev_cmd_vel_, cmd_vel, dt);

	//publish filtered velocities
	publish_cmd_velocity(cmd_vel);

	//reset current commanded velocities
	reset_latest_velocities();

	last_cmd_time_ = current_time;
	prev_cmd_vel_ = cmd_vel;
}

void command_velocity_filter_node::filter_velocities(const geometry_msgs::Twist& input_vel, const geometry_msgs::Twist & prev_vel,
		geometry_msgs::Twist& output_vel, double dt)
{
	//get current and past velocities
	double vx = input_vel.linear.x;
	double vy = input_vel.linear.y;
	double vw = input_vel.angular.z;
	double prev_vx = prev_vel.linear.x;
	double prev_vy = prev_vel.linear.y;
	double prev_vw = prev_vel.angular.z;

	ROS_DEBUG("input: vx %f vy %f vw %f", vx,vy,vw);
	ROS_DEBUG("prev: vx %f vy %f vw %f", prev_vx, prev_vw, prev_vw);

	// get differences of velocities
	double vx_diff = vx - prev_vx;
	double vy_diff = vy - prev_vy;
	double vw_diff = vw - prev_vw;

	double v_trans_diff = sqrt(vx_diff * vx_diff + vy_diff * vy_diff);
	double v_rot_diff = vw_diff;
	double vx_diff_rat = vx_diff / v_trans_diff;
	double vy_diff_rat = vy_diff / v_trans_diff;
	double v_rot_diff_sign = get_sign(v_rot_diff);

	// calculate linear and angular accelerations
	double current_a_trans = v_trans_diff / dt;
	double current_a_rot = v_rot_diff / dt;

	// Restrain translational velocity if acceleration above limit
	if (abs(current_a_trans) > acc_linear_limit)
	{
		double v_trans = acc_linear_limit * dt;
		vx = vx_diff_rat * v_trans + prev_vx;
		vy = vy_diff_rat * v_trans + prev_vy;
	}

	// Restrain rotational velocity if acceleration above limit
	if (abs(current_a_rot) > acc_angular_limit)
	{
		double v_rot = acc_angular_limit * dt;
		vw = v_rot_diff_sign * v_rot + prev_vw;
	}

	output_vel.linear.x = vx;
	output_vel.linear.y = vy;
	output_vel.angular.z = vw;

	ROS_DEBUG("output: vx %f vy %f vw %f", vx, vy, vw);

}

void command_velocity_filter_node::publish_cmd_velocity(const geometry_msgs::Twist& vel)
{
	cmd_vel_pub.publish(vel);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "command_velocity_filter_node");
	command_velocity_filter_node cmd_filter;
	ros::spin();
}
