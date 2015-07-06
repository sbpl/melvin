/*
 * command_velocity_filter.h
 *
 *  Created on: Mar 9, 2015
 *      Author: bmacallister
 */

#ifndef COMMAND_VELOCITY_FILTER_H_
#define COMMAND_VELOCITY_FILTER_H_


#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>

class command_velocity_filter_node
{

public:

	command_velocity_filter_node();

	~command_velocity_filter_node();

	void initialize();

	void setup_ros();


private:

	void cmd_vel_callback(const geometry_msgs::TwistConstPtr & msg);

	void set_latest_velocities(const geometry_msgs::Twist & vel);

	void reset_latest_velocities();

	void timer_callback(const ros::TimerEvent & event);

	void filter_velocities(const geometry_msgs::Twist & input_vel, const geometry_msgs::Twist & prev_vel, geometry_msgs::Twist & output_vel, double dt);

	void publish_cmd_velocity(const geometry_msgs::Twist & vel);

	// node handles
	ros::NodeHandle n;
	ros::NodeHandle pn;

	//ros subscribers
	ros::Subscriber cmd_vel_sub;

	//ros publisher
	ros::Publisher cmd_vel_pub;

	//ros timer
	ros::Timer cmd_vel_publish_timer;

	//parameters
	std::string frame_name;
	std::string received_cmd_vel_topic_name;
	std::string publish_cmd_vel_topic_name;
	double cmd_vel_pub_rate;
	double acc_linear_limit;
	double acc_angular_limit;

	geometry_msgs::Twist latest_recv_velocities_;
	geometry_msgs::Twist prev_cmd_vel_;
	ros::Time last_cmd_time_;

};

template<typename T>
T get_sign(T value)
{
	return (value < 0) ? -1 : 1;
}



#endif /* COMMAND_VELOCITY_FILTER_H_ */
