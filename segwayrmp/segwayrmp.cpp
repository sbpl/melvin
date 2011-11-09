/*
 *  Copyright (C) 2003  John Sweeney & Brian Gerkey
 *  Copyright (C) 2007-2010 Mike Vande Weghe, Carnegie Mellon University
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

  
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <ros/ros.h>

#include "rmp_frame.h"
#include "segwayrmp.h"
#define 	DTOR(d)   ((d) * M_PI / 180.0)
#define 	NORMALIZE(z)   atan2(sin(z), cos(z))

// Number of RMP read cycles, without new speed commands from clients,
// after which we'll stop the robot (for safety).
#define RMP_TIMEOUT_MSECS 800

double covariance[36] = {2.0, 0, 0, 0, 0, 0,  // covariance on gps_x
                         0, 2.0, 0, 0, 0, 0,  // covariance on gps_y
                         0, 0, 2.0, 0, 0, 0,  // covariance on gps_z
                         0, 0, 0, M_PI/16, 0, 0,  // large covariance on rot x
                         0, 0, 0, 0, M_PI/16, 0,  // large covariance on rot y
                         0, 0, 0, 0, 0, M_PI/4};  // large covariance on rot z


////////////////////////////////////////////////////////////////////////////////
// Constructor
SegwayRMP::SegwayRMP()
  : odom(), nh(), tf(), curr_xspeed(0), curr_yawspeed(0)
{
    odom.header.frame_id = "/segway_odom_origin";
    oPub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    spPub = nh.advertise<std_msgs::Float32>("segway_power", 1);
    srPub = nh.advertise<segwayrmp::RawData>("segway_rawdata", 1);
    cvSub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1,
                                               &SegwayRMP::cmd_vel_cb, this);
    omSub = nh.subscribe<std_msgs::String>("operating_mode", 1,
                                           &SegwayRMP::op_mode_cb, this);
    this->canio = NULL;
    //nh.param("canio", caniotype, std::string("kvaser"));  //disabed since we only use one type
    nh.param("canio_channel", caniochannel, 0);
    nh.param("max_xspeed", max_xspeed, 1.5); // meters/sec

    if (max_xspeed < 0) {
        max_xspeed = -max_xspeed;
    }

    nh.param("max_yawspeed", max_yawspeed, 3.0);  // radians/sec

    if (this->max_yawspeed < 0) {
        this->max_yawspeed = -this->max_yawspeed;
    }

    nh.param("length", length, 0.610);
    nh.param("width", width, 0.508);
    nh.param("yaw_dot_bias", yaw_dot_bias, 0.0);

    // create the transforms from base_link to each unrotated wheel.
    // height of 0.3 is a guess
    btQuaternion aa, bb;

    aa.setRPY(-M_PI_2, 0.0, 0.0);
    bb.setRPY(M_PI_2, 0.0, 0.0);

    left_wheel_tf_base = btTransform(aa, btVector3(0.0, 0.27,0.2365));
    right_wheel_tf_base = btTransform(bb, btVector3(0.0, -0.27, 0.2365));
    ClearOdometry();

    this->motor_enabled = true;
    this->firstread = true;
    // set the initial time
    gettimeofday(&(this->lasttime),NULL);
}


SegwayRMP::~SegwayRMP()
{
}


int
SegwayRMP::Shutdown()
{
    ROS_INFO("Shutting down CAN bus");
    
    // send zero velocities, for some semblance of safety
    CanPacket pkt;

    MakeVelocityCommand(&pkt,0,0);
  
    Write(pkt);

    // shutdown the CAN
    canio->Shutdown();
    delete canio;
    canio = NULL;
  
    return 0;
}


// Main function for device thread.
void 
SegwayRMP::spin()
{
    assert(this->canio = new CANIOrmpusb);

    if (this->canio->Init() < 0) {
        ROS_FATAL("error on CAN Init\n");
    }

    CanPacket pkt;

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
    ROS_INFO( "starting main loop\n");

    while (nh.ok()) {  	
        // Read from the RMP
        int ret = ReadFrame();

        if (ret < 0) {
            ROS_FATAL("ReadFrame() errored; bailing");
        }

        if (ret == 1) {
            oPub.publish(odom);       
            spPub.publish(power);
            srPub.publish(raw);
            /*tf::Transform txIdentity(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
                                     tf::Point(0.0, 0.0, 0.0));*/

            // This is updating the robot position with respect to the world
            tf.sendTransform(tf::StampedTransform(tf::Transform(
                             tf::Quaternion(odom.pose.pose.orientation.x,
                                            odom.pose.pose.orientation.y,
                                            odom.pose.pose.orientation.z,
                                            odom.pose.pose.orientation.w),
                             tf::Point(odom.pose.pose.position.x,
                                       odom.pose.pose.position.y, 0.0)
								), odom.header.stamp, "/odom", "/base_footprint"));

	    // the wheel transforms are used just for visualizing wheel
	    // rotation in Rviz.  You can comment them out if you want the
	    // driver to run slightly faster.
	    float left_wheel_rotations = raw.left / RMP_WHEEL_COUNT_PER_M
	                               / (2.0 * M_PI * RMP_WHEEL_RADIUS);
	    float right_wheel_rotations = raw.right / RMP_WHEEL_COUNT_PER_M
	                               / (2.0 * M_PI * RMP_WHEEL_RADIUS);
	    // subtract the integer portion, leaving just the fraction
	    // subtract the integer portion, leaving just the fraction
	    left_wheel_rotations -= truncf(left_wheel_rotations);
	    right_wheel_rotations -= truncf(right_wheel_rotations);
            btQuaternion leftQuat, rightQuat;
            leftQuat.setRPY(0.0, 0.0, left_wheel_rotations * 2.0 * M_PI);
            rightQuat.setRPY(0.0, 0.0, -right_wheel_rotations * 2.0 * M_PI);
	    btTransform left_tf = left_wheel_tf_base * btTransform(leftQuat);
	    btTransform right_tf = right_wheel_tf_base * btTransform(rightQuat);
	    tf.sendTransform(tf::StampedTransform (left_tf,ros::Time::now(),
                             "segway_base", "segway_wheel_left"));
	    tf.sendTransform(tf::StampedTransform (right_tf,ros::Time::now(),
                             "segway_base", "segway_wheel_right"));
	    
        }

        // check to see how long since the last command
        struct timeval curr;

        // get the current time
        gettimeofday(&curr, NULL);

        // calculate how long since the last command
        double msecs = (curr.tv_sec - this->lasttime.tv_sec) * 1000.0
                     + (curr.tv_usec - this->lasttime.tv_usec) / 1000.0;

        if (msecs > RMP_TIMEOUT_MSECS) {
            if (curr_xspeed || curr_yawspeed) {
                //ROS_WARNING("timeout exceeded: %d msecs since last command "
                //            "stopping robot\n", (int) msecs);
                curr_xspeed = 0.0;
                curr_yawspeed = 0.0;
            }
        }

        if (!motor_enabled) {
            curr_xspeed = 0.0;
            curr_yawspeed = 0.0;
        }

        // make a velocity command... could be zero
        MakeVelocityCommand(&pkt, curr_xspeed, curr_yawspeed);
        if (Write(pkt) < 0) {
            ROS_ERROR("error on write\n");    	
        }

        ros::spinOnce();

        usleep(10000);

    } //end while loop

    //do shutdown stuff here:
    Shutdown();
}


void
SegwayRMP::cmd_vel_cb(const geometry_msgs::TwistConstPtr &cmd_vel)
{
  this->curr_xspeed = cmd_vel->linear.x; // meters/sec
  this->curr_yawspeed = cmd_vel->angular.z;  // radians/sec
  ROS_DEBUG("received command %f %f\n", cmd_vel->linear.x, cmd_vel->angular.z);
  gettimeofday(&(this->lasttime), NULL);
}


void
SegwayRMP::op_mode_cb(const std_msgs::StringConstPtr &op_mode)
{
    //use strings so it's harder to send accidental signal
    if (op_mode->data == std::string("enable")) {
        this->motor_enabled = true;
    } else if (op_mode->data == std::string("disable")) {
        this->motor_enabled = false;
    } else if (op_mode->data == std::string("reset")) {
        ClearOdometry();
        firstread = true;
        CanPacket pkt;
        MakeStatusCommand(&pkt, (uint16_t)RMP_CAN_CMD_RST_INT,
                          (uint16_t)RMP_CAN_RST_ALL);
        if (Write(pkt) < 0) {
            ROS_ERROR("SegwayRMP:: error sending reset command\n");
        }
    }
}


int
SegwayRMP::ReadFrame(void)
{
    CanPacket pkt[CANIOrmpusb::MAX_EXTRACTED_PACKETS];
    rmp_frame_t data_frame;
    data_frame.ready = 0;
    
    int packets = canio->ReadFrame(pkt);
    if (packets < 0) {
        return -1;
    }

    while (packets > 0) {
        data_frame.AddPacket(pkt[packets-1]);
        packets--;
    }

    if (data_frame.IsReady()) {
      UpdateData(&data_frame);
      data_frame.ready = 0;
      return 1;
    }

    return 0;
}


int
SegwayRMP::Read(void)
{
    CanPacket pkt;
    int ret;
    rmp_frame_t data_frame;

    //static struct timeval last;
    //struct timeval curr;

    data_frame.ready = 0;

    ret = 0;
    // read until we've gotten all 8 packets for this cycle on this channel
    while ((ret = canio->ReadPacket(&pkt)) > 0)
    {
        // then we got a new packet...
        //log(ros::DEBUG,"SEGWAYIO: pkt: %s\n", pkt.toString());

        data_frame.AddPacket(pkt);

        // if data has been filled in, then let's make it the latest data 
        // available to ROS...
        if (data_frame.IsReady()) {
            UpdateData(&data_frame);
            data_frame.ready = 0;
            break;
        }
    }

    if (ret < 0) {
        ROS_ERROR("error (%d) reading packet on channel %d", ret,
                  this->caniochannel);
    }

    return 0;
}


void
SegwayRMP::UpdateData(rmp_frame_t * data_frame)
{
    double trust_gyro = 0.5; // A number in [0 ... 1]
    static ros::Time first, last = ros::Time(0);
    ros::Time now = ros::Time::now();
    int delta_lin_raw, delta_ang_raw;
    double delta_lin, delta_ang;
    double roll, pitch, roll_dot, pitch_dot;
    double gyro_yaw_rate, odom_yaw_rate, yaw_dot;
    double dt = (now - last).toSec();

    // ROS now requires us to set the time stamp ourselves, so set
    // it ASAP to closely match the true reading.
    odom.header.stamp = ros::Time::now();

    //to ensure that the odometry starts off as zero:
    static bool firstupdate=true;
    if(firstupdate) {
    	this->last_raw_foreaft = data_frame->foreaft;
	firstupdate=false;
    }

    // Get the new linear and angular encoder values and compute
    // odometry.
    delta_lin_raw = Diff(this->last_raw_foreaft, data_frame->foreaft);
    this->last_raw_foreaft = data_frame->foreaft;

    delta_ang_raw = Diff(this->last_raw_yaw, data_frame->yaw);
    this->last_raw_yaw = data_frame->yaw;

    roll = DTOR(double(data_frame->roll) / double(RMP_GYRO_COUNT_PER_DEG));
    pitch = DTOR(double(data_frame->pitch) / double(RMP_GYRO_COUNT_PER_DEG));
    roll_dot = DTOR(double(data_frame->roll_dot) /
                    double(RMP_GYRO_COUNT_PER_DEG_PER_S));
    pitch_dot = DTOR(double(data_frame->pitch_dot) /
                     double(RMP_GYRO_COUNT_PER_DEG_PER_S));
    gyro_yaw_rate = DTOR(-double(data_frame->yaw_dot) /
                         double(RMP_GYRO_COUNT_PER_DEG_PER_S)) - yaw_dot_bias;

    delta_lin = (double)delta_lin_raw / (double)RMP_WHEEL_COUNT_PER_M;
    delta_ang = (double)delta_ang_raw / (double)RMP_YAW_COUNT_PER_REV 
                                        * M_PI* 2.0;
    odom_yaw_rate = delta_ang / dt;

    yaw_dot = gyro_yaw_rate * trust_gyro + odom_yaw_rate * (1.0 - trust_gyro);

    // First-order odometry integration
    //this->odom_x += delta_lin * cos(this->odom_yaw);
    //this->odom_y += delta_lin * sin(this->odom_yaw);
    this->odom_yaw += delta_ang;

    // Normalize yaw in [0, 2PI]
    this->odom_yaw = atan2(sin(this->odom_yaw), cos(this->odom_yaw));
    if (this->odom_yaw < 0) {
        this->odom_yaw += 2.0 * M_PI;
    }

    odom.pose.pose.position.x += delta_lin * cos(odom_yaw);
    odom.pose.pose.position.y += delta_lin * sin(odom_yaw);
    odom.pose.pose.position.z = 0.0;

    btQuaternion qt;
    qt.setRPY(roll, pitch, this->odom_yaw);
    this->odom.pose.pose.orientation.x = qt.x();
    this->odom.pose.pose.orientation.y = qt.y();
    this->odom.pose.pose.orientation.z = qt.z();
    this->odom.pose.pose.orientation.w = qt.w();

    // combine left and right wheel velocity to get forward velocity
    // change from counts/s into m/s
    this->odom.twist.twist.linear.x = ((double)data_frame->left_dot +
                                       (double)data_frame->right_dot) /
                                       (double)RMP_WHEELSPEED_COUNT_PER_M_PER_S / 2.0;

    // no side speeds for this bot
    this->odom.twist.twist.linear.y = 0.0;
    this->odom.twist.twist.linear.z = 0.0;

    // from counts/sec into deg/sec.  also, take the additive
    // inverse, since the RMP reports clockwise angular velocity as
    // positive.
    this->odom.twist.twist.angular.z = yaw_dot;
    this->odom.twist.twist.angular.y = pitch_dot;
    this->odom.twist.twist.angular.x = roll_dot;

    if (last == ros::Time(0)) {
        first = now;
    }
    last = now;

    // fill in power data.  the RMP returns a percentage of full,
    // and the specs for the HT say that it's a 72 volt system.  assuming
    // that the RMP is the same, we'll convert to decivolts for ROS.
    this->power.data = (float)data_frame->battery;

    // raw values, for those who are interested
    this->raw.left      = data_frame->left;
    this->raw.right     = data_frame->right;
    this->raw.foreaft   = data_frame->foreaft;
    this->raw.yaw       = data_frame->yaw;
    this->raw.pitch     = data_frame->pitch;
    this->raw.roll      = data_frame->roll;
    this->raw.left_dot  = data_frame->left_dot;
    this->raw.right_dot = data_frame->right_dot;
    this->raw.yaw_dot   = data_frame->yaw_dot;
    this->raw.pitch_dot = data_frame->pitch_dot;
    this->raw.roll_dot  = data_frame->roll_dot;
    this->raw.frames    = data_frame->frames;
    this->raw.battery   = data_frame->battery;

    firstread = false;  
}  


int
SegwayRMP::Write(CanPacket& pkt)
{
    return canio->WritePacket(pkt);
}

int16_t SegwayRMP::trans_speed_to_rmp_counts(double xspeed) {
    // put in the last speed commands as well
  int16_t trans = (int16_t) (xspeed * RMP_WHEELSPEED_COUNT_PER_M_PER_S);
  if (trans > RMP_MAX_TRANS_VEL_COUNT) {
    trans = RMP_MAX_TRANS_VEL_COUNT;
  } else if (trans < -RMP_MAX_TRANS_VEL_COUNT) {
    trans = -RMP_MAX_TRANS_VEL_COUNT;
  }
  return trans;
}

int16_t SegwayRMP::rot_speed_to_rmp_counts(double yawspeed) {
  int16_t rot = (int16_t) (yawspeed / 2.0 / M_PI *
			       RMP_YAWSPEED_COUNT_PER_REV_PER_S);
  if (rot > RMP_MAX_ROT_VEL_COUNT) {
    rot = RMP_MAX_ROT_VEL_COUNT;
  } else if (rot < -RMP_MAX_ROT_VEL_COUNT) {
    rot = -RMP_MAX_ROT_VEL_COUNT;
  }
  return rot;
}

/* Creates a status CAN packet from the given arguments
 */  
void
SegwayRMP::MakeStatusCommand(CanPacket* pkt, uint16_t cmd, uint16_t val)
{
    pkt->id = RMP_CAN_ID_COMMAND;
    pkt->PutSlot(2, cmd);

    // it was noted in the windows demo code that they
    // copied the 8-bit value into both bytes like this
    pkt->PutByte(6, val);
    pkt->PutByte(7, val);

    // include the current speeds, too
    int16_t trans = trans_speed_to_rmp_counts(curr_xspeed);
    int16_t rot =   rot_speed_to_rmp_counts(curr_yawspeed);
    // We've noticed that the Segway can't turn in place with a pure commanded
    // rotation; this is an attempt to help.
    // From measurements, we found that adding an extra translation of the
    // same sign as the rotation seems to help.
    if (rot > 0) {
        trans += 16;
    } else if (rot < 0) {
        trans -= 19;
    }
    pkt->PutSlot(0, (uint16_t)trans);
    pkt->PutSlot(1, (uint16_t)rot);

    if (cmd) {
        ROS_DEBUG("SEGWAYIO: STATUS: cmd: %02x val: %02x pkt: %s\n", 
                 cmd, val, pkt->toString());
    }
}


/* takes a command (in host byte order) and turns it into CAN packets 
 * for the RMP 
 */
void
SegwayRMP::MakeVelocityCommand(CanPacket* pkt, 
                               double xspeed, 
                               double yawspeed)
{
    pkt->id = RMP_CAN_ID_COMMAND;
    pkt->PutSlot(2, (uint16_t)RMP_CAN_CMD_NONE);

    if (xspeed > this->max_xspeed) {
      ROS_WARN("xspeed thresholded!(%f > %f)", xspeed, this->max_xspeed);
      xspeed = this->max_xspeed;
    } else if (xspeed < -this->max_xspeed) {
      ROS_WARN("xspeed thresholded! (%f<%f)", xspeed, -this->max_xspeed);
      xspeed = -this->max_xspeed;
    }

    if (yawspeed > this->max_yawspeed) {
      ROS_WARN("yawspeed thresholded! (%f > %f)", yawspeed,
		  this->max_yawspeed);
      yawspeed = this->max_yawspeed;
    } else if (yawspeed < -this->max_yawspeed) {
      ROS_WARN("yawspeed thresholded! (%f < %f)", yawspeed,
		  this->max_yawspeed);
      yawspeed = -this->max_yawspeed;
    }

    this->curr_xspeed = xspeed;
    this->curr_yawspeed = yawspeed;
    
    ROS_DEBUG("Converting %2.3f trans %2.3f rot to counts\n",
	      xspeed,yawspeed);
    // convert from command units to wheelspeed counts/second
    int16_t trans = trans_speed_to_rmp_counts(xspeed);
    int16_t rot =   rot_speed_to_rmp_counts(yawspeed);


    // We've noticed that the Segway can't turn in place with a pure commanded
    // rotation; this is an attempt to help.
    // From measurements, we found that adding an extra translation of the
    // same sign as the rotation seems to help.
    //    if (rot > 0) {
    //        trans += 16;
    //    } else if (rot < 0) {
    //        trans -= 19;
    //    }

    pkt->PutSlot(0, (uint16_t)trans);
    pkt->PutSlot(1, (uint16_t)rot);

    ROS_DEBUG("Sending to segway: trans=%d rot=%d\n",
    	      trans,rot);

    static int printcount = 0;
    if (++printcount > 100) {
        //     log(ros::DEBUG,"trans=%d rot=%d\n",trans,rot);
        printcount = 0;
    }

    // alternate between the three config commands we want to add to the packet
    static int config_command = 0;
    if (++config_command == 1) {
        pkt->PutSlot(2, (uint16_t)10);  // set the scaling value (parameter 10)
        pkt->PutSlot(3, (uint16_t)16);  // to 1.0 (16 means 1.0)
    } else if (config_command == 2) {
        pkt->PutSlot(2, (uint16_t)15);  // set the balance lockout mode (param 15)
        pkt->PutSlot(3, (uint16_t)1);  // to locked-out (value 1)
    } else if (config_command == 3) {
        pkt->PutSlot(2, (uint16_t)14);  // set the current limit scale factor (param 14)
        pkt->PutSlot(3, (uint16_t)256);  // to 1.0 (value 256 means 1.0)
        config_command = 0;
    }
}


void
SegwayRMP::MakeShutdownCommand(CanPacket* pkt)
{
    pkt->id = RMP_CAN_ID_SHUTDOWN;

    ROS_INFO("SEGWAYIO: SHUTDOWN: pkt: %s\n", pkt->toString());
}


// Calculate the difference between two raw counter values, taking care
// of rollover.
int
SegwayRMP::Diff(uint32_t from, uint32_t to)
{

    // if this is the first time, report no change
    if (firstread) {
        return 0;
    }

    /*
     * Due to the magic of 2's complement, no special tricks are needed.
     */
    return int(to - from);
}
#if 0
int
SegwayRMP::Diff(uint32_t from, uint32_t to)
{
    int diff1, diff2;
    static uint32_t max = (unit32_t) ~0L; //(uint32_t)pow(2.0, 32)-1;

    // if this is the first time, report no change
    if (firstread) {
        return 0;
    }

    diff1 = to - from;

    /* find difference in two directions and pick shortest */
    if (to > from) {
        diff2 = -(from + max - to);
    } else  {
        diff2 = max - from + to;
    }

    if (abs(diff1) < abs(diff2))  {
        return(diff1);
    } else {
        return(diff2);
    }
}
#endif


void
SegwayRMP::ClearOdometry(void)
{
    // Initialize to all zeros
    odom = nav_msgs::Odometry();
    odom.header.frame_id = "/segway_odom_origin";
    odom_yaw = 0.0;

    // TODO: set covariance matrices to something meaningful
    for(int i=0; i < 36; i++)
      odom.pose.covariance[i] = covariance[i];
    
}


int
main(int argc, char **argv)
{
    // Initialize ros
    ros::init(argc, argv, "segwayrmp");

    // Create a new instance of pinger
    SegwayRMP s;

    // Wait for pinger to finish
    s.spin();

    // Done
    std::cout << "SegwayRMP is done!" << std::endl;

    return 0;
}
