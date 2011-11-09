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

#if !defined(_segwayrmp_h_)
#define _segwayrmp_h_


#include <unistd.h>
#include "canio.h"
//#include "canio_kvaser.h"
#include "canio_rmpusb.h"
#include "ros/node_handle.h"
#include "time.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "segwayrmp/RawData.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <math.h>
#include "boost/thread/mutex.hpp"
#include "std_msgs/String.h"

#include "tf/transform_broadcaster.h"

// Forward declarations
class rmp_frame_t;

// Driver for robotic Segway
class SegwayRMP
{
public: 
    // Constructors etc
    SegwayRMP();
    virtual ~SegwayRMP();

    // Setup/shutdown routines.
    virtual int Shutdown();
    nav_msgs::Odometry odom;
    std_msgs::Float32 power;
    segwayrmp::RawData raw;
    void spin();
    ros::NodeHandle nh;
    tf::TransformBroadcaster tf;

private:
    void cmd_vel_cb(const geometry_msgs::TwistConstPtr &);
    void op_mode_cb(const std_msgs::StringConstPtr &);
    
    ros::Publisher oPub, spPub, srPub;
    ros::Subscriber cvSub, omSub;
    
    int16_t trans_speed_to_rmp_counts(double xspeed);
    int16_t rot_speed_to_rmp_counts(double yawspeed);

    const char* portname;
    int caniochannel;

    struct timeval lasttime;

    double max_xspeed, max_yawspeed;
    double length, width;
    double yaw_dot_bias;

    bool firstread;

    CANIO *canio;

    double curr_xspeed, curr_yawspeed; // command speeds (m/s and rad/s)

    // Flag set if motors are currently enabled
    bool motor_enabled;

    // For handling rollover
    uint32_t last_raw_yaw, last_raw_left, last_raw_right, last_raw_foreaft;

    // Odometry calculation
    double odom_yaw;

    // helper to read a cycle of data from the RMP
    int Read();
    int ReadFrame();

    // Calculate the difference between two raw counter values, taking care
    // of rollover.
    int Diff(uint32_t from, uint32_t to);

    // helper to write a packet
    int Write(CanPacket& pkt);

    // helper to create a status command packet from the given args
    void MakeStatusCommand(CanPacket* pkt, uint16_t cmd, uint16_t val);

    // helper to take a player command and turn it into a CAN command packet
    void MakeVelocityCommand(CanPacket* pkt, double xspeed, double yawspeed);
    
    void MakeShutdownCommand(CanPacket* pkt);

    void UpdateData(rmp_frame_t *);

    void ClearOdometry(void);

    btTransform left_wheel_tf_base, right_wheel_tf_base;
};

#endif // _segwayrmp_h_
