/*
 *  Copyright (C) 2009-2010 Ross Knepper, Carnegie Mellon University
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

#include <stdio.h>
#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>


#define ESC 27
#define LATCH_TIME 1.0  // seconds


double rollmax, rollmin, rolltmax, rolltmin;
double pitchmax, pitchmin, pitchtmax, pitchtmin;
double yawmax, yawmin, yawtmax, yawtmin;
nav_msgs::Odometry omax, omin, tmax, tmin;

void
latch(double now, unsigned long offset, const void *valp,
      void *omaxp, void *ominp, void *tmaxp, void *tminp)
{
    double val = *(double*)((char*)valp + offset);
    double &omax = *(double*)((char*)omaxp + offset);
    double &omin = *(double*)((char*)ominp + offset);
    double &tmax = *(double*)((char*)tmaxp + offset);
    double &tmin = *(double*)((char*)tminp + offset);

    if (val > omax || now > tmax + LATCH_TIME) {
        omax = val;
        tmax = now;
    } else if (val < omin || now > tmin + LATCH_TIME) {
        omin = val;
        tmin = now;
    }
}


void
cb(const nav_msgs::OdometryConstPtr &msg)
{
    double now = ros::Time::now().toSec();
    const geometry_msgs::Quaternion &quat = msg->pose.pose.orientation;
    double roll, pitch, yaw;
    double squ = quat.w * quat.w;
    double sqx = quat.x * quat.x;
    double sqy = quat.y * quat.y;
    double sqz = quat.z * quat.z;

    roll = atan2(2 * (quat.y*quat.z + quat.w*quat.x), squ - sqx - sqy + sqz);

    pitch = asin(-2 * (quat.x*quat.z - quat.w * quat.y));

    yaw = atan2(2 * (quat.x*quat.y + quat.w*quat.z), squ + sqx - sqy - sqz);

    printf("%c[1;1H", ESC);

    printf("pose/position: (%10f, %10f, %10f)          \n",
           msg->pose.pose.position.x, msg->pose.pose.position.y,
           msg->pose.pose.position.z);
    printf("pose/orientation: (%10f, %10f, %10f)          \n",
           roll, pitch, yaw);

    printf("twist/linear: (%10f, %10f, %10f)          \n",
           msg->twist.twist.linear.x, msg->twist.twist.linear.y,
           msg->twist.twist.linear.z);
    printf("twist/angular: (%10f, %10f, %10f)          \n",
           msg->twist.twist.angular.x, msg->twist.twist.angular.y,
           msg->twist.twist.angular.z);

    latch(now, (size_t)&msg->pose.pose.position.x - (size_t)&*msg,
          &*msg, &omax, &omin, &tmax, &tmin);
    latch(now, (size_t)&msg->pose.pose.position.y - (size_t)&*msg,
          &*msg, &omax, &omin, &tmax, &tmin);
    latch(now, (size_t)&msg->pose.pose.position.z - (size_t)&*msg,
          &*msg, &omax, &omin, &tmax, &tmin);
    latch(now, 0L, &roll, &rollmax, &rollmin, &rolltmax, &rolltmin);
    latch(now, 0L, &pitch, &pitchmax, &pitchmin, &pitchtmax, &pitchtmin);
    latch(now, 0L, &yaw, &yawmax, &yawmin, &yawtmax, &yawtmin);
    latch(now, (size_t)&msg->twist.twist.linear.x - (size_t)&*msg,
          &*msg, &omax, &omin, &tmax, &tmin);
    latch(now, (size_t)&msg->twist.twist.linear.y - (size_t)&*msg,
          &*msg, &omax, &omin, &tmax, &tmin);
    latch(now, (size_t)&msg->twist.twist.linear.z - (size_t)&*msg,
          &*msg, &omax, &omin, &tmax, &tmin);
    latch(now, (size_t)&msg->twist.twist.angular.x - (size_t)&*msg,
          &*msg, &omax, &omin, &tmax, &tmin);
    latch(now, (size_t)&msg->twist.twist.angular.y - (size_t)&*msg,
          &*msg, &omax, &omin, &tmax, &tmin);
    latch(now, (size_t)&msg->twist.twist.angular.z - (size_t)&*msg,
          &*msg, &omax, &omin, &tmax, &tmin);

    printf("\nLatching values (max): \n");
    printf("pose/position: (%10f, %10f, %10f)          \n",
           omax.pose.pose.position.x, omax.pose.pose.position.y,
           omax.pose.pose.position.z);
    printf("pose/orientation: (%10f, %10f, %10f)          \n",
           rollmax, pitchmax, yawmax);
    printf("twist/linear: (%10f, %10f, %10f)          \n",
           omax.twist.twist.linear.x, omax.twist.twist.linear.y,
           omax.twist.twist.linear.z);
    printf("twist/angular: (%10f, %10f, %10f)          \n",
           omax.twist.twist.angular.x, omax.twist.twist.angular.y,
           omax.twist.twist.angular.z);

    printf("\nLatching values (min): \n");
    printf("pose/position: (%10f, %10f, %10f)          \n",
           omin.pose.pose.position.x, omin.pose.pose.position.y,
           omin.pose.pose.position.z);
    printf("pose/orientation: (%10f, %10f, %10f)          \n",
           rollmin, pitchmin, yawmin);
    printf("twist/linear: (%10f, %10f, %10f)          \n",
           omin.twist.twist.linear.x, omin.twist.twist.linear.y,
           omin.twist.twist.linear.z);
    printf("twist/angular: (%10f, %10f, %10f)          \n",
           omin.twist.twist.angular.x, omin.twist.twist.angular.y,
           omin.twist.twist.angular.z);
}


int
main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, cb);

    printf("%c[2J", ESC);

    ros::spin();
}
