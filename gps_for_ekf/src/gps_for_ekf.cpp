#include <gps_for_ekf/gps_for_ekf.h>

double covariance[36] = {10.0, 0, 0, 0, 0, 0,  // covariance on gps_x
                      0, 10.0, 0, 0, 0, 0,  // covariance on gps_y
                      0, 0, 10.0, 0, 0, 0,  // covariance on gps_z
                      0, 0, 0, 99999, 0, 0,  // large covariance on rot x
                      0, 0, 0, 0, 99999, 0,  // large covariance on rot y
                      0, 0, 0, 0, 0, 99999};  // large covariance on rot z

GPSForEKF::GPSForEKF()
{
  gpsd_subscriber_ = nh_.subscribe<gps_common::GPSFix>("fix", 1, boost::bind(&GPSForEKF::gpsFixCallback, this, _1));
  gps_publisher_ = nh_.advertise<nav_msgs::Odometry>("topic_name",5);
}

void GPSForEKF::gpsFixCallback(const gps_common::GPSFixConstPtr &fix)
{
  nav_msgs::Odometry msg;
  
  //values are set to nan when no gps measurements are received 
  if(isnan(fix->longitude) || isnan(fix->latitude))
    return;

  msg.header.stamp = ros::Time::now();           // time of gps measurement
  msg.header.frame_id = "base_footprint";        // the tracked robot frame
  msg.pose.pose.position.x = fix->longitude;     // x measurement GPS.
  msg.pose.pose.position.y = fix->latitude;      // y measurement GPS.
  msg.pose.pose.position.z = 0.0;                // z measurement GPS.
  msg.pose.pose.orientation.x = 1;               // identity quaternion
  msg.pose.pose.orientation.y = 0;               // identity quaternion
  msg.pose.pose.orientation.z = 0;               // identity quaternion
  msg.pose.pose.orientation.w = 0;               // identity quaternion
  
  for(int i=0; i < 36; i++)
    msg.pose.covariance[i] = covariance[i];

  gps_publisher_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"gps_for_ekf");

  GPSForEKF gps;

  ros::spin();

  return 0;
}

