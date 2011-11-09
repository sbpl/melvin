#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>

using namespace std;

class GPSForEKF
{
  public:

    GPSForEKF();

    ~GPSForEKF(){};

  private:

    ros::NodeHandle nh_;

    ros::Publisher gps_publisher_;

    ros::Subscriber gpsd_subscriber_;

    void gpsFixCallback(const gps_common::GPSFix::ConstPtr& fix);
};

