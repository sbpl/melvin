#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher g_twist_pub;

void TwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    geometry_msgs::Twist transformed = *msg;
    transformed.angular.z = -transformed.angular.z;
    g_twist_pub.publish(transformed);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twist_ned_transformer");
    ros::NodeHandle nh;

    ros::Subscriber twist_sub = nh.subscribe("cmd_vel_in", 10, TwistCallback);
    g_twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_out", 10);

    ros::spin();
    return 0;
}
