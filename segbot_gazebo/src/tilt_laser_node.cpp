#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tilt_laser_node");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    double min_angle;
    double max_angle;
    double velocity;
    bool down = true;

    if (!ph.getParam("min_angle_rad", min_angle) ||
        !ph.getParam("max_angle_rad", max_angle) ||
        !ph.getParam("velocity_rps", velocity))
    {
        ROS_ERROR("Failed to retrieve parameters from the param server");
        return 1;
    }

    double prev_cmd = 0.0;

    ros::Publisher cmd_pub = nh.advertise<std_msgs::Float64>("command", 1);

    ros::Rate loop_rate(10.0);
    ros::Duration loop_duration(loop_rate);

    while (ros::ok()) {
        double next_cmd = prev_cmd;
        if (down) {
            next_cmd -= loop_duration.toSec() * velocity;
        }
        else {
            next_cmd += loop_duration.toSec() * velocity;
        }

        // clamp and flip if necessary
        if (next_cmd < min_angle) {
            next_cmd = min_angle;
            down = !down;
        }
        else if (next_cmd > max_angle) {
            next_cmd = max_angle;
            down = !down;
        }

        std_msgs::Float64 next;
        next.data = next_cmd;
        cmd_pub.publish(next);
        prev_cmd = next_cmd;

        loop_rate.sleep();
    }

    return 0;
}
