// system includes
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <segbot_msgs/ServoAngle.h>
#include <tf/transform_broadcaster.h>

namespace servo_controller {

class ServoController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:

    bool init(hardware_interface::PositionJointInterface* robot, ros::NodeHandle& controller_nh);
    bool init(hardware_interface::PositionJointInterface* robot, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting();
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping();

private:

    hardware_interface::JointHandle base_servo_joint;
    hardware_interface::JointHandle tilt_servo_joint;
    ros::NodeHandle nh_;
    float baseAngle;
    float tiltAngle;

    ros::Subscriber baseServoAngle_sub;
    ros::Subscriber tiltServoAngle_sub;
    void baseCmdCallback(const segbot_msgs::ServoAngleConstPtr& msg);
    void tiltCmdCallback(const segbot_msgs::ServoAngleConstPtr& msg);
};

} // namespace servo_controller
