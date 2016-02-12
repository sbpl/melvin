#include <servo_controller/servo_controller.h>
#include <pluginlib/class_list_macros.h>

namespace servo_controller {

/// Controller initialization in non-realtime
bool ServoController::init(
    hardware_interface::PositionJointInterface* robot,
    ros::NodeHandle& controller_nh)
{
    std::string base_servo_name;
    if (!controller_nh.getParam("base_servo_joint", base_servo_name)) {
        ROS_ERROR("No joint given in namespace: '%s')", controller_nh.getNamespace().c_str());
        return false;
    }

    std::string tilt_servo_name;
    if (!controller_nh.getParam("tilt_servo_joint", tilt_servo_name)) {
        ROS_ERROR("No joint given in namespace: '%s')", controller_nh.getNamespace().c_str());
        return false;
    }

    try {
        base_servo_joint = robot->getHandle(base_servo_name);
    }
    catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR("Failed to acquire Joint Handle to joint '%s' (%s)", base_servo_name.c_str(), ex.what());
        return false;
    }

    try {
        tilt_servo_joint = robot->getHandle(tilt_servo_name);
    }
    catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR("Failed to acquire Joint Handle to joint '%s' (%s)", tilt_servo_name.c_str(), ex.what());
        return false;
    }

    std::string baseServoAngleName;
    controller_nh.getParam("base_servo_angle_name", baseServoAngleName);
    baseServoAngle_sub = nh_.subscribe<segbot_msgs::ServoAngle>(
            baseServoAngleName,
            1,
            &ServoController::baseCmdCallback,
            this);
    baseAngle = 0;

    std::string tiltServoAngleName;
    controller_nh.getParam("tilt_servo_angle_name", tiltServoAngleName);
    tiltServoAngle_sub = nh_.subscribe<segbot_msgs::ServoAngle>(
            tiltServoAngleName.c_str(),
            1,
            &ServoController::tiltCmdCallback,
            this);
    tiltAngle = 0;
    return true;
}

bool ServoController::init(
    hardware_interface::PositionJointInterface* robot,
    ros::NodeHandle& root_nh,
    ros::NodeHandle& controller_nh)
{
    return init(robot, controller_nh);
}

/// Controller startup in realtime
void ServoController::starting()
{
}

/// Controller update loop in realtime
void ServoController::update(const ros::Time& time, const ros::Duration& period)
{
    base_servo_joint.setCommand(-baseAngle);
    tilt_servo_joint.setCommand(tiltAngle + M_PI / 180 * 4);
}

/// Controller stopping in realtime
void ServoController::stopping()
{
}

void ServoController::baseCmdCallback(
    const segbot_msgs::ServoAngleConstPtr& msg)
{
    baseAngle = msg->angle;
}

void ServoController::tiltCmdCallback(
    const segbot_msgs::ServoAngleConstPtr& msg)
{
    tiltAngle = msg->angle;
}

} // namespace servo_controller

/// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS(ServoControllerPlugin, servo_controller::ServoController, controller_interface::ControllerBase)
