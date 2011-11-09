#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <segbot_msgs/ServoAngle.h>

namespace servo_controller_ns{

	class ServoController: public pr2_controller_interface::Controller
	{
		private:
		  pr2_mechanism_model::JointState* base_servo_joint;
		  pr2_mechanism_model::JointState* tilt_servo_joint;
      ros::NodeHandle nh_;
      float baseAngle;
      float tiltAngle;

      ros::Subscriber baseServoAngle_sub;
      ros::Subscriber tiltServoAngle_sub;
      void cmdCallback(const segbot_msgs::ServoAngleConstPtr& msg);

		public:
		  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
		  void starting();
		  void update();
		  void stopping();
	};
} 

