#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

namespace servo_sim_controller_ns{

	class ServoSimController: public pr2_controller_interface::Controller
	{
		private:
		  pr2_mechanism_model::JointState* base_servo_joint;
		  pr2_mechanism_model::JointState* tilt_servo_joint;
      ros::NodeHandle nh_;
      double base_desired;
      double base_min;
      double base_max;
      double base_desired_vel;
      double last_base_effort;
      double tilt_desired;
      double tilt_min;
      double tilt_max;
      double tilt_desired_vel;
      double last_tilt_effort;
      double last_t;
      double last_base_pos;
      double last_tilt_pos;
      double last_base_vel;
      double last_tilt_vel;

		public:
		  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
		  void starting();
		  void update();
		  void stopping();
	};
} 

