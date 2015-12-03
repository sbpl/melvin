#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

namespace base_sim_controller_ns{

	class BaseSimController: public pr2_controller_interface::Controller
	{
		private:
		  pr2_mechanism_model::JointState* left_wheel_joint, * right_wheel_joint;
		  double linearVel, angularVel;
      double error_sum_right;
      double error_sum_left;
      double last_t;
      double last_error_right;
      double last_error_left;
      double last_right_torque;
      double last_left_torque;
      ros::NodeHandle nh_;

      ros::Publisher odom_pub;
      tf::TransformBroadcaster odom_broadcaster;
      double x;
      double y;
      double theta;


 
      ros::Subscriber cmd_sub;
      void cmdCallback(const geometry_msgs::TwistConstPtr& twist);

		public:
		  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
		  void starting();
		  void update();
		  void stopping();
	};
} 

