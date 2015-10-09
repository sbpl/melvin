
#ifndef INCDpiDataTypes_hh
#define INCDpiDataTypes_hh

#include <string.h>

//#define ROBOT_POSE_FORM "{double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double}"
#define ROBOT_POSE_FORM "{double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double}"
#define ROBOT_POSE_MESSAGE "RobotPoseMessage"
#define ROBOT_POSE_MESSAGE_MOTION_PLANNER "RobotPoseMessage_Motion_Planner"
#define NEXT_WAYPOINT_TO_TRACK "NextWaypointToTrack"
#define PLANNER_GOAL_MESSAGE "PlannerGoalMessage"

// Commanded pose (timestamp is used as a path index)
#define POSE_CMD_FORM "{double, double, double, double, double, double, double, double, double, double, double, double, double}"
#define POSE_CMD_MESSAGE "PoseCmdMessage"

struct VehicleState {
	VehicleState() {
	}

	VehicleState(double _x, double _y, double _z, double _roll, double _pitch,
			double _yaw, double _forwardvelocity, double _vx, double _vy,
			double _vz, double _rollrate, double _pitchrate, double _yawrate,
			double _t = 0) :
		x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw),
				forwardvelocity(_forwardvelocity), vx(_vx), vy(_vy), vz(_vz),
				rollrate(_rollrate), pitchrate(_pitchrate), yawrate(_yawrate),
				t(_t) {
	}

	// Pose
	double x, y, z;
	double roll, pitch, yaw;

	// Translational velocities
	double forwardvelocity;
	double desiredforwardvelocity;
	double vx, vy, vz;

	// Angular velocities
	double rollrate, pitchrate, yawrate;

	// World frame velocities
	double world_vx, world_vy, world_vz;

	// Time stamp / index
	union {
		double t;
		double index;
	};

	VehicleState &operator=(const VehicleState &rhs) {
		if (this == &rhs)
			return *this;
		else {
			memcpy(this, &rhs, sizeof(VehicleState));
			return *this;
		}
	}

  static const char *getIPCFormat(void) {
    return ROBOT_POSE_FORM;
  }
};


#endif /* INCDpiDataTypes_hh */
