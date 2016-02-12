// standard includes
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

// system includes
#include <controller_manager/controller_manager.h>
#include <getopt.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pthread.h>
#include <ros/ros.h>
#include <sys/stat.h>

using namespace std;

static struct
{
    char *program_;
    char *interface_;
    char *xml_;
    bool allow_unprogrammed_;
    bool stats_;
} g_options;

void Usage(string msg = "")
{
    fprintf(stderr, "Usage: %s [options]\n", g_options.program_);
    fprintf(stderr, "  Available options\n");
    fprintf(stderr, "    -i, --interface <interface> Connect to EtherCAT devices on this interface\n");
    fprintf(stderr, "    -x, --xml <file|param>      Load the robot description from this file or parameter name\n");
    fprintf(stderr, "    -u, --allow_unprogrammed    Allow control loop to run with unprogrammed devices\n");
    fprintf(stderr, "    -h, --help                  Print this message and exit\n");
    if (msg != "") {
        fprintf(stderr, "Error: %s\n", msg.c_str());
        exit(-1);
    }
    else {
        exit(0);
    }
}

static const int NSEC_PER_SEC = 1e+9;

static void timespecInc(struct timespec &tick, int nsec)
{
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= NSEC_PER_SEC) {
        tick.tv_nsec -= NSEC_PER_SEC;
        tick.tv_sec++;
    }
}

bool ParseLimits(
    TiXmlElement* root,
    const std::vector<std::string>& joint_names,
    std::vector<double>& joint_min_limits_out,
    std::vector<double>& joint_max_limits_out,
    std::vector<double>& joint_vel_limits_out,
    std::vector<double>& joint_eff_limits_out)
{
    // default to no limits
    const double def_min_limit = -std::numeric_limits<double>::max();
    const double def_max_limit = -std::numeric_limits<double>::max();
    const double def_vel_limit = std::numeric_limits<double>::max();
    const double def_eff_limit = std::numeric_limits<double>::max();
    std::vector<double> joint_min_limits(joint_names.size(), def_min_limit);
    std::vector<double> joint_max_limits(joint_names.size(), def_max_limit);
    std::vector<double> joint_vel_limits(joint_names.size(), def_vel_limit);
    std::vector<double> joint_eff_limits(joint_names.size(), def_eff_limit);

    const TiXmlNode* joint_node = nullptr;
    for (joint_node = root->FirstChild("joint"); joint_node;
        joint_node = root->IterateChildren("joint", joint_node))
    {
        const TiXmlElement* joint_element =
                dynamic_cast<const TiXmlElement*>(joint_node);
        if (!joint_element) {
            ROS_WARN("'joint' node is not an XML element?");
            continue;
        }

        const char* joint_name = joint_element->Attribute("name");
        const char* type = joint_element->Attribute("type");
        if (!joint_name || !type) {
            ROS_WARN("Joint element does not have a name or a type");
            continue;
        }

        auto jit = std::find(
                joint_names.begin(), joint_names.end(),
                std::string(joint_name));
        if (jit == joint_names.end()) {
            // this is not the joint you're looking for
            continue;
        }

        size_t jidx = std::distance(joint_names.begin(), jit);
        if (jidx < 0 || jidx >= joint_names.size()) {
            ROS_WARN("Joint index is invalid for joint '%s'", joint_name);
            continue;
        }

        ROS_INFO("Found joint '%s' at index %zu", joint_name, jidx);

        //////////////////////////
        // fill in joint limits //
        //////////////////////////

        // 0 DOF
        const bool is_fixed = strcmp(type, "fixed") == 0;

        // 1 DOF
        const bool is_revolute = strcmp(type, "revolute") == 0;
        const bool is_prismatic = strcmp(type, "prismatic") == 0;
        const bool is_continuous = strcmp(type, "continuous") == 0;

        // 3 DOF
        const bool is_planar = strcmp(type, "planar") == 0;

        // 6 DOF
        const bool is_floating = strcmp(type, "floating") == 0;

        const bool limit_required = is_revolute || is_prismatic;

        const TiXmlNode* limit_node = joint_element->FirstChild("limit");

        if (limit_node) {
            const TiXmlElement* limit_element =
                    dynamic_cast<const TiXmlElement*>(limit_node);
            if (!limit_element) {
                ROS_ERROR("'limit' node is not an XML element?");
                return false;
            }

            double lower_limit = -std::numeric_limits<double>::max();
            double upper_limit = std::numeric_limits<double>::max();
            double velocity_limit = std::numeric_limits<double>::max();
            double effort_limit = std::numeric_limits<double>::max();

            int res;

            // lower limits
            res = limit_element->QueryDoubleAttribute("lower", &lower_limit);
            if (res == TIXML_SUCCESS) {
                // yay
            }
            else if (res == TIXML_NO_ATTRIBUTE) {
                if (limit_required) {
                    ROS_ERROR("'lower' attribute of 'limit' tag is required for revolute and prismatic joints (joint = %s)", joint_name);
                    return false;
                }

                if (is_continuous) {
                    lower_limit = 0.0;
                }
            }
            else {
                ROS_ERROR("'lower' attribute of 'limit' tag is malformed (joint = %s)", joint_name);
                return false;
            }

            // upper limits
            res = limit_element->QueryDoubleAttribute("upper", &upper_limit);
            if (res == TIXML_SUCCESS) {
                // yay
            }
            else if (res == TIXML_NO_ATTRIBUTE) {
                if (limit_required) {
                    ROS_ERROR("'upper' attribute of 'limit' tag is required for revolute and pristmatic joints (joint = %s)", joint_name);
                }

                if (is_continuous) {
                    upper_limit = 0.0;
                }
            }
            else {
                ROS_ERROR("'upper' attribute of 'limit' tag is malformed (joint = %s)", joint_name);
                return false;
            }

            // velocity limits
            res = limit_element->QueryDoubleAttribute("velocity", &velocity_limit);
            if (res == TIXML_SUCCESS) {
                // yay
            }
            else if (res == TIXML_NO_ATTRIBUTE) {
                ROS_ERROR("'limit' tag requires a 'velocity' attribute (joint = %s)", joint_name);
                return false;
            }
            else {
                ROS_ERROR("'velocity' attribute of 'limit' tag is malformed (joint = %s)", joint_name);
                return false;
            }

            res = limit_element->QueryDoubleAttribute("effort", &effort_limit);
            if (res == TIXML_SUCCESS) {
                // yay
            }
            else if (res == TIXML_NO_ATTRIBUTE) {
                ROS_ERROR("'limit' tag requires a 'effort' attribute (joint = %s)", joint_name);
                return false;
            }
            else {
                ROS_ERROR("'effort' attribute of 'limit' tag is malformed (joint = %s)", joint_name);
                return false;
            }

            joint_min_limits[jidx] = lower_limit;
            joint_max_limits[jidx] = upper_limit;
            joint_vel_limits[jidx] = velocity_limit;
            joint_eff_limits[jidx] = effort_limit;
        }
        else {
            if (limit_required) {
                ROS_ERROR("'limit' tag required for revolute and prismatic joints (joint = %s)", joint_name);
                return false;
            }

            if (is_continuous) {
                joint_min_limits[jidx] = 0.0;
                joint_max_limits[jidx] = 0.0;
            }
        }
    }

    joint_min_limits_out = std::move(joint_min_limits);
    joint_max_limits_out = std::move(joint_max_limits);
    joint_vel_limits_out = std::move(joint_vel_limits);
    joint_eff_limits_out = std::move(joint_eff_limits);
    return true;
}

void* controlLoop(void *)
{
    int rv = 0;

    int period;
    int policy;


    ////////////////////////////
    // Load robot description //
    ////////////////////////////

    ROS_INFO("Parsing URDF");

    TiXmlDocument xml;
    struct stat st;
    if (0 == stat(g_options.xml_, &st)) {
        xml.LoadFile(g_options.xml_);
    }
    else {
        ROS_INFO("Xml file not found, reading from parameter server");
        ros::NodeHandle top_level_node;
        std::string robot_description;
        if (top_level_node.getParam(g_options.xml_, robot_description)) {
            xml.Parse(robot_description.c_str());
        }
        else {
            ROS_FATAL("Could not load the xml from parameter server: %s", g_options.xml_);
            rv = -1;
            return (void*)rv;
        }
    }

    TiXmlElement* root_element = xml.RootElement();
    TiXmlElement* root = xml.FirstChildElement("robot");
    if (!root || !root_element) {
        ROS_FATAL("Could not parse the xml from %s", g_options.xml_);
        rv = -1;
        return (void*)rv;
    }

    ////////////////////////////////////////////////////
    // Look through the URDF for the specified joints //
    ////////////////////////////////////////////////////

    std::vector<std::string> joint_names = {
        "segway_left_wheel_joint",
        "segway_right_wheel_joint",
        "segway_trailer_wheel_caster_joint",
        "segway_trailer_wheel_joint",
        "R283_hokuyo_laser1_joint",
        "R313_hokuyo_laser2_joint"
    };

    ///////////////////////////////////
    // initialize joint descriptions //
    ///////////////////////////////////

    ROS_INFO("Initializing joint limits");

    std::vector<double> joint_min_limits;
    std::vector<double> joint_max_limits;
    std::vector<double> joint_vel_limits;
    std::vector<double> joint_eff_limits;

    if (!ParseLimits(
            root, joint_names,
            joint_min_limits, joint_max_limits,
            joint_vel_limits, joint_eff_limits))
    {
        ROS_FATAL("Failed to parse URDF for limits");
        rv = -1;
        return (void*)rv;
    }

    //////////////////////////////
    // initialize joint buffers //
    //////////////////////////////

    ROS_INFO("Initializing joint command buffers");

    // default to 0 position
    std::vector<double> joint_positions(joint_names.size(), 0.0);
    std::vector<double> joint_velocities(joint_names.size(), 0.0);
    std::vector<double> joint_efforts(joint_names.size(), 0.0);

    // default to 0 command
    std::vector<double> joint_position_commands(joint_names.size(), 0.0);
    std::vector<double> joint_velocity_commands(joint_names.size(), 0.0);
    std::vector<double> joint_effort_commands(joint_names.size(), 0.0);

    // default to 0 position
    std::vector<double> joint_last_joint_position_commands(joint_names.size(), 0.0);

    ///////////////////////////////
    // initialize robot hardware //
    ///////////////////////////////

    ROS_INFO("Initializing robot hardware");

    hardware_interface::RobotHW hw;

    hardware_interface::JointStateInterface js_interface;
    hardware_interface::PositionJointInterface pj_interface;

    // create joint state and joint position command handles
    for (size_t jidx = 0; jidx < joint_names.size(); ++jidx) {
        js_interface.registerHandle(hardware_interface::JointStateHandle(
                joint_names[jidx],
                &joint_positions[jidx],
                &joint_velocities[jidx],
                &joint_efforts[jidx]));

        hardware_interface::JointHandle joint_pos_handle;
        joint_pos_handle = hardware_interface::JointHandle(
                js_interface.getHandle(joint_names[jidx]),
                &joint_position_commands[jidx]);
        pj_interface.registerHandle(joint_pos_handle);
    }

    hw.registerInterface(&js_interface);
    hw.registerInterface(&pj_interface);

    controller_manager::ControllerManager cm(&hw);

    // Set to realtime scheduler for this thread
    struct sched_param thread_param;
    policy = SCHED_FIFO;
    thread_param.sched_priority = sched_get_priority_max(policy);
    pthread_setschedparam(pthread_self(), policy, &thread_param);

    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    period = 1e+6; // 1 ms in nanoseconds

    // Snap to the nearest second
    tick.tv_sec = tick.tv_sec;
    tick.tv_nsec = (tick.tv_nsec / period + 1) * period;
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

    ros::Time last_now = ros::Time::now();
    while (ros::ok()) {
        const ros::Time now = ros::Time::now();
        cm.update(now, last_now - now);
        last_now = now;

        // Compute end of next period
        timespecInc(tick, period);

        struct timespec before;
        clock_gettime(CLOCK_REALTIME, &before);
        if ((before.tv_sec + before.tv_nsec / 1e9) > (tick.tv_sec + tick.tv_nsec / 1e9)) {
            // We overran, snap to next "period"
            tick.tv_sec = before.tv_sec;
            tick.tv_nsec = (before.tv_nsec / period) * period;
            timespecInc(tick, period);
        }

        // Sleep until end of period
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    }

end:

    return (void*)rv;
}

#define CLOCK_PRIO 0
#define CONTROL_PRIO 0

static pthread_t controlThread;
static pthread_attr_t controlThreadAttr;

int main(int argc, char *argv[])
{
    // Initialize ROS and parse command-line arguments
    ros::init(argc, argv, "realtime_loop");

    // Parse options
    g_options.program_ = argv[0];
    while (1) {
        static struct option long_options[] = {
            { "help", no_argument, 0, 'h' },
            { "allow_unprogrammed", no_argument, 0, 'u' },
            { "interface", required_argument, 0, 'i' },
            { "xml", required_argument, 0, 'x' },
        };
        int option_index = 0;
        int c = getopt_long(argc, argv, "hi:usx:", long_options, &option_index);
        if (c == -1)
            break;
        switch (c) {
        case 'h':
            Usage();
            break;
        case 'u':
            g_options.allow_unprogrammed_ = 1;
            break;
        case 'i':
            g_options.interface_ = optarg;
            break;
        case 'x':
            g_options.xml_ = optarg;
            break;
        case 's':
            g_options.stats_ = 1;
            break;
        }
    }
    if (optind < argc) {
        Usage("Extra arguments");
    }

    if (!g_options.xml_) {
        Usage("You must specify a robot description XML file");
    }

    // Start thread

    int rv;
    if ((rv = pthread_create(&controlThread, &controlThreadAttr, controlLoop, 0)) != 0) {
        ROS_FATAL("Unable to create control thread: rv = %d", rv);
        ROS_BREAK();
    }

    ros::spin();
    pthread_join(controlThread, (void **)&rv);

    return rv;
}
