#include <stdio.h>
#include <stdint.h>

#include <chrono>
#include <string>
#include <thread>

#include <SDL/SDL.h>
#include <SDL/SDL_joystick.h>
#include <SDL/SDL_keyboard.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <spellbook/utils/utils.h>
#include <spellbook/stringifier/stringifier.h>

#define ENUM_TO_STRING

const char* to_cstr(SDL_EventType e)
{
#define TO_STRING_CLAUSE(ev) case ev: return #ev;
    switch (e) {
    TO_STRING_CLAUSE(SDL_QUIT)
    TO_STRING_CLAUSE(SDL_SYSWMEVENT)
    // keyboard events
    TO_STRING_CLAUSE(SDL_KEYDOWN)
    TO_STRING_CLAUSE(SDL_KEYUP)
//    TO_STRING_CLAUSE(SDL_TEXTEDITING)
//    TO_STRING_CLAUSE(SDL_TEXTINPUT)
    // mouse events
    TO_STRING_CLAUSE(SDL_MOUSEMOTION)
    TO_STRING_CLAUSE(SDL_MOUSEBUTTONDOWN)
    TO_STRING_CLAUSE(SDL_MOUSEBUTTONUP)
//    TO_STRING_CLAUSE(SDL_MOUSEWHEEL)
    // joystick events
    TO_STRING_CLAUSE(SDL_JOYAXISMOTION)
    TO_STRING_CLAUSE(SDL_JOYBALLMOTION)
    TO_STRING_CLAUSE(SDL_JOYHATMOTION)
    TO_STRING_CLAUSE(SDL_JOYBUTTONDOWN)
    TO_STRING_CLAUSE(SDL_JOYBUTTONUP)
//    TO_STRING_CLAUSE(SDL_JOYDEVICEADDED)
//    TO_STRING_CLAUSE(SDL_JOYDEVICEREMOVED)
    // controller events
//    TO_STRING_CLAUSE(SDL_CONTROLLERAXISMOTION)
//    TO_STRING_CLAUSE(SDL_CONTROLLERBUTTONDOWN)
//    TO_STRING_CLAUSE(SDL_CONTROLLERBUTTONUP)
//    TO_STRING_CLAUSE(SDL_CONTROLLERDEVICEADDED)
//    TO_STRING_CLAUSE(SDL_CONTROLLERDEVICEREMOVED)
//    TO_STRING_CLAUSE(SDL_CONTROLLERDEVICEREMAPPED)
    default:
        return "Other";
    }
#undef TO_STRING_CLAUSE
}

class TeleopNode
{
public:

    TeleopNode() : m_nh() { }

    ~TeleopNode()
    {
        SDL_Quit();
    }

    bool initialize()
    {
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK)) {
            fprintf(stderr, "Unable to initialize SDL: %s\n", SDL_GetError());
            SDL_Quit();
            return 1;
        }
    
        ROS_INFO("%d joysticks available", SDL_NumJoysticks());
    
        if (!SDL_NumJoysticks()) {
            fprintf(stderr, "No joysticks available\n");
            SDL_Quit();
            return 1;
        }

        SDL_Joystick* joy = SDL_JoystickOpen(0);
        ROS_INFO("Opened joystick 0 (%s)", SDL_JoystickName(0));
        ROS_INFO(" -> %d axes", SDL_JoystickNumAxes(joy));
        ROS_INFO(" -> %d balls", SDL_JoystickNumBalls(joy));
        ROS_INFO(" -> %d hats", SDL_JoystickNumHats(joy));
        ROS_INFO(" -> %d buttons", SDL_JoystickNumButtons(joy));

        m_prev_axis_values.resize(SDL_JoystickNumAxes(joy), 0);

        int16_t db_off = (int16_t)(0.05 * (double)std::numeric_limits<uint16_t>::max());
        const int16_t db_min = std::numeric_limits<int16_t>::min() + db_off;
        const int16_t db_max = std::numeric_limits<int16_t>::max() - db_off;
        m_db_mins.resize(SDL_JoystickNumAxes(joy), db_min);
        m_db_maxs.resize(SDL_JoystickNumAxes(joy), db_max);
        ROS_INFO("Deadbands at %d and %d", db_min, db_max);

        const int16_t capture_min = -(db_off);// >> 1);
        const int16_t capture_max =  (db_off);// >> 1);
        ROS_INFO("Capture region around 0: [%d, %d]", capture_min, capture_max);
        m_db_zero_mins.resize(SDL_JoystickNumAxes(joy), capture_min);
        m_db_zero_maxs.resize(SDL_JoystickNumAxes(joy), capture_max);

        m_vel_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        m_min_vel_mps = 0.0; // not sure how to define these quite yet..via another deadband?
        m_min_rot_vel_rps = 0.0;

        m_max_vel_mps = 0.6;
        m_max_rot_vel_rps = 0.6;
        return true;
    }

    int run()
    {
        ros::Rate loop_rate(60.0);
        while (ros::ok()) {
            // process all pending events
            SDL_Event event;

            int num_processed = 0;
            while (SDL_PollEvent(&event))
            {
                ROS_DEBUG("Received event of type %s", to_cstr((SDL_EventType)event.type));
                switch (event.type) {
                case SDL_QUIT:
                    ros::shutdown();
                    break;
                case SDL_JOYAXISMOTION:
                {
                    SDL_JoyAxisEvent& axis_event = event.jaxis;

//                    ROS_INFO(" -> which: %u", (unsigned int)axis_event.which);
                    ROS_DEBUG("axis = %u", (unsigned int)axis_event.axis);
                    ROS_DEBUG("value = %d", (int)axis_event.value);

                    // 1. adjust this value as per the deadbands
                    int16_t adjusted = adjust(axis_event);
                    ROS_DEBUG("adjusted = %d", (int)adjusted);

                    if (adjusted != m_prev_axis_values[axis_event.axis]) {
                        handle_jaxis_event(axis_event.axis, adjusted);
                    }
                    // else, ignore the event
                }   break;
                case SDL_JOYBALLMOTION:
                {
//                    SDL_JoyBallEvent& ball_event = event.jball;
                }   break;
                case SDL_JOYHATMOTION:
                {
//                    SDL_JoyHatEvent& hat_event = event.jhat;
                }   break;
                case SDL_JOYBUTTONDOWN:
                case SDL_JOYBUTTONUP:
                {
//                    SDL_JoyButtonEvent& button_event = event.jbutton;
                }   break;
                default:
                    break;
                }
                ++num_processed;
            }

            ROS_DEBUG("Processed %d SDL events", num_processed);
            ROS_DEBUG("Current Axis State: %s", to_string(m_prev_axis_values).c_str());
            m_vel_pub.publish(m_next_cmd);

            loop_rate.sleep();
        }

        return 0;
    }

private:

    ros::NodeHandle m_nh;
    ros::Publisher m_vel_pub;

    double m_min_vel_mps;
    double m_max_vel_mps;
    double m_min_rot_vel_rps;
    double m_max_rot_vel_rps;

    std::vector<int16_t> m_prev_axis_values; // adjusted values
    std::vector<int16_t> m_db_mins;
    std::vector<int16_t> m_db_maxs;
    std::vector<int16_t> m_db_zero_mins;
    std::vector<int16_t> m_db_zero_maxs;

    int16_t m_dband_lo_a0;
    int16_t m_dband_hi_a0;

    geometry_msgs::Twist m_next_cmd;

    int16_t adjust(const SDL_JoyAxisEvent& jaxis)
    {
        // [ min, db_min, db_min_0, db_max_0, db_max, max ]

        // adjust this value so that:
        //     values less than db_min return min
        //     values more than db_max return max
        //     values in between db_min and db_max return properly scaled values
        //         between min and max

        if (jaxis.value <= m_db_mins[jaxis.axis]) {
            return std::numeric_limits<int16_t>::min();
        }

        if (jaxis.value >= m_db_maxs[jaxis.axis]) {
            return std::numeric_limits<int16_t>::max();
        }

        if (jaxis.value >= m_db_zero_mins[jaxis.axis] &&
            jaxis.value <= m_db_zero_maxs[jaxis.axis])
        {
            return 0;
        }

        const uint16_t zero_db_width =
                (uint16_t)((int32_t)m_db_zero_maxs[jaxis.axis] -
                           (int32_t)m_db_zero_mins[jaxis.axis]);

        const int16_t eff_min = m_db_mins[jaxis.axis];
        const int16_t eff_max = m_db_maxs[jaxis.axis];
        const uint16_t eff_width =
                (uint16_t)((int32_t)eff_max - (int32_t)eff_min) - zero_db_width;

        const uint16_t eff_off = (uint16_t)((int32_t)jaxis.value - (int32_t)eff_min);

        // min + (eff_off / eff_width) * width;
        const int16_t abs_min = std::numeric_limits<int16_t>::min();
        return abs_min + (eff_off * std::numeric_limits<uint16_t>::max() / eff_width);
    }

    void handle_jaxis_event(uint8_t axis, int16_t adjusted)
    {
        if (axis == 0) {
            double pct = (double)adjusted / (double)std::numeric_limits<int16_t>::max();
            // note negative here so that "stick right" => "turn right"
            m_next_cmd.angular.z = -pct * m_max_rot_vel_rps;
        }

        if (axis == 1) {
            // note negative here so that "stick up" => "drive forward"
            double pct = (double)adjusted / (double)std::numeric_limits<int16_t>::max();
            m_next_cmd.linear.x = -pct * m_max_vel_mps;
        }

        m_next_cmd.linear.y = m_next_cmd.linear.z = 0.0;
        m_next_cmd.angular.x = m_next_cmd.angular.y = 0.0;

        m_prev_axis_values[axis] = adjusted;
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "teleop_joystick");

    TeleopNode node;
    if (!node.initialize())
    {
        ROS_ERROR("Failed to initialize Teleop Node");
        return 1;
    }

    return node.run();
}
