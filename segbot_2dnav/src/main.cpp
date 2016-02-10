/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <stdio.h>
#include <string.h>

#include <getopt.h>
#include <pr2_controller_manager/controller_manager.h>
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

std::string g_robot_desc;

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

static inline double now()
{
    struct timespec n;
    clock_gettime(CLOCK_MONOTONIC, &n);
    return double(n.tv_nsec) / NSEC_PER_SEC + n.tv_sec;
}

static void timespecInc(struct timespec &tick, int nsec)
{
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= NSEC_PER_SEC) {
        tick.tv_nsec -= NSEC_PER_SEC;
        tick.tv_sec++;
    }
}

void* controlLoop(void *)
{
    int rv = 0;

    int period;
    int policy;

    TiXmlElement *root;
    TiXmlElement *root_element;

    // Initialize fake hardware interface
    pr2_hardware_interface::HardwareInterface hw;
    hw.addActuator(new pr2_hardware_interface::Actuator("segway_left_wheel_motor"));
    hw.addActuator(new pr2_hardware_interface::Actuator("segway_right_wheel_motor"));
    hw.addActuator(new pr2_hardware_interface::Actuator("segway_trailer_wheel_caster_motor"));
    hw.addActuator(new pr2_hardware_interface::Actuator("segway_trailer_wheel_motor"));
    hw.addActuator(new pr2_hardware_interface::Actuator("R283_hokuyo_laser1_motor"));
    hw.addActuator(new pr2_hardware_interface::Actuator("R313_hokuyo_laser2_motor"));

    // Create controller manager
    pr2_controller_manager::ControllerManager cm(&hw);

    // Load robot description
    TiXmlDocument xml;
    struct stat st;
    if (0 == stat(g_options.xml_, &st)) {
        xml.LoadFile(g_options.xml_);
    }
    else {
        ROS_INFO("Xml file not found, reading from parameter server");
        ros::NodeHandle top_level_node;
        if (top_level_node.getParam(g_options.xml_, g_robot_desc)) {
            xml.Parse(g_robot_desc.c_str());
        }
        else {
            ROS_FATAL("Could not load the xml from parameter server: %s", g_options.xml_);
            rv = -1;
            goto end;
        }
    }
    root_element = xml.RootElement();
    root = xml.FirstChildElement("robot");
    if (!root || !root_element) {
        ROS_FATAL("Could not parse the xml from %s", g_options.xml_);
        rv = -1;
        goto end;
    }

    // Initialize the controller manager from robot description
    if (!cm.initXml(root)) {
        ROS_FATAL("Could not initialize the controller manager");
        rv = -1;
        goto end;
    }

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

    //last_published = now();
    while (ros::ok()) {
        cm.update();
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
