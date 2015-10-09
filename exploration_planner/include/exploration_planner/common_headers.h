#ifndef _COMMON_HEADERS_JMB
#define _COMMON_HEADERS_JMB
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <cstdio>
#include <vector>
#include <deque>
#include <cstring>
#include <time.h>
#include <queue>
#include <limits>
#include <fstream>
#include <string>

#include "stdint.h" // only for XP MS C++
using namespace std;

#include "messages_IPC.h"
#include "map_globals.h"
#include "global_planner.h"
//#include "raycaster.h"

//extern int repetition;
//
//struct planner_options { 
//    float plan_time; // time to run additional paths (0 for fixed)
//    int inc_full; // 1 for incremental, 2 for full (3 for testing with both)
//    int max_paths; // maximum number of paths to run
//    int traj_base; // 1 for A* on random points, 2 for A* to closest unknown point
//    float dist_before_plan; // max distance to move before replanning 
//    
//    planner_options() {
//        plan_time = 0;
//        inc_full = 1;
//        max_paths = 250;
//        traj_base = 1;
//        dist_before_plan = 0;
//    }
//};

//#define _DEBUGLVL_JMB 5



#endif
