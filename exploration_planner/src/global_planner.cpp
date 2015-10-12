#include <exploration_planner/common_headers.h>
using namespace std;

////#include "astarpoint.h"
//#include "ipc.h"
#include <exploration_planner/filetransfer.h>
#include <sbpl/headers.h>
//#include "/home/jon/ros/cturtle/stacks/motion_planners/sbpl/src/sbpl/headers.h" // SBPL
//#include "/home/jon/ros/pkgs/motion_planners/sbpl/src/sbpl/headers.h"

#define MODULE_NAME "Global Planner Linux"

//#include "c:\Documents and Settings\Jon\My Documents\Schoolwork\research\SBPL\src\sbpl\headers.h" // SBPL
//#include "C:\Documents and Settings\jbutzke\My Documents\Sarnoff Project Global Planner\IPC_version\headers.h"
//#define MODULE_NAME "Global Planner Windows"

using namespace std;
// Module written by Jonathan Michael Butzke
//	serves as the main subroutine to determine a path.
/*
 int map_sizex_m=0;
 int map_sizey_m=0;
 float coverage_cell_size=0;
 float cost_cell_size=0;
 float elev_cell_size=0;
 int coverage_size_x=0;
 int coverage_size_y=0;
 int cost_size_x=0;
 int cost_size_y=0;
 int elev_size_x=0;
 int elev_size_y=0;

 //map variables
 unsigned char * cover_map = new unsigned char[1];
 unsigned char * cost_map = new unsigned char[1];
 int16_t * elev_map = new int16_t[1];

 unsigned char * inflated_cost_map = new unsigned char[1];
 unsigned char ** cost_map_pa = new unsigned char*[1]; // ptr to first element of each row
 unsigned char ** inf_cost_map_pa = new unsigned char*[1]; //ptr to first element of each row for inflated map

 priority_queue<frontier_pts, vector<frontier_pts>, fp_compare> frontier;
 vector<Traj_pt_s> traj; // trajectory
 GP_TRAJECTORY gp_traj; // message holder

 float robot_xx=1, robot_yy=1; // float posit of robot
 int robot_x=1, robot_y=1; // x and y cell coordinates of robot
 float theta = 0;

 //robot variables
 float sensor_radius= 10.0; // distance sensors can see in m
 int16_t sensor_height=120; // sensor height in cm
 float SENSORWIDTH = 70*M_PI/180;  // sensor width in radians used to determine start and finish vectors for ray tracing

 float inflation_size=0;  //size in cells to inflate obstacles
 float MAX_VELOCITY=5.0; // meters per second
 float MAX_TURN_RATE=1.0; // radians per second
 float GP_PLAN_TIME=5.0; // seconds to allow for planning
 int HIGH_IG_THRES = 50000; // set by initialization and full map updates
 float IG_RATIO = 0.01; // ratio of total possible unknown to remaining unknown before switching to pure greedy search
 float LR_MARGIN = 10; // amount one side must be greater to influence pan angles
 float DIST_GAIN = 0.1; // factor to switch between greedy and IG
 int CELLS_PER_SEC = 10*10*M_PI*20*KNOWN/360; // number of cells that can be swept out per second
 float VIEW_PROB_FACTOR = 0.3; // retards calculated speed based on likelihood of observation
 bool WRITE_FILES = true; // flag to write files out
 bool DISPLAY_OUTPUT = true; // flag to display any output

 // sensor variables
 int NUMVECTORS = 0;
 vector<RAY_TEMPLATE_PT> rayendpts;
 int SVL[9], SVR[9], FVL[9], FVR[9];  // holds the start and finish vectors for the right and left 120 arcs
 */

GPLAN::GPLAN() {
	map_sizex_m = 0;
	map_sizey_m = 0;
	coverage_cell_size = 0;
	cost_cell_size = 0;
	elev_cell_size = 0;
	coverage_size_x = 0;
	coverage_size_y = 0;
	cost_size_x = 0;
	cost_size_y = 0;
	elev_size_x = 0;
	elev_size_y = 0;

	//map variables
	cover_map = new unsigned char[1];
	cost_map = new unsigned char[1];
	elev_map = new int16_t[1];

	inflated_cost_map = new unsigned char[1];
	cost_map_pa = new unsigned char*[1]; // ptr to first element of each row
	inf_cost_map_pa = new unsigned char*[1]; //ptr to first element of each row for inflated map

	//	priority_queue<frontier_pts, vector<frontier_pts>, fp_compare> frontier;
	//	vector<Traj_pt_s> traj; // trajectory
	//	GP_TRAJECTORY gp_traj; // message holder

	robot_xx = 1;
	robot_yy = 1; // float posit of robot
	robot_x = 1;
	robot_y = 1; // x and y cell coordinates of robot
	theta = 0;

	//robot variables
	sensor_radius = 10.0; // distance sensors can see in m
	sensor_height = 120; // sensor height in cm
	SENSORWIDTH = (float)(240 * M_PI / 180); // sensor width in radians used to determine start and finish vectors for ray tracing

	inflation_size = 0; //size in cells to inflate obstacles
	MAX_VELOCITY = 5.0; // meters per second
	MAX_TURN_RATE = 1.0; // radians per second
	GP_PLAN_TIME = 1.0; // seconds to allow for planning
	HIGH_IG_THRES = 50000; // set by initialization and full map updates
	IG_RATIO = (float).01; // ratio of total possible unknown to remaining unknown before switching to pure greedy search
	LR_MARGIN = 10; // amount one side must be greater to influence pan angles
	DIST_GAIN = .6; // factor to switch between greedy and IG
	CELLS_PER_SEC = (int)(10.0 * 10.0 * M_PI * 20.0 * KNOWN / 360.0); // number of cells that can be swept out per second
	VIEW_PROB_FACTOR = (float)0.3; // retards calculated speed based on likelihood of observation
	WRITE_FILES = true; // flag to write files out
	DISPLAY_OUTPUT = true; // flag to display any output
	THETA_BIAS = 1;// can not equal zero
	// sensor variables
	NUMVECTORS = 0;
	//	vector<RAY_TEMPLATE_PT> rayendpts;
	//	int SVL[9], SVR[9], FVL[9], FVR[9];  // holds the start and finish vectors for the right and left 120 arcs
	gp_traj.traj_array = new float[1];
}

void GPLAN::setPixel(int x, int y) {
	unsigned int locptr = 0;
	//cout << " ray pt " << x << "," << y << endl;
	float tempangle = atan2((float) y, (float) x);
	if (tempangle < 0) {
		tempangle += (float)(2.0 * M_PI);
	}

	if (rayendpts.size() == 0) {
		rayendpts.push_back(RAY_TEMPLATE_PT(x, y, tempangle));
	} else {
		while ((locptr < rayendpts.size()) && (rayendpts[locptr].angle
				<= tempangle)) {
			locptr++;
		}
		if (locptr == 0) {
			rayendpts.insert(rayendpts.begin(),
					RAY_TEMPLATE_PT(x, y, tempangle));
		} else if (rayendpts[locptr - 1].angle != tempangle) {
			if (locptr == rayendpts.size()) {
				rayendpts.push_back(RAY_TEMPLATE_PT(x, y, tempangle));
			} else {
				rayendpts.insert(rayendpts.begin() + locptr, RAY_TEMPLATE_PT(x,
						y, tempangle));
			}
		}
	}
}

void GPLAN::rasterCircle(int radius) {
	// "Midpoint circle algorithm." Wikipedia, The Free Encyclopedia. 6 Jul 2009, 03:19 UTC. 6 Jul 2009 <http://en.wikipedia.org/w/index.php?title=Midpoint_circle_algorithm&oldid=300524864>.

	// centerpoint and radius are inputs

	int f = 1 - radius;
	int ddF_x = 1;
	int ddF_y = -2 * radius;
	int x = 0;
	int y = radius;

	rayendpts.clear();

	setPixel(0, radius);
	setPixel(0, -radius);
	setPixel(radius, 0);
	setPixel(-radius, 0);
	setPixel((int) (sqrt(.5) * radius), (int) (sqrt(.5) * radius));
	setPixel((int) (sqrt(.5) * radius), -(int) (sqrt(.5) * radius));
	setPixel(-(int) (sqrt(.5) * radius), (int) (sqrt(.5) * radius));
	setPixel(-(int) (sqrt(.5) * radius), -(int) (sqrt(.5) * radius));

	while (x < y) {
		// ddF_x == 2 * x + 1;
		// ddF_y == -2 * y;
		// f == x*x + y*y - radius*radius + 2*x - y + 1;
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		setPixel(x, y);
		setPixel(-x, y);
		setPixel(x, -y);
		setPixel(-x, -y);
		setPixel(y, x);
		setPixel(-y, x);
		setPixel(y, -x);
		setPixel(-y, -x);
	}
	NUMVECTORS = rayendpts.size();
	cout << "Number of rays in template: " << NUMVECTORS << endl;
}

int GPLAN::ValidVec(int vec) {
	// verifies that the vector is within limits
	if (vec < 0) {
		vec = NUMVECTORS + vec;
	} else if (vec >= NUMVECTORS) {
		vec = vec - NUMVECTORS;
	}
	return vec;
}

bool GPLAN::OnMap(int x, int y) {
	// function to determine if a point is on the map
	if ((x < cost_size_x) && (x >= 0) && (y < cost_size_y) && (y >= 0)) {
		return true;
	} else {
		return false;
	}
}

double GPLAN::return_path(int x_target, int y_target, const int dijkstra[],
		vector<Traj_pt_s> & traj) {
	// function to return the optimal path to a target location given a dijkstra map
	//cout << "start return path " << endl;
	Traj_pt_s current;
	vector<Traj_pt_s> inv_traj;
	int x_val, y_val, best_x_val = 0, best_y_val = 0;
	double cost = 0;
	x_val = current.x = x_target;
	y_val = current.y = y_target;

	inv_traj.clear();
	inv_traj.push_back(current);

	int min_val;
	double temp_cost;
	if (dijkstra[x_val + cost_size_x * y_val] < DIJKSTRA_LIMIT) {
		while ((x_val != robot_x) || (y_val != robot_y)) {
			min_val = DIJKSTRA_LIMIT + 10;// 90000000;
			temp_cost = 0;
			//printf("%d, %d - ", x_val, y_val);
			for (int x = -1; x < 2; x++) {
				for (int y = -1; y < 2; y++) {
					if (OnMap(x + x_val, y + y_val) && !((x == 0) && (y == 0))) {
						int val = dijkstra[(x + x_val) + (cost_size_x * (y
								+ y_val))];
						if (val < min_val) {
							min_val = val;
							best_y_val = y_val + y;
							best_x_val = x_val + x;
							temp_cost = stepcost[x + 1][y + 1];
						} // if val
					} // if onmap
				} // y
			} //x

			cost += temp_cost;
			current.x = x_val = best_x_val;
			current.y = y_val = best_y_val;
			inv_traj.push_back(current);
		}
		//cout << " invert " << endl;
		// invert the trajectory to send back
		traj.clear();
		while (inv_traj.size() != 0) {
			current.x = inv_traj.back().x;
			current.y = inv_traj.back().y;
			traj.push_back(current);
			inv_traj.pop_back();
			//printf("path %d, %d\n", current.x, current.y);
		}
		return (cost * cost_cell_size);
	} else {
		printf("WARNING: Invalid goal location - %i %i\n", x_target, y_target);
		return (-1);
	}
	//cout << "done return path " << endl;
}

bool GPLAN::map_alloc(void) {
	//remove old storage
	//printf("start del "); fflush(stdout);
	delete[] cover_map;
	delete[] cost_map;
	delete[] elev_map;
	delete[] inflated_cost_map;
	delete[] cost_map_pa;
	delete[] inf_cost_map_pa;

	//printf("done del "); fflush(stdout);

	//allocate new storage
	cover_map = new unsigned char[coverage_size_x * coverage_size_y];
	cost_map = new unsigned char[cost_size_x * cost_size_y];
	elev_map = new int16_t[elev_size_x * elev_size_y];
	inflated_cost_map = new unsigned char[cost_size_x * cost_size_y];

	//printf("done alloc "); fflush(stdout);
	// set up pointer to array of pointers for [][] indexing of cost map
	cost_map_pa = new unsigned char*[cost_size_y];
	inf_cost_map_pa = new unsigned char*[cost_size_y];
	for (int j = 0; j < cost_size_y; j++) {
		cost_map_pa[j] = &cost_map[j * cost_size_x];
		inf_cost_map_pa[j] = &inflated_cost_map[j * cost_size_x];
	}

//	printf("done pa ");
	//fflush(stdout);
	// false if any map is null
	if ((cover_map != NULL) && (cost_map != NULL) && (elev_map != NULL)
			&& (inflated_cost_map != NULL) && (cost_map_pa != NULL)
			&& (inf_cost_map_pa != NULL)) {
		return true;
	} else {
		return false;
	}
}

//bool OnFrontier(int x, int y) {
//    for (int j=-1; j<2;j++) {
//        for (int i=-1; i<2; i++) {
//            if (cover_map[i+coverage_size_x*j]==UNKNOWN) { return true; }
//        }
//    }
//    return false;
//}

void GPLAN::sample_point(int &x_target, int &y_target, const int dijkstra[],
		const unsigned char temp_cover_map[], const unsigned int IG_map[]) {
	// fxn to pick possible target points
	// 	// convolve map to get gain from each point
	// combine maps to get benefit from each point + cost
	//cout << "      start sample_point" << endl;
//	static int prev_x = 0, prev_y = 0;
//	cout << "  start sample point ";
	x_target = -1;
	y_target = -1;
	//cout << "       Frontier is empty " << endl;
	//while (!frontier.empty()) {
	if(!frontier.empty()) {	
	x_target = frontier.top().x;
		y_target = frontier.top().y;
		//cout << x_target << "," << y_target << " I:" << frontier.top().IG << " C:" << frontier.top().cost << " T:" << frontier.top().total << endl;
		//    printf(" %i %i %i %i %i\n", cover_map[x_target + coverage_size_x*y_target],
		//    cover_map[(x_target+1) + coverage_size_x*(y_target)],
		//    cover_map[(x_target) + coverage_size_x*(y_target+1)],
		//    cover_map[(x_target-1) + coverage_size_x*(y_target)],
		//    cover_map[(x_target) + coverage_size_x*(y_target-1)]);
		frontier.pop();
		//if ((pow((float) (prev_x - x_target), 2) + pow((float) (prev_y
		//		- y_target), 2)) > 10) {
		//	prev_x = x_target;
		//	prev_y = y_target;
		//	break;
		//}

	}
	//cout << "end" << endl;
}

void GPLAN::goto_nearest(int &x_target, int &y_target, const int dijkstra[],
		const unsigned char temp_cover_map[]) {
	//fxn finds the nearest unknown cell and sets it as the target
	static int prev_x = -2, prev_y = -2;

	//find nearest point that is unexplored
	if (prev_x != -1) {
		Astarnearest(robot_x, robot_y, x_target, y_target, inflated_cost_map,
				temp_cover_map);
	}
	int i = 0, j = 0, count = 0, sign = 1, dir = 0, total = 1;
	if (DISPLAY_OUTPUT) {printf(" prelim target is %i %i\n", x_target, y_target);}
	if ((x_target == -1) && (y_target == -1)) {
		if (DISPLAY_OUTPUT) {printf("No valid target cells from Astarnearest\n");}
		prev_x = -1;
		prev_y = -1;
		return;
	}
	// find nearest accessable point to desired location
	while ((prev_x != x_target) || (prev_y != y_target)) {
		if (OnMap(i + x_target, j + y_target)) {
			if (dijkstra[(i + x_target) + cost_size_x * (j + y_target)]
					< DIJKSTRA_LIMIT) {
				prev_x = x_target;
				prev_y = y_target;
				x_target += i;
				y_target += j;
				if (DISPLAY_OUTPUT) {printf("nearest target is %i %i\n", x_target, y_target);}
				return;
			}
		}
		if (count == total) {
			count = 0;
			if (dir == 0) {
				dir = 1;
			} else {
				dir = 0;
				total++;
				sign *= -1;
			}
		}
		if (dir == 0) {
			i += sign;
			count++;
		} else {
			j += sign;
			count++;
		}
		if (i > cost_size_x) {
			printf("     No more unexplored area");
			x_target = -1;
			y_target = -1;
			return;
		}
	}
	// if the target is the same,  function exits
	// add small random offset to target
	x_target += rand() % 5 + rand() % 5 - 4;
	y_target += rand() % 5 + rand() % 5 - 4;
	while(!OnMap(x_target, y_target)) {
		x_target += rand() % 5 + rand() % 5 - 4;
		y_target += rand() % 5 + rand() % 5 - 4;
	}

	if (DISPLAY_OUTPUT) {cout << "going for " << x_target << " " << y_target << endl;}
	return;
}
/*



 // if the target point is the same as the previous run, select random points
 cout << "      randomly selecting next point" << endl;

 //random picks
 x_target = y_target = -2;

 while ((!OnMap(x_target, y_target)) || (dijkstra[x_target+cost_size_x*y_target]>=DIJKSTRA_LIMIT)) {
 x_target = rand() % cost_size_x;
 y_target = rand() % cost_size_y;
 }

 cout << "     End sample_point  -   next target is: " << x_target << "," << y_target << " val: " << (int) (dijkstra[x_target+cost_size_x*y_target]==DIJKSTRA_LIMIT) << endl;  */

bool GPLAN::calc_all_IG(unsigned int IG_map[]) {
	// function calculates the IG for each point on the coverage map and updates IG_map

	//initialize bottom row to 0's
	for (int i = 0; i < coverage_size_x; i++) {
		IG_map[i] = 0;
	}

	//initialize x=0 edge to 0's
	for (int j = 0; j < coverage_size_y; j++) {
		IG_map[coverage_size_x * j] = 0;
	}

	unsigned int lowest_IG = (int)1e7;
    unsigned int highest_IG = 0;
    unsigned int highx = 0;
    unsigned int highy = 0;
//    unsigned int lowx = 0;
//    unsigned int lowy = 0;

	//calc remaining values
	for (int j = 1; j < coverage_size_y; j++) {
		for (int i = 1; i < coverage_size_x; i++) {
			if (cost_map[i + cost_size_x * j] == OBSTACLE) {
				IG_map[i + coverage_size_x * j] = IG_map[(i - 1)
						+ coverage_size_x * j] + IG_map[i + coverage_size_x
						* (j - 1)]
						- IG_map[(i - 1) + coverage_size_x * (j - 1)];
			} else {
				IG_map[i + coverage_size_x * j] = IG_map[(i - 1)
						+ coverage_size_x * j] + IG_map[i + coverage_size_x
						* (j - 1)]
						- IG_map[(i - 1) + coverage_size_x * (j - 1)]
						- (int) cover_map[i + coverage_size_x * j] + KNOWN;
			}
			//cout << IG_map[i+coverage_size_x*j] << "::" << i << "," << j << endl;
			if (IG_map[i + coverage_size_x * j] < lowest_IG) {
				lowest_IG = IG_map[i + coverage_size_x * j];
//				lowx = i;
//				lowy = j;
			}
			if (IG_map[i + coverage_size_x * j] > highest_IG) {
				highest_IG = IG_map[i + coverage_size_x * j];
				highx = i;
				highy = j;
//				printf("hig %d %d %d\n", highest_IG, highx, highy);
			}
		}
	}
	if (DISPLAY_OUTPUT) {cout << " highest IG is " << highest_IG << " at " << highx << "," << highy
		<< " of " << HIGH_IG_THRES << endl;}

	if ((int)highest_IG < HIGH_IG_THRES) {
		return false;
	} else
		return true;

}

unsigned int GPLAN::get_IG(unsigned int IG_map[], int x, int y, int dim) {
	// function returns the IG in a box +/- dim from point (x,y)

	int t, r, l, b; // top right left and bottom dimensions
	t = min((coverage_size_y - 1), y + dim);
	b = max(0, y - dim);
	l = max(0, x - dim);
	r = min((coverage_size_x - 1), x + dim);

	return (IG_map[r + coverage_size_x * t] - IG_map[r + coverage_size_x * b]
			+ IG_map[l + coverage_size_x * b] - IG_map[l + coverage_size_x * t]);
}

double GPLAN::heading_bias(int x, int y) {
	// fxn to calc a heading bias to prefer points closer to "straight ahead"straight ahead and 0 being straight behind
	//returns a value between 0 and 1 with 1 being 

	double theta_pt = atan2((double)(y-robot_y), (double)(x-robot_x));
	double theta_diff = theta_pt - theta;

	//normalize between -pi and pi
	if (theta_diff > M_PI) { theta_diff -= 2*M_PI;}
	if (theta_diff < -M_PI) { theta_diff += 2*M_PI;}


//printf("Theta penalty for (%i, %i) with robot at (%i, %i, %f) is %f or %f\n", x, y, robot_x, robot_y, theta, (1-(abs(theta_diff)/(M_PI))), ((1+cos(theta_diff))/2) );
	//return (1-(abs(theta_diff)/(M_PI)));
return ((1.0+cos(THETA_BIAS*theta_diff))/2.0);
}




void GPLAN::find_frontier(unsigned int IG_map[], int dijkstra[]) {
	// function scans coverage map and populates the frontier queue with frontier points
	while (!frontier.empty()) {
		frontier.pop();
	}
	for (int j = 1; j < coverage_size_y - 1; j++) {
		for (int i = 1; i < coverage_size_x - 1; i++) {
			if (dijkstra[i + coverage_size_x * j] < DIJKSTRA_LIMIT) {
				if ((cover_map[i + coverage_size_x * j] == KNOWN)
						&& ((cover_map[(i + 1) + coverage_size_x * (j)] != KNOWN) 
						|| (cover_map[(i - 1) + coverage_size_x * (j)] != KNOWN)
						|| (cover_map[(i) + coverage_size_x * (j + 1)] != KNOWN) 
						|| (cover_map[(i) + coverage_size_x * (j - 1)] != KNOWN))) {
							frontier_pts temp(i, j, heading_bias(i,j)*IG_map[i + coverage_size_x * j],
								dijkstra[i + coverage_size_x * j], DIST_GAIN);
							frontier.push(temp);
					//cout << i << "," << j << " is on the frontier " << endl;
				}
			}
		}
	}
}
//
//void denoise(unsigned char src[], int strel, int mapm, int mapn) {
//	// performs morphological open
//	int xx = strel / 2;
//	int yy = strel / 2;
//
//	unsigned char *temp = new unsigned char[mapm * mapn];
//	unsigned char *dest = new unsigned char[mapm * mapn];
//
//	for (int j = 0; j < mapn; j++) {
//		for (int i = 0; i < mapm; i++) {
//			temp[i + j * mapm] = UNKNOWN;
//			dest[i + j * mapm] = KNOWN;
//		}
//	}
//
//	//perform dilation of known areas
//	for (int j = 0; j < mapn; j++) {
//		for (int i = 0; i < mapm; i++) {
//			if (src[i + mapm * j] != UNKNOWN) {
//				for (int x = 0; x < strel; x++) {
//					for (int y = 0; y < strel; y++) {
//						if (OnMap(x + i - xx, y + j - yy)) {
//							temp[x + i - xx + (y + j - yy) * mapm] = KNOWN;
//						}
//					}
//				}
//			}
//		}
//	}
//
//	// perform erosion on temp map to return known areas to size
//	for (int j = 0; j < mapn; j++) {
//		for (int i = 0; i < mapm; i++) {
//			if (temp[i + mapm * j] == UNKNOWN) {
//				for (int x = 0; x <= strel; x++) {
//					for (int y = 0; y <= strel; y++) {
//						if (OnMap(x + i - xx, y + j - yy)) {
//							dest[x + i - xx + (y + j - yy) * mapm] = UNKNOWN;
//						}
//					}
//				}
//			}
//		}
//	}
//
//	// copy the dest array back onto the original src array
//	memcpy((void *)src,(void *) dest, sizeof(unsigned char)*mapm*mapn);
//	delete[] dest;
//	delete[] temp;
//
//}

//void print_local(int x, int y, int window) {
//    // function to print small local map of given location
//         printf("\n");
//
//    for(int j=y-window; j< y+window; j++) {
//        for(int i = x-window; i< x+window; i++) {
//            if ((i==x)&&(j==y)) {printf("XXXX "); }
//            else {printf("%04i ", cover_map[i+coverage_size_x*j]);//, inflated_cost_map[i+cost_size_x*j]);
//            }
//        }
//        printf("\n");
//    }
//}

void GPLAN::global_planner(float goal_x, float goal_y, float goal_theta) {
	// function provides a traj to best found goal point to (nearest or most valuable)
	printf("Started exploration planner\n");
	//fflush(stdout);

	float *obs_array = new float[cost_size_x * cost_size_y];
	float **obs_ptr_array = new float*[cost_size_y];
	float *nonfree_array = new float[cost_size_x * cost_size_y];
	float **nonfree_ptr_array = new float*[cost_size_y];

	unsigned int * IG_map = new unsigned int[coverage_size_x * coverage_size_y];

	//calculate the IG at each point based on current coverage map
	bool IG_above_thres;
	IG_above_thres = calc_all_IG(IG_map);

	// set up array of pointers
	for (int j = 0; j < cost_size_y; j++) {
		obs_ptr_array[j] = &obs_array[j * cost_size_x];
		nonfree_ptr_array[j] = &nonfree_array[j * cost_size_x];
	}

	// remove obstacles from unknown areas
	for (int j = 0; j < cost_size_y; j++) {
		for (int i = 0; i < cost_size_x; i++) {
			if (cover_map[i + coverage_size_x * j] == UNKNOWN) {
				elev_map[i + coverage_size_x * j] = -OBS16;
				cost_map[i + coverage_size_x * j] = 0;
			}
		}
	}

	// inflate map
	computeDistancestoNonfreeAreas(cost_map_pa, cost_size_y, cost_size_x, OBSTACLE, obs_ptr_array, nonfree_ptr_array);
	int dim = (int)(sensor_radius / (2.0 * cost_cell_size));
	//update inflated map based on robot size and make unknown areas obstacles w/o inflation
	for (int j = 0; j < cost_size_y; j++) {
		for (int i = 0; i < cost_size_x; i++) {
			if (((!UNKNOWN_ARE_PASSABLE) && (cover_map[i + cost_size_x * j]
					== UNKNOWN)) || (obs_ptr_array[j][i] <= inflation_size)) {
				if ((!UNKNOWN_ARE_PASSABLE) && (cover_map[i + cost_size_x * j]
						== UNKNOWN)) {
					inflated_cost_map[i + cost_size_x * j] = UNKOBSTACLE;
				}
				if (obs_ptr_array[j][i] <= inflation_size) {
					inflated_cost_map[i + cost_size_x * j] = OBSTACLE;
					cover_map[i + coverage_size_x * j] = KNOWN;
				}
			}
			//if(obs_ptr_array[j][i]<=inflation_size) { inflated_cost_map[i+cost_size_x*j] = OBSTACLE; 	}
			else {
		/*		int pad_dist = (int)(3.0/cost_cell_size); 
				int buffer = (pad_dist - (int)inflation_size);
				//printf("%i %i %f\n", pad_dist, buffer, inflation_size);
				inflated_cost_map[i + cost_size_x * j] = (unsigned char) max(0.0, (double)((pad_dist - obs_ptr_array[j][i])*200/buffer));
			*/	//printf("(%4.2f", (double)((pad_dist - obs_ptr_array[j][i])*200/buffer));
			 	inflated_cost_map[i + cost_size_x * j] = (unsigned char) max( 0.0, 
						(double) (cost_map[i + cost_size_x * j]) 
						- (double)(get_IG(IG_map, i, j, dim)) / (4.0 * dim * dim)
						);
//old first arg for above //						(double) inflated_cost_map[i + cost_size_x*j],

				//printf("+%4.2f-%4.2f)=%i\n",
			//			(double) (cost_map[i + cost_size_x * j]), 
			//			 (double)(get_IG(IG_map, i, j, dim)) / (4.0 * dim * dim),
			//			 inflated_cost_map[i + cost_size_x * j] );
			}
		}
	}

	// ensure map boundaries are solid
	for (int i = 0; i < cost_size_x; i++) {
		elev_map[i] = OBS16;
		elev_map[i + cost_size_x * (cost_size_y - 1)] = OBS16;
		cost_map[i] = OBSTACLE;
		cost_map[i + cost_size_x * (cost_size_y -1)] = OBSTACLE;
	}

	for (int j = 0; j < cost_size_y; j++) {
		elev_map[j * cost_size_x] = OBS16;
		elev_map[(cost_size_x - 1) + j * cost_size_x] = OBS16;
		cost_map[j * cost_size_x] = OBSTACLE;
		cost_map[(cost_size_x - 1) + j * cost_size_x] = OBSTACLE;
		}

//	cout << "inflate " << endl;
	// ensure that the robot cell is not an obstacle and clear a little box if there is
	int rad = (int)ceil(inflation_size + 1);
	float rad_2 = pow(inflation_size + 1, 2);
	cover_map[robot_x + coverage_size_x * robot_y] = KNOWN;
	if (cost_map[robot_x + cost_size_x * robot_y] >= OBSTACLE) {
		for (int xxx = -rad; xxx < rad; xxx++) {
			for (int yyy = -rad; yyy < rad; yyy++) {
				if (OnMap(robot_x + xxx, robot_y + yyy) && ((xxx * xxx + yyy
						* yyy) < rad_2)) {
					cost_map[robot_x + xxx + cost_size_x * (robot_y + yyy)] = 0;
          /*
							= min((int) cost_map[robot_x + xxx + cost_size_x
									* (robot_y + yyy)], OBSTACLE - ((rad * rad)
									/ (1 + xxx * xxx + yyy * yyy)));
          */
					cover_map[robot_x + xxx + cost_size_x * (robot_y + yyy)]
							= KNOWN;
				}
			}
		}
	}
	if (inflated_cost_map[robot_x + cost_size_x * robot_y] >= OBSTACLE) {
		for (int xxx = -rad; xxx < rad; xxx++) {
			for (int yyy = -rad; yyy < rad; yyy++) {
				if (OnMap(robot_x + xxx, robot_y + yyy) && ((xxx * xxx + yyy
						* yyy) < rad_2)) {
					inflated_cost_map[robot_x + xxx + cost_size_x * (robot_y
							+ yyy)] = 0; 
          /*
          min((int) inflated_cost_map[robot_x + xxx
							+ cost_size_x * (robot_y + yyy)], OBSTACLE - ((3
							* rad * rad) / (1 + xxx * xxx + yyy * yyy)));
              */
					cover_map[robot_x + xxx + cost_size_x * (robot_y + yyy)] = KNOWN;
				}
			}
		}
	}

	// setup search environment
	SBPL2DGridSearch search(cost_size_y, cost_size_x, cost_cell_size);
	search.setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_HEAP);

	// dijkstra map to get cost to all points
	search.search(inf_cost_map_pa, OBSTACLE, robot_y, robot_x, robot_y + 1,
			robot_x + 1, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);

	// get distance to each accessable point on inflated map
	int * dijkstra = new int[cost_size_x * cost_size_y];
	for (int j = 0; j < cost_size_y; j++) {
		for (int i = 0; i < cost_size_x; i++) {
			dijkstra[i + cost_size_x * j]
					= (int) (search.getlowerboundoncostfromstart_inmm(j, i));
		}
	}
//	cout << "find frontier " << endl;
	find_frontier(IG_map, dijkstra);

	if (DISPLAY_OUTPUT) {printf("Robot pose x=%d xx=%f y=%d yy=%f\n", robot_x, robot_xx, robot_y,
		robot_yy);}

//	for (int qq = -5; qq <= 5; qq++) {
//		for (int ww = -5; ww <= 5; ww++) {
//			printf("%3d/%3d/%3d ", (int) cost_map[robot_x + qq + cost_size_x
//					* (robot_y + ww)], (int) cover_map[robot_x + qq
//					+ cost_size_x * (robot_y + ww)], (int) elev_map[robot_x
//					+ qq + cost_size_x * (robot_y + ww)]);
//		}
//		printf("\n");
//	}

	double best_score = 0; // tracks best score this run
	int x_target, y_target;//, best_x, best_y;
	vector<Traj_pt_s> test_traj; // temp trajectory
	vector<int> traj_score_l; // storage for score at each point along trajectory for post processing
	vector<int> traj_score_r;
	unsigned char * temp_cover_map = new unsigned char[coverage_size_x
			* coverage_size_y]; // non-const storage for each possible goal

	// variable to track time spent planning
	time_t start, finish;
	time(&start);
	time(&finish);

	if (goal_x != -1) {
		// set goal
//		double dist;
		if (DISPLAY_OUTPUT) {
            cout << "determining path to assigned goal" << endl;
        }
		/*dist = */return_path(
                (int) (goal_x / cost_cell_size),
                (int) (goal_y / cost_cell_size),
                dijkstra,
                traj);

	} else {
		//find good point
		//clear old traj
		traj.clear();

//		cout << "start while " << endl;
		while (difftime(finish, start) < GP_PLAN_TIME) { // while less than plan time  (XP should not have 0.5)
			//cout << difftime(finish, start) << " time diff" << endl;
			cout << ".";// << IG_above_thres;
			// temp map for tracking changes during runs
			memcpy((void *) temp_cover_map, (void *) cover_map, coverage_size_x
					* coverage_size_y * (sizeof(unsigned char)));

			// find a good candidate goal point if no goal is sent
			if (!IG_above_thres) {
				goto_nearest(x_target, y_target, dijkstra, temp_cover_map);
			} else {
				sample_point(x_target, y_target, dijkstra, temp_cover_map,
						IG_map);
			}

			if (DISPLAY_OUTPUT) {printf("%d %d is potential goal", x_target, y_target);}

			// if return is -1, -1 then no more points found return null trajectory
			if ((x_target == -1) && (y_target == -1)) {
				cout << " Break from while loop " << endl;
				break;
			}

			//	print_local(x_target, y_target, 5);
			// determine path to each goal point
			double dist;
			dist = return_path(x_target, y_target, dijkstra, test_traj);

			if (DISPLAY_OUTPUT) {printf(" and the distance is %f", dist);}

			// determine gain from each possible goal point
			double temp_score = 0;
			for (int current_loc = 1; current_loc < (int)test_traj.size(); current_loc++) {
				//cout << "test loc " <<  test_traj[current_loc].x << "," << test_traj[current_loc].y << endl;
				// determine the direction of travel in each axis
				int x_dir = test_traj[current_loc].x - test_traj[current_loc
						- 1].x + 1;
				int y_dir = test_traj[current_loc].y - test_traj[current_loc
						- 1].y + 1;
				int direction = dir[x_dir][y_dir];
				if (direction != NOMOVE) {
					// pass current location and inflated map to raycaster returns score
					temp_score += cast_all_rays(test_traj[current_loc].x,
							test_traj[current_loc].y, temp_cover_map, elev_map,
							SVR[direction], FVR[direction]);
					temp_score += cast_all_rays(test_traj[current_loc].x,
							test_traj[current_loc].y, temp_cover_map, elev_map,
							SVL[direction], FVL[direction]);
				} // if !NOMOVE
			} //for current_loc
			if (DISPLAY_OUTPUT) {
				printf(" with a score %f and idx of %f", temp_score, 
					(heading_bias(x_target, y_target)*((temp_score*DIST_GAIN)+1.0)/ (dist*(1.0-DIST_GAIN)+0.1))	); }
			printf(" and a heading bias of %f\n", heading_bias(x_target, y_target));
			// store as traj if best score per distance traveled
			if ((heading_bias(x_target, y_target)*((temp_score*DIST_GAIN)+1.0)/ (dist*(1.0-DIST_GAIN)+0.1)) > best_score) {
				traj.swap(test_traj);
			//traj = test_traj;
				best_score =(heading_bias(x_target, y_target)*((temp_score*DIST_GAIN)+1.0)/ (dist*(1.0-DIST_GAIN)+0.1));
				if (DISPLAY_OUTPUT) {cout << "New best score " << best_score << ":" << x_target
					<< "," << y_target << " size " << traj.size() << " dist " << dist << " score " << temp_score << endl;}
			}
			//cout << "current best score " << best_score <<  ":" << traj.back().x << "," << traj.back().y << " size " << traj.size() <<  endl;

			//finish time
			time(&finish);
		} // while time remaining
		if (DISPLAY_OUTPUT) {cout << "GP done looking at points" << endl;}
	} // else find good point
	// select highest scoring trajectory after XX seconds
	best_score = 0;
	if (traj.empty()) {
		cout << " traj is empty - no valid trajectory" << endl;
		return;
	} else {
		cout << "final best goal " << traj.back().x << "," << traj.back().y
			<< " size " << traj.size() << endl;
	}
	//allocate space for scores
	traj_score_l.resize(traj.size());
	traj_score_r.resize(traj.size());

	// copy original coverage map back...
	memcpy((void *) temp_cover_map, (void *) cover_map, coverage_size_x
			* coverage_size_y * (sizeof(unsigned char)));

	const float ANGLE_45 = (float)(M_PI / 4.0);
	const float ANGLE_120 = (float)(M_PI * 2.0 / 3.0);

	traj[0].xx = traj[0].x * cost_cell_size;
	traj[0].yy = traj[0].y * cost_cell_size;

	for (int current_loc = 1; current_loc < (int)traj.size(); current_loc++) {
		//cout << "loc " <<  traj[current_loc].x << "," << traj[current_loc].y << endl;
		// determine the direction of travel in each axis
		int x_dir = traj[current_loc].x - traj[current_loc - 1].x + 1;
		int y_dir = traj[current_loc].y - traj[current_loc - 1].y + 1;
		int direction = dir[x_dir][y_dir];

		//cout << direction << ": dir 0 is along x-axis" << SVR[direction] << " " << FVR[direction] << " " << FVL[direction] << endl;
		if (direction != NOMOVE) {
			// pass current location and inflated map to raycaster returns score
			traj_score_r[current_loc] = cast_all_rays(traj[current_loc].x,
					traj[current_loc].y, temp_cover_map, elev_map,
					SVR[direction], FVR[direction]);
			traj_score_l[current_loc] = cast_all_rays(traj[current_loc].x,
					traj[current_loc].y, temp_cover_map, elev_map,
					SVL[direction], FVL[direction]);
			best_score += traj_score_l[current_loc] + traj_score_r[current_loc];
			// set theta = to angle of travel
			traj[current_loc].theta = direction * ANGLE_45;
			traj[current_loc].xx = traj[current_loc].x * cost_cell_size;
			traj[current_loc].yy = traj[current_loc].y * cost_cell_size;

			// if r is greater than margin*l then keep the head pointed to the right otherwise check for left, otherwise sweep fully
//			float angle_swept = 0; // how far does sensor need to sweep
			int cells_to_see = 0; // how many cells should be detected at this location
//			int flag;
			if (traj_score_r[current_loc] > LR_MARGIN
					* traj_score_l[current_loc]) {
				traj[current_loc].right_pan = traj[current_loc].theta
						- ANGLE_120;
				traj[current_loc].left_pan = traj[current_loc].theta + ANGLE_45;
//				angle_swept = ANGLE_120 + ANGLE_45;
				cells_to_see = traj_score_r[current_loc];
//				flag = 1;
			} else if (traj_score_l[current_loc] > LR_MARGIN
					* traj_score_r[current_loc]) {
				traj[current_loc].right_pan = traj[current_loc].theta
						- ANGLE_45;
				traj[current_loc].left_pan = traj[current_loc].theta
						+ ANGLE_120;
//				angle_swept = ANGLE_120 + ANGLE_45;
				cells_to_see = traj_score_l[current_loc];
//				flag = 3;
			} else {
				traj[current_loc].right_pan = traj[current_loc].theta
						- ANGLE_120;
				traj[current_loc].left_pan = traj[current_loc].theta
						+ ANGLE_120;
//				angle_swept = ANGLE_120 + ANGLE_120;
				cells_to_see = traj_score_l[current_loc]
						+ traj_score_r[current_loc];
//				flag = 2;
			}

			// ensure angle values are within range
			if (traj[current_loc].right_pan > 2 * M_PI) {
				traj[current_loc].right_pan -= (float)(2 * M_PI);
			}
			if (traj[current_loc].right_pan < 0) {
				traj[current_loc].right_pan += (float)(2 * M_PI);
			}
			if (traj[current_loc].left_pan > 2 * M_PI) {
				traj[current_loc].left_pan -= (float)(2 * M_PI);
			}
			if (traj[current_loc].left_pan < 0) {
				traj[current_loc].left_pan += (float)(2 * M_PI);
			}
			if (traj[current_loc].theta > 2 * M_PI) {
				traj[current_loc].theta -= (float)(2 * M_PI);
			}
			if (traj[current_loc].theta < 0) {
				traj[current_loc].theta += (float)(2 * M_PI);
			}

			//printf("dir.flag r/c/l: c/c = v %i.%i %f %f %f :", direction,flag, traj[current_loc].right_pan, traj[current_loc].theta, traj[current_loc].left_pan); fflush(stdout);
			// determine speed for each cell
			float speed;
			//printf(" %i / %i ", CELLS_PER_SEC, cells_to_see); fflush(stdout);
			speed = ((float) CELLS_PER_SEC / (float) cells_to_see)
					* cost_cell_size * VIEW_PROB_FACTOR; // corrects for scoring by multiplying by KNOWN value
			if (speed > MAX_VELOCITY) {
				speed = MAX_VELOCITY;
			}
			traj[current_loc].velocity = speed;

			//printf("= %f\n", traj[current_loc].velocity);  fflush(stdout);


		} // if !NOMOVE
	} //for current_loc

	if (!traj.empty())
		if (DISPLAY_OUTPUT) {cout << "goal point " << traj.back().x << "," << traj.back().y
				<< " cost val = " << (int) cost_map[traj.back().x + cost_size_x
				* traj.back().y];}

	// post process traj to smooth
	// determine best direction to point sensor head
	// determine best velocity

	// write results to disk - cover map shows what was presumed to have been seen during traversal
	if (WRITE_FILES) {
		if (DISPLAY_OUTPUT) {printf(" writing map files to disk\n");}
		writefiles(temp_cover_map, inflated_cost_map, elev_map, "Map_out.txt",
				cost_size_x, cost_size_y);
		writefileextra(dijkstra, "Map_extra.txt", cost_size_x, cost_size_y);
		writefiletraj(best_score, traj, "Map_traj.txt");
		writeBMP(cover_map, inflated_cost_map, cost_size_x, cost_size_y , traj , "Map.bmp") ;
	}

	// set variables and allocate new storage for trajectory array of floats
	gp_traj.num_traj_pts = traj.size();
	gp_traj.traj_dim = GP_TRAJ_DIM;
	delete[] gp_traj.traj_array;
	gp_traj.traj_array = new float[gp_traj.num_traj_pts * GP_TRAJ_DIM];

	//set traj data into array of floats
	for (int q = 0; q < gp_traj.num_traj_pts; q++) {
		gp_traj.traj_array[q * GP_TRAJ_DIM] = (float) traj[q].x
				* cost_cell_size;
		gp_traj.traj_array[q * GP_TRAJ_DIM + 1] = (float) traj[q].y
				* cost_cell_size;
		gp_traj.traj_array[q * GP_TRAJ_DIM + 2] = traj[q].theta;
		gp_traj.traj_array[q * GP_TRAJ_DIM + 3] = traj[q].velocity;
		gp_traj.traj_array[q * GP_TRAJ_DIM + 4] = traj[q].right_pan;
		gp_traj.traj_array[q * GP_TRAJ_DIM + 5] = traj[q].left_pan;
	}

	//publish message to screen and to server
	//IPC_printData(IPC_msgFormatter (GP_TRAJECTORY_MSG), stdout, &gp_traj);
	//	IPC_publishData(GP_TRAJECTORY_MSG, &gp_traj); // needed for XP

	// frees un-needed arrays
	delete[] IG_map;
	delete[] dijkstra;
	delete[] obs_array;
	delete[] nonfree_array;
	delete[] obs_ptr_array;
	delete[] nonfree_ptr_array;
	delete[] temp_cover_map;
	//printf("done deleting temp storage\n");
}

/*
 static void GP_MAP_DATA_Handler (MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
 //function to handle map parameter update messages
 GP_MAP_DATA_PTR gp_map_data_p;
 IPC_unmarshall(IPC_msgInstanceFormatter(msgRef), callData, (void **)&gp_map_data_p);
 printf("Handler: Receiving %s (size %lu) [%s] \n", IPC_msgInstanceName(msgRef),  sizeof(callData), (char *)clientData);

 //function to print message to screen
 //IPC_printData(IPC_msgInstanceFormatter(msgRef), stdout, gp_map_data_p);

 //update size variables
 cost_size_x = gp_map_data_p->cost_size_x;
 cost_size_y = gp_map_data_p->cost_size_y;
 coverage_size_x = gp_map_data_p->coverage_size_x;
 coverage_size_y = gp_map_data_p->coverage_size_y;
 elev_size_x = gp_map_data_p->elev_size_x;
 elev_size_y = gp_map_data_p->elev_size_y;
 cost_cell_size = gp_map_data_p->cost_cell_size;
 coverage_cell_size = gp_map_data_p->coverage_cell_size;
 elev_cell_size = gp_map_data_p->elev_cell_size;

 //initialize astar vectors for later use
 Astarpoint_init(cost_size_x, cost_size_y);

 //allocates space for the maps
 map_alloc();

 //sets greedy threshold
 HIGH_IG_THRES = (int)(IG_RATIO*cost_size_x*cost_size_y*KNOWN);

 //frees memory used by the message
 IPC_freeDataElements(IPC_msgInstanceFormatter(msgRef), (void *) gp_map_data_p);
 IPC_freeByteArray(callData);
 }
 */

/*
 static void GP_ROBOT_PARAMETER_Handler (MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
 // function handles robot parameter update messages
 GP_ROBOT_PARAMETER_PTR gp_robot_parameter_p;
 IPC_unmarshall(IPC_msgInstanceFormatter(msgRef), callData, (void **)&gp_robot_parameter_p);
 printf("Handler: Receiving %s (size %lu) [%s] \n", IPC_msgInstanceName(msgRef),  sizeof(callData), (char *)clientData);

 //stores new parameters
 MAX_VELOCITY = gp_robot_parameter_p->MAX_VELOCITY;
 MAX_TURN_RATE = gp_robot_parameter_p->MAX_TURN_RATE;
 sensor_radius = gp_robot_parameter_p->sensor_radius;
 sensor_height = gp_robot_parameter_p->sensor_height;

 // establish sensor endpoints
 int	sensor_int_radius = (int)(sensor_radius/cost_cell_size);
 rasterCircle((int)(sensor_radius/cost_cell_size));

 //determine start and stop vector numbers for each direction
 int delta_vec = NUMVECTORS*SENSORWIDTH/(2*M_PI); // 120 degrees worth of vectors
 int cardinal_vec = NUMVECTORS/8; // 45 degrees worth of vectors
 for(int i=0; i<8; i++) {
 FVR[i] = SVL[i] = ValidVec(cardinal_vec*i);
 FVL[i] = ValidVec(cardinal_vec*i+delta_vec);
 SVR[i] = ValidVec(cardinal_vec*i-delta_vec);

 cout << " vectors " << SVR[i] << " " << FVR[i] << " " << SVL[i] << " " << FVL[i] << endl;

 }

 //for nomove set the start and finish vectors to a full circle
 SVR[8] = SVL[8] = 0;
 FVR[8] = FVL[8] = NUMVECTORS-1;


 //determines outer bounding circle of robot for inflation purposes
 float max_dist=0;
 for (int i=0;i<gp_robot_parameter_p->I_DIMENSION; i++) {
 float dist = sqrt(pow(gp_robot_parameter_p->PerimeterArray[i*gp_robot_parameter_p->J_DIMENSION], 2)+ pow(gp_robot_parameter_p->PerimeterArray[i*gp_robot_parameter_p->J_DIMENSION+1], 2));
 printf(" data point %d is %f away\n", i, dist);
 if (dist > max_dist) {max_dist = dist;}
 }
 inflation_size = max_dist/cost_cell_size;

 //function to print message to screen
 //IPC_printData(IPC_msgInstanceFormatter(msgRef), stdout, gp_robot_parameter_p);

 //free the memory used by the message
 IPC_freeDataElements(IPC_msgInstanceFormatter(msgRef), (void *) gp_robot_parameter_p);
 IPC_freeByteArray(callData);
 }
 */

/*
 static void GP_POSITION_UPDATE_Handler (MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
 //function handles the position update messages
 GP_POSITION_UPDATE_PTR gp_position_update_p;
 IPC_unmarshall(IPC_msgInstanceFormatter(msgRef), callData, (void **)&gp_position_update_p);
 printf("Handler: Receiving %s (size %lu) [%s] \n", IPC_msgInstanceName(msgRef),  sizeof(callData), (char *)clientData);

 //updates the stored robot position as a float
 robot_xx=gp_position_update_p->x;
 robot_yy=gp_position_update_p->y;

 // .... and as an int in cells
 int temp_robot_x= (int)(robot_xx/cost_cell_size);
 int temp_robot_y= (int)(robot_yy/cost_cell_size);

 //updtes orientation
 theta = gp_position_update_p->theta;

 //function to print message data to screen
 IPC_printData(IPC_msgInstanceFormatter(msgRef), stdout, gp_position_update_p);

 //calls the global planner to get (and publish) a new plan
 if (OnMap(temp_robot_x, temp_robot_y) && ( temp_robot_x !=0) && (temp_robot_y !=0)) {
 robot_x = temp_robot_x;
 robot_y = temp_robot_y;
 global_planner(-1, -1, -1);
 }
 else { cout << " Invalid position received " << endl; }

 //frees memory used by message
 IPC_freeDataElements(IPC_msgInstanceFormatter(msgRef), (void *)gp_position_update_p);
 IPC_freeByteArray(callData);
 }
 */

/*
 static void GP_GOAL_ASSIGN_Handler (MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
 //function handles the position update messages
 GP_GOAL_ASSIGN_PTR gp_goal_assign_p;
 IPC_unmarshall(IPC_msgInstanceFormatter(msgRef), callData, (void **)&gp_goal_assign_p);
 printf("Handler: Receiving %s (size %lu) [%s] \n", IPC_msgInstanceName(msgRef),  sizeof(callData), (char *)clientData);

 //updates the stored robot position as a float
 robot_xx=gp_goal_assign_p->x;
 robot_yy=gp_goal_assign_p->y;

 // .... and as an int in cells
 robot_x= (int)(robot_xx/cost_cell_size);
 robot_y= (int)(robot_yy/cost_cell_size);

 //updates orientation
 theta = gp_goal_assign_p->theta;

 //function to print message data to screen
 IPC_printData(IPC_msgInstanceFormatter(msgRef), stdout, gp_goal_assign_p);

 //calls the global planner to get (and publish) a new plan
 global_planner(gp_goal_assign_p->goal_x, gp_goal_assign_p->goal_y,  gp_goal_assign_p->goal_theta );

 //frees memory used by message
 IPC_freeDataElements(IPC_msgInstanceFormatter(msgRef), (void *)gp_goal_assign_p);
 IPC_freeByteArray(callData);
 }
 */

/*
 static void GP_FULL_UPDATE_Handler (MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
 //function handles full map updates
 GP_FULL_UPDATE_PTR gp_full_update_p;
 IPC_unmarshall(IPC_msgInstanceFormatter(msgRef), callData, (void **)&gp_full_update_p);
 printf("Handler: Receiving %s (size %lu) [%s] \n", IPC_msgInstanceName(msgRef),  sizeof(callData), (char *)clientData);

 // function to print received message to screen
 //IPC_printData(IPC_msgInstanceFormatter(msgRef), stdout, gp_full_update_p);
 //update size variables
 cost_size_x = gp_full_update_p->sent_cost_x;
 cost_size_y = gp_full_update_p->sent_cost_y;
 coverage_size_x = gp_full_update_p->sent_cover_x;
 coverage_size_y = gp_full_update_p->sent_cover_y;
 elev_size_x = gp_full_update_p->sent_elev_x;
 elev_size_y = gp_full_update_p->sent_elev_y;

 //allocate memory for the maps according to the size variables
 map_alloc();
 //initialize the astar vectors for later use
 Astarpoint_init(cost_size_x, cost_size_y);
 //sets greedy threshold
 HIGH_IG_THRES = (int)(IG_RATIO*cost_size_x*cost_size_y*KNOWN);

 // copy data to local map
 memcpy((void *)cover_map, (void *)gp_full_update_p->coverage_map, coverage_size_x*coverage_size_y*sizeof(unsigned char));
 memcpy((void *)cost_map, (void *)gp_full_update_p->cost_map, cost_size_x*cost_size_y*sizeof(unsigned char));
 memcpy((void *)elev_map, (void *)gp_full_update_p->elev_map, elev_size_x*elev_size_y*sizeof(int16_t));

 //free memory used by message
 IPC_freeDataElements(IPC_msgInstanceFormatter(msgRef), (void *) gp_full_update_p);
 IPC_freeByteArray(callData);
 }
 */

/*
 static void GP_SHORT_UPDATE_Handler (MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
 // function handles short map updates
 GP_SHORT_UPDATE_PTR gp_short_update_p;
 IPC_unmarshall(IPC_msgInstanceFormatter(msgRef), callData, (void **)&gp_short_update_p);
 printf("Handler: Receiving %s (size %lu) [%s] \n", IPC_msgInstanceName(msgRef),  sizeof(callData), (char *)clientData);

 IPC_printData(IPC_msgInstanceFormatter(msgRef), stdout, gp_short_update_p);

 //variables for map coordinates
 int lly = gp_short_update_p->y_cost_start;
 int llx = gp_short_update_p->x_cost_start;
 int sizey = gp_short_update_p->sent_cost_y;
 int sizex = gp_short_update_p->sent_cost_x;

 //place data into the correct arrays
 for(int j = 0; j< sizey; j++){
 for (int i=0;i< sizex;i++) {
 if (OnMap(i+llx, j+lly)) {
 cover_map[(i+llx)+coverage_size_x*(j+lly)] = gp_short_update_p->coverage_map[i+sizex*j];
 cost_map[(i+llx)+cost_size_x*(j+lly)] = gp_short_update_p->cost_map[i+sizex*j];
 elev_map[(i+llx)+elev_size_x*(j+lly)] = gp_short_update_p->elev_map[i+sizex*j];
 }
 }
 }
 //free the message data
 IPC_freeDataElements(IPC_msgInstanceFormatter(msgRef), (void *) gp_short_update_p);
 IPC_freeByteArray(callData);
 }
 */

/*static void stdinHnd (int fd, void *clientData) {
 // function handles keyboard input for the purpose of writing to file or quitting
 char inputLine[81];
 fgets(inputLine, 80, stdin);
 switch (inputLine[0]) {
 case 'q': case 'Q':
 IPC_disconnect();
 exit(0);
 case 'f': case 'F':
 writefiles(cover_map, inflated_cost_map, elev_map, cost_size_x, cost_size_y);
 //writefileextra(dijkstra, cost_size_x, cost_size_y);
 //writefiletraj(best_score, traj);
 break;
 default:
 printf("stdinHnd [%s]: Received %s", (char *)clientData, inputLine);
 fflush(stdout);
 }
 printf("\n'f' to write to file\n'q' to quit\n");
 }
 */

bool GPLAN::gplan_init(GP_MAP_DATA * gp_map_data_p,
		GP_ROBOT_PARAMETER * gp_robot_parameter_p,
		GP_FULL_UPDATE * gp_full_update_p) {
	//function to handle map and robot parameters and loads full map

	//update size variables
	cost_cell_size = gp_map_data_p->cost_cell_size;
	coverage_cell_size = gp_map_data_p->coverage_cell_size;
	elev_cell_size = gp_map_data_p->elev_cell_size;
	printf(" Cells are %f %f %f m square\n", cost_cell_size,
			coverage_cell_size, elev_cell_size);

	//stores new parameters
	MAX_VELOCITY = gp_robot_parameter_p->MAX_VELOCITY;
	MAX_TURN_RATE = gp_robot_parameter_p->MAX_TURN_RATE;
	sensor_radius = gp_robot_parameter_p->sensor_radius;
	sensor_height = gp_robot_parameter_p->sensor_height;
	printf(
			" The robot can go %f m/s and turn at %f rad/sec.  The sensor is %d cm high and can see %f m\n",
			MAX_VELOCITY, MAX_TURN_RATE, sensor_height, sensor_radius);

	// establish sensor endpoints
//	int sensor_int_radius = (int) (sensor_radius / cost_cell_size);
	rasterCircle((int) (sensor_radius / cost_cell_size));

	//determine start and stop vector numbers for each direction
	int delta_vec = (int)(NUMVECTORS * SENSORWIDTH / (2.0 * M_PI)); // viewing angle width of vectors
	int cardinal_vec = (int)(NUMVECTORS / 8.0); // 45 degrees worth of vectors
	for (int i = 0; i < 8; i++) {
		FVR[i] = SVL[i] = ValidVec(cardinal_vec * i);
		FVL[i] = ValidVec(cardinal_vec * i + delta_vec);
		SVR[i] = ValidVec(cardinal_vec * i - delta_vec);
		cout << " vectors " << SVR[i] << " " << FVR[i] << " " << SVL[i] << " "
				<< FVL[i] << endl;
	}

	//for nomove set the start and finish vectors to a full circle
	SVR[8] = SVL[8] = 0;
	FVR[8] = FVL[8] = NUMVECTORS - 1;

	//determines outer bounding circle of robot for inflation purposes
	float max_dist = 0;
	for (int i = 0; i < gp_robot_parameter_p->I_DIMENSION; i++) {
		float dist = (float)sqrt(pow(gp_robot_parameter_p->PerimeterArray[i
				* gp_robot_parameter_p->J_DIMENSION], 2) + pow(
				gp_robot_parameter_p->PerimeterArray[i
						* gp_robot_parameter_p->J_DIMENSION + 1], 2));
		printf(" data point %d is %f away\n", i, dist);
		if (dist > max_dist) {
			max_dist = dist;
		}
	}
	inflation_size = max_dist / cost_cell_size;

	//update size variables
	cost_size_x = gp_full_update_p->sent_cost_x;
	cost_size_y = gp_full_update_p->sent_cost_y;
	coverage_size_x = gp_full_update_p->sent_cover_x;
	coverage_size_y = gp_full_update_p->sent_cover_y;
	elev_size_x = gp_full_update_p->sent_elev_x;
	elev_size_y = gp_full_update_p->sent_elev_y;
	printf("The maps are %d x %d\n", cost_size_x, cost_size_y);

	//allocate memory for the maps according to the size variables
	bool val = map_alloc();

	printf("done alloc ");
	fflush(stdout);
	//initialize the astar vectors for later use
	Astarpoint_init(cost_size_x, cost_size_y);
	//sets greedy threshold
	HIGH_IG_THRES = (int) (IG_RATIO * cost_size_x * cost_size_y * KNOWN);

	printf("memcpy ");
	fflush(stdout);
	// copy data to local map
	memcpy((void *) cover_map, (void *) gp_full_update_p->coverage_map,
			coverage_size_x * coverage_size_y * sizeof(unsigned char));
	memcpy((void *) cost_map, (void *) gp_full_update_p->cost_map, cost_size_x
			* cost_size_y * sizeof(unsigned char));
	memcpy((void *) elev_map, (void *) gp_full_update_p->elev_map, elev_size_x
			* elev_size_y * sizeof(int16_t));

	printf("done\n");
	fflush(stdout);
	return val;
}

bool GPLAN::gplan_init(
    GP_MAP_DATA* gp_map_data_p,
    GP_ROBOT_PARAMETER* gp_robot_parameter_p,
    GP_FULL_UPDATE* gp_full_update_p,
    GP_PLANNER_PARAMETER* gp_planner_param_p)
{
	// function to initialize map and robot data, read in initial full map, and set highlevel planning parameters
	gplan_init(gp_map_data_p, gp_robot_parameter_p, gp_full_update_p);

	// set variables
	GP_PLAN_TIME = gp_planner_param_p->GP_PLAN_TIME; // seconds to allow for planning
	if (gp_planner_param_p->IG_RATIO <= 1) {IG_RATIO = gp_planner_param_p->IG_RATIO;} // ratio of total possible unknown to remaining unknown before switching to pure greedy search
	LR_MARGIN = gp_planner_param_p->LR_MARGIN; // amount one side must be greater to influence pan angles
	VIEW_PROB_FACTOR = gp_planner_param_p->VIEW_PROB_FACTOR; // retards calculated speed based on likelihood of observation
	WRITE_FILES = gp_planner_param_p->WRITE_FILES; // flag to write files out
	DISPLAY_OUTPUT = gp_planner_param_p->DISPLAY_OUTPUT; // flag to display any output
	if (abs(gp_planner_param_p->SENSORWIDTH) < 2*M_PI) {
        SENSORWIDTH = gp_planner_param_p->SENSORWIDTH;
    }

	if ((gp_planner_param_p->DIST_GAIN >=0) && (gp_planner_param_p->DIST_GAIN <=1)) {
        DIST_GAIN = gp_planner_param_p->DIST_GAIN;
    }

    return true;
}

vector<Traj_pt_s> GPLAN::gplan_plan(GP_POSITION_UPDATE * gp_position_update_p,
		GP_SHORT_UPDATE * gp_short_update_p) {
	//function replans based on updated short map and position update

	//variables for map coordinates
	int lly = gp_short_update_p->y_cost_start;
	int llx = gp_short_update_p->x_cost_start;
	int sizey = gp_short_update_p->sent_cost_y;
	int sizex = gp_short_update_p->sent_cost_x;

	if (DISPLAY_OUTPUT) {printf(" sent map size is %d x %d starting at (%d,%d)\n", sizex, sizey, llx, lly);}

	//place data into the correct arrays
	for (int j = 0; j < sizey; j++) {
		for (int i = 0; i < sizex; i++) {
			if (OnMap(i + llx, j + lly)) {
				cover_map[(i + llx) + coverage_size_x * (j + lly)]
						= gp_short_update_p->coverage_map[i + sizex * j];
				cost_map[(i + llx) + cost_size_x * (j + lly)]
						= gp_short_update_p->cost_map[i + sizex * j];
				elev_map[(i + llx) + elev_size_x * (j + lly)]
						= gp_short_update_p->elev_map[i + sizex * j];
			}
		}
	}

	//updates the stored robot position as a float
	robot_xx = gp_position_update_p->x;
	robot_yy = gp_position_update_p->y;


	// .... and as an int in cells

	int temp_robot_x= (int)((robot_xx/cost_cell_size)+0.5);
	int temp_robot_y= (int)((robot_yy/cost_cell_size)+0.5);

	//updates orientation
	theta = gp_position_update_p->theta;

	//calls the global planner to get (and publish) a new plan
	if (OnMap(temp_robot_x, temp_robot_y) && (temp_robot_x != 0)
			&& (temp_robot_y != 0)) {
		robot_x = temp_robot_x;
		robot_y = temp_robot_y;
		global_planner(-1, -1, -1);
	} else {
		cout << " Invalid position received " << endl;
	}

	return traj;
}

/*

 int main (void) {
 //initialize variables and seed random number generator
 gp_traj.traj_array = new float[1];
 srand((int)(time(NULL)));

 //stuff to initialize system in the event no init message comes
 //initialize astar vectors for later use
 Astarpoint_init(cost_size_x, cost_size_y);
 //sets greedy threshold
 HIGH_IG_THRES = (int)(IG_RATIO*cost_size_x*cost_size_y*KNOWN);
 // establish sensor endpoints
 int	sensor_int_radius = (int)(sensor_radius/cost_cell_size);
 rasterCircle((int)(sensor_radius/cost_cell_size));

 //determine start and stop vector numbers for each direction
 int delta_vec = NUMVECTORS*SENSORWIDTH/(2*M_PI); // 120 degrees worth of vectors
 int cardinal_vec = NUMVECTORS/8; // 45 degrees worth of vectors
 for(int i=0; i<8; i++) {
 FVR[i] = SVL[i] = ValidVec(cardinal_vec*i);
 FVL[i] = ValidVec(cardinal_vec*i+delta_vec);
 SVR[i] = ValidVec(cardinal_vec*i-delta_vec);

 cout << " vectors " << SVR[i] << " " << FVR[i] << " " << SVL[i] << " " << FVL[i] << endl;

 }

 //for nomove set the start and finish vectors to a full circle
 SVR[8] = SVL[8] = 0;
 FVR[8] = FVL[8] = NUMVECTORS-1;



 // Connect to the central server
 printf("\nIPC_connect(%s)\n", MODULE_NAME);
 IPC_connect(MODULE_NAME);

 // Define the messages that this module publishes
 printf("\nIPC_defineMsg(%s, IPC_VARIABLE_LENGTH, %s)\n", GP_TRAJECTORY_MSG, GP_TRAJECTORY_FORM);
 IPC_defineMsg(GP_TRAJECTORY_MSG, IPC_VARIABLE_LENGTH, GP_TRAJECTORY_FORM);

 // Subscribe to the messages that this module listens to.
 printf("\nIPC_subscribe(%s, GP_MAP_DATA_Handler, %s)\n", GP_MAP_DATA_MSG, MODULE_NAME);
 IPC_subscribe(GP_MAP_DATA_MSG, GP_MAP_DATA_Handler, (void *)MODULE_NAME);
 IPC_setMsgQueueLength(GP_MAP_DATA_MSG, 1);

 printf("\nIPC_subscribe(%s, GP_ROBOT_PARAMETER_Handler, %s)\n", GP_ROBOT_PARAMETER_MSG, MODULE_NAME);
 IPC_subscribe(GP_ROBOT_PARAMETER_MSG, GP_ROBOT_PARAMETER_Handler, (void *)MODULE_NAME);
 IPC_setMsgQueueLength(GP_ROBOT_PARAMETER_MSG, 1);

 printf("\nIPC_subscribe(%s, GP_POSITION_UPDATE_Handler, %s)\n", GP_POSITION_UPDATE_MSG, MODULE_NAME);
 IPC_subscribe(GP_POSITION_UPDATE_MSG, GP_POSITION_UPDATE_Handler, (void *)MODULE_NAME);
 IPC_setMsgQueueLength(GP_POSITION_UPDATE_MSG, 1);

 printf("\nIPC_subscribe(%s, GP_GOAL_UPDATE_Handler, %s)\n", GP_GOAL_ASSIGN_MSG, MODULE_NAME);
 IPC_subscribe(GP_GOAL_ASSIGN_MSG, GP_GOAL_ASSIGN_Handler, (void *)MODULE_NAME);

 printf("\nIPC_subscribe(%s, GP_FULL_UPDATE_Handler, %s)\n", GP_FULL_UPDATE_MSG, MODULE_NAME);
 IPC_subscribe(GP_FULL_UPDATE_MSG, GP_FULL_UPDATE_Handler, (void *)MODULE_NAME);

 printf("\nIPC_subscribe(%s, GP_SHORT_UPDATE_Handler, %s)\n", GP_SHORT_UPDATE_MSG, MODULE_NAME);
 IPC_subscribe(GP_SHORT_UPDATE_MSG, GP_SHORT_UPDATE_Handler, (void *)MODULE_NAME);

 // Subscribe a handler for tty input. Typing "q" will quit the program.
 //	printf("\nIPC_subscribeFD(%d, stdinHnd, %s)\n", _fileno(stdin),MODULE_NAME);
 //IPC_subscribeFD(_fileno(stdin), stdinHnd, (void *)MODULE_NAME);
 printf("\nType 'q' to quit\n");

 IPC_setVerbosity(IPC_Print_Errors);

 IPC_dispatch();

 //wait for messages
 //while(true) {
 //IPC_listen(1000);
 //}

 //clean up on exit
 IPC_disconnect();
 return(0);
 }

 */

