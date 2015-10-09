#ifndef _MAP_GLOBALS_JMB
#define _MAP_GLOBALS_JMB

// flag to determine if unknown cells are allowed to be traversed (unknown are set to obstacles if not)
#define UNKNOWN_ARE_PASSABLE false
/*
extern float coverage_cell_size; // size of cell in meters
extern float cost_cell_size; // size of cell in meters
extern float elev_cell_size; // size of cell in meters
extern int coverage_size_x; // x and y dimension of coverage map
extern int coverage_size_y;
extern int cost_size_x;// x and y dimension of cost map
extern int cost_size_y;
extern int elev_size_x; // x and y dimension of elevation map
extern int elev_size_y;

extern float sensor_radius; // sensor radius in meters
extern int16_t sensor_height; // sensor height in centimeters
extern int NUMVECTORS;
*/
const int MINFLAG = 251; // value of lowest FLAG
const int OBSTACLE = 250; // value of OBSTACLES on cost map
const int UNKOBSTACLE = 251; // value for UNKNOWN cells on obstacle map
const int UNKNOWN = 0; // value for unknown on coverage map
const int KNOWN = 249; // value for known on coverage map
//const int UNKSCORE = 256; // score to assign for UNKNOWN (NOT USED)	
const int DIJKSTRA_LIMIT = 10000000; // value above which cells are not reachable
//extern int HIGH_IG_THRES; // value of IG below which search is pure greedy
const int16_t OBS16 = 30000; // default value for obstacles

// 8 connected moves and costs
enum {MOVERIGHT, MOVEUPRIGHT, MOVEUP, MOVEUPLEFT, MOVELEFT, MOVEDOWNLEFT, MOVEDOWN, MOVEDOWNRIGHT, NOMOVE};
const int dir[3][3] = {{MOVEDOWNLEFT, MOVELEFT, MOVEUPLEFT}, {MOVEDOWN, NOMOVE, MOVEUP}, {MOVEDOWNRIGHT, MOVERIGHT, MOVEUPRIGHT}};
const double stepcost[3][3] = {{1.414213562, 1, 1.414213562}, {1, 0, 1}, {1.414213562, 1, 1.414213562}};

// struct used internally for trajectories
#define GP_TRAJ_DIM 8 

struct Traj_pt_s {
	int x; // position in cells
	int y;
	float xx; // position in meters
	float yy;
	float theta; // heading in radians
	float velocity; // in m/s
	float right_pan; // heading of right pan limit in radians
	float left_pan; // heading of left pan limit in radians

	Traj_pt_s() {
		x = 0;
		y = 0;
		xx =0;
		yy=0;
		theta = 0.0;
		velocity = 1.0;
		right_pan = 0.0;
		left_pan = 0.0;
		}

	Traj_pt_s(int a, int b, float c, float d, float e, float f, float g, float h) {
		x = a;
		y = b;
		xx = c;
		yy = d;
		theta = e;
		velocity = f;
		right_pan = g;
		left_pan = h;
	}
};



class frontier_pts {
	public:
		int x;
		int y;
		unsigned int IG;
		int cost;
		double weight;
		double total;

		frontier_pts() {
			x = -1;
			y = -1;
			IG = 0;
			cost = 0;
			total = 0;
			weight = 1.0;
		}
		frontier_pts(int a, int b, unsigned int c, int d, double e) {
			x = a;
			y = b;
			IG = c;
			cost =  d;
			weight = e;
			total = ((c*e)+1.0)/ (d*(1.0-e)+0.1);
		}
};

class fp_compare {
	public:
		bool operator () (const frontier_pts& lhs, const frontier_pts& rhs) const
		{
			return (lhs.total<rhs.total); 
		}
};


struct RAY_TEMPLATE_PT {
int x;
int y;
float angle;

	RAY_TEMPLATE_PT() {
		x = 0;
		y = 0;
		angle = 0.0;
		}

	RAY_TEMPLATE_PT(int a, int b, float c) {
		x = a;
		y = b;
		angle = c;
		}
};

#endif
