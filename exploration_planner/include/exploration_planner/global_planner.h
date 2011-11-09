#ifndef _GLOBAL_PLANNER_JMB
#define _GLOBAL_PLANNER_JMB

#include <exploration_planner/common_headers.h>

using namespace std;

class GPLAN {
	public:

		bool gplan_init(GP_MAP_DATA * gp_map_data_p, GP_ROBOT_PARAMETER * gp_robot_parameter_p, GP_FULL_UPDATE * gp_full_update_p);
		bool gplan_init(GP_MAP_DATA * gp_map_data_p, GP_ROBOT_PARAMETER * gp_robot_parameter_p, GP_FULL_UPDATE * gp_full_update_p, GP_PLANNER_PARAMETER  * gp_planner_param_p);
		vector<Traj_pt_s> gplan_plan(GP_POSITION_UPDATE * gp_position_update_p, GP_SHORT_UPDATE * gp_short_update_p);

		//constuctor
		GPLAN();


	private:

		//map variables
		int map_sizex_m;
		int map_sizey_m;
		float coverage_cell_size;
		float cost_cell_size;
		float elev_cell_size;
		int coverage_size_x;
		int coverage_size_y;
		int cost_size_x;
		int cost_size_y;
		int elev_size_x;
		int elev_size_y;
		unsigned char * cover_map;
		unsigned char * cost_map;
		int16_t * elev_map; 
		unsigned char * inflated_cost_map;
		unsigned char ** cost_map_pa; // ptr to first element of each row
		unsigned char ** inf_cost_map_pa; //ptr to first element of each row for inflated map

		//posit variables
		float robot_xx, robot_yy; // float posit of robot
		int robot_x, robot_y; // x and y cell coordinates of robot
		float theta;

		//robot variables
		float sensor_radius; // distance sensors can see in m
		int16_t sensor_height; // sensor height in cm
		float SENSORWIDTH;  // sensor width in radians used to determine start and finish vectors for ray tracing
		float inflation_size;  //size in cells to inflate obstacles
		float MAX_VELOCITY; // meters per second
		float MAX_TURN_RATE; // radians per second
		int CELLS_PER_SEC; // number of cells that can be swept out per second
		float VIEW_PROB_FACTOR; // retards calculated speed based on likelihood of observation

		//planner variables
		float GP_PLAN_TIME; // seconds to allow for planning
		int HIGH_IG_THRES; // set by initialization and full map updates
		float IG_RATIO; // ratio of total possible unknown to remaining unknown before switching to pure greedy search
		float LR_MARGIN; // amount one side must be greater to influence pan angles
		double DIST_GAIN; // factor to switch between greedy and IG
		bool WRITE_FILES; // flag to write files out
		bool DISPLAY_OUTPUT; // flag to display any output
		double THETA_BIAS; // 0 TO 1 Adds additional bias to rough direction to goal location
		// sensor variables
		int NUMVECTORS;
		vector<RAY_TEMPLATE_PT> rayendpts;
		int SVL[9], SVR[9], FVL[9], FVR[9];  // holds the start and finish vectors for the right and left 120 arcs

		// output and misc variables
		priority_queue<frontier_pts, vector<frontier_pts>, fp_compare> frontier;
		vector<Traj_pt_s> traj; // trajectory
		GP_TRAJECTORY gp_traj; // message holder


		// functions
		void setPixel(int x, int y);
		void rasterCircle(int radius);
		int ValidVec(int vec);
		bool OnMap(int x, int y);
		double return_path(int x_target, int y_target, const int dijkstra[], vector<Traj_pt_s> & traj);
		bool map_alloc(void);
		void sample_point(int &x_target, int &y_target, const int dijkstra[], const unsigned char temp_cover_map[], const unsigned int IG_map[]);
		void goto_nearest(int &x_target, int &y_target, const int dijkstra[], const unsigned char temp_cover_map[]);
		bool calc_all_IG(unsigned int IG_map[]);
		unsigned int get_IG(unsigned int IG_map[], int x, int y, int dim);
		void find_frontier(unsigned int IG_map[], int dijkstra[]);
		void global_planner(float goal_x, float goal_y, float goal_theta);
		void cast_single_ray(int x0, int y0, int x1, int y1, int & score, unsigned char cover_map[], const int16_t elev[]) ;
		int cast_all_rays(int x0, int y0, unsigned char cover_map[], const int16_t elev_map[], int start_vec, int end_vec);
		void Astarpoint_init(int size_x, int size_y);
		bool AstarGC(const int startx,const int starty,const int goalx,const int goaly, const unsigned char cost_map[]);
		void Astarpoint(const int startx,const int starty,const int goalx,const int goaly, const unsigned char cost_map[], double & dist_traveled, std::vector<Traj_pt_s> & bestpath);
		void Astarnearest(const int startx,const int starty,int & goalx,int & goaly, const unsigned char cost_map[], const unsigned char cover_map[]);
		double heading_bias(int x, int y);
};

#endif
