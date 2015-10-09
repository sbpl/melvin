#include <exploration_planner/common_headers.h>
#include <exploration_planner/astarpoint.h>
using namespace std;

double INFd = numeric_limits<double>::infinity();
vector<vector<bool> > Closed;
vector<vector<double> > G;

class Cell {
	public:
		int x;
		int y;
		float f;
		float h;

		friend bool operator< (const Cell & cell1, const Cell & cell2);
		friend bool operator> (const Cell & cell1, const Cell & cell2);

		Cell() { x=0; y=0; f=0; h=0; }

		Cell(int a, int b, float c, float d) { 
			x=a;
			y=b;
			f=c + d;
			h=d;
		}

};

bool operator< (const Cell & cell1, const Cell & cell2)
{ return cell1.f < cell2.f; }

bool operator> (const Cell & cell1, const Cell & cell2)
{ return cell1.f > cell2.f; }

void GPLAN::Astarpoint_init(int size_x, int size_y) {
	// Set up sizes
	Closed.resize(size_x);
	G.resize(size_x);
	for (int i = 0; i < size_x; ++i) {
		Closed[i].resize(size_y);
		G[i].resize(size_y);
	}
}


void GPLAN::Astarnearest(const int startx,const int starty,int & goalx,int & goaly, const unsigned char cost_map[], const unsigned char cover_map[]) {
	// initialize Closed and G arrays
	//
	for (int i = 0;i < cost_size_x; i++) {
		for(int j = 0; j < cost_size_y; j++) {
			Closed[i][j] = false;
			G[i][j] = INFd;
		}
	}

	// initialize cost per step
	//  3 2 1
	//  4   0
	//  5 6 7
	
	float stepcost[8];
	stepcost[0] = stepcost[2] = stepcost[4] = stepcost[6] = (float)1*cost_cell_size;
	stepcost[1] = stepcost[3] = stepcost[5] = stepcost[7] = (float)(sqrt(2.0)*cost_cell_size);

	const int dX[8] = {1, 1, 0, -1, -1, -1,  0,  1};
	const int dY[8] = {0, 1, 1,  1,  0, -1, -1, -1};

	float dist = 0;

	// setup open cell queue
	priority_queue<Cell, deque<Cell>,greater<Cell> > OpenCells;

	Cell current;

	G[startx][starty] = 0;

	// init current cell to start
	current.x = startx;
	current.y = starty;
	current.h = 0;
	current.f = current.h;

	OpenCells.push(current);

	while ((!OpenCells.empty()) && (cover_map[current.x+coverage_size_x* current.y] == KNOWN)) { // while not empty and goal not evaluated 
	//printf(" %d %d \n", current.x, current.y ); fflush(stdout);
		current = OpenCells.top(); // read top value
		OpenCells.pop(); // remove from queue
	//cout << current.x << "," << current.y<< endl;
		Closed[current.x][current.y] = true; // add to closed
		for (int k = 0;k<8;k++) { // check neighbors
			int evalx = current.x + dX[k];
			int evaly = current.y + dY[k];
			if ((evalx >= 0) && (evalx < cost_size_x)) {  // within x bounds
				if ((evaly >= 0) && (evaly < cost_size_y)) { // within y bounds
					if (cost_map[evalx+cost_size_x * evaly] != OBSTACLE) { // not an obstacle  (may be a unknown square otherwise classified as an obstacle)
						if (!Closed[evalx][evaly]) { // not already evaluated
							if (G[evalx][evaly] > G[current.x][current.y] + stepcost[k]) {  // if smaller g value
								G[evalx][evaly] = G[current.x][current.y] + stepcost[k]; // update g
								Cell temp(evalx, evaly, G[evalx][evaly]+cost_map[evalx+cost_size_x*evaly],0); // setup cell
								OpenCells.push(temp); // add to open list
							} // end smaller g
						} // end closed
					} // end obstacle
				} // end cost_size_y
			} // end x dim
		} // end check neighbors
	} // end while

	if (OpenCells.empty()) {
		goalx = -1;
		goaly = -1;
	}

	if  (cover_map[current.x+coverage_size_x* current.y] != KNOWN) {
		goalx = current.x;
		goaly = current.y;
	}
//cout << endl << endl << current.x << "," << current.y << endl;

}




void GPLAN::Astarpoint(const int startx,const int starty,const int goalx,const int goaly, const unsigned char cost_map[], double & dist_traveled, vector<Traj_pt_s> & bestpath ) {
	// returns path 
	// inputs are start and goal grid locations, the cost map
	printf("astar starts \n");
	// initialize Closed and G arrays
	for (int i = 0;i < cost_size_x; i++) {
		for(int j = 0; j < cost_size_y; j++) {
			Closed[i][j] = false;
			G[i][j] = INFd;
		}
	}
printf("astar closed and g initialized\n");
	// initialize cost per step
	//  3 2 1
	//  4   0
	//  5 6 7
	
	double stepcost[8];
	stepcost[0] = stepcost[2] = stepcost[4] = stepcost[6] = 1*cost_cell_size;
	stepcost[1] = stepcost[3] = stepcost[5] = stepcost[7] = sqrt(2.0)*cost_cell_size;

	const int dX[8] = {1, 1, 0, -1, -1, -1,  0,  1};
	const int dY[8] = {0, 1, 1,  1,  0, -1, -1, -1};

	double dist = 0;

	// setup open cell queue
	priority_queue<Cell, deque<Cell>,greater<Cell> > OpenCells;
printf("astar open\n");
	Cell current;

	G[startx][starty] = 0;

	// init current cell to start
	current.x = startx;
	current.y = starty;
	current.h = (float)sqrt(pow(goalx-current.x,2.0)+pow(goaly-current.y,2.0));
	current.f = current.h;

	OpenCells.push(current);
cout <<" Goal is " << goalx << "," << goaly << endl;;

//cout.flush();
	while ((!OpenCells.empty()) && (!Closed[goalx][goaly])) { // while not empty and goal not evaluated 
		current = OpenCells.top(); // read top value
//cout << current.x << "," << current.y << endl;
		OpenCells.pop(); // remove from queue
		Closed[current.x][current.y] = true; // add to closed
		for (int k = 0;k<8;k++) { // check neighbors
			int evalx = current.x + dX[k];
			int evaly = current.y + dY[k];
//cout << "eval " << evalx << "," << evaly << cost_size_x << " " << cost_size_y << endl;
			if ((evalx >= 0) && (evalx < cost_size_x)) {  // within x bounds
				if ((evaly >= 0) && (evaly < cost_size_y)) { // within y bounds
					if (cost_map[evalx*cost_size_y + evaly] != OBSTACLE) { // not an obstacle
						if (!Closed[evalx][evaly]) { // not already evaluated
							if (G[evalx][evaly] > G[current.x][current.y] + stepcost[k]) {   // if smaller g value
								G[evalx][evaly] = G[current.x][current.y] + stepcost[k]; // update g
								Cell temp(evalx, evaly, G[evalx][evaly]+cost_map[evalx*cost_size_y+evaly], cost_cell_size*sqrt(pow(goalx-evalx,2.0)+pow(goaly-evaly,2.0))); // setup cell
								OpenCells.push(temp); // add to open list
							} // end smaller g
						} // end closed
					} // end obstacle
				} // end cost_size_y
			} // end x dim
		} // end check neighbors
	} // end while

	//for (int i=0;i<cost_size_x;i++) {
	//        for (int j=0;j<cost_size_y;j++) {
	//                printf("%1.1f ", G[i][j]);
	//        }
	//        cout << endl;
	//}


	vector<Traj_pt_s> invbestpath;

	if ((current.x !=goalx) || (current.y != goaly)) { cout << " $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ did not make it to goal" << endl; return; } // if did not end up at the goal, then return empty trajectory
       
//cout.flush();
	Traj_pt_s currentpt, nextpt, bestpt;

	currentpt.x = goalx;
	currentpt.y = goaly;
bestpath.clear();
	invbestpath.push_back(currentpt);
 int dir = 0;
dist_traveled = 0;

	while ((currentpt.x != startx) || (currentpt.y != starty)) {
		double Gmin = INFd;
		for (int k = 0; k < 8; k++) {
			nextpt.x = currentpt.x + dX[k];
			nextpt.y = currentpt.y + dY[k];
//cout << nextpt.x << "," << nextpt.y << endl;
			if ((nextpt.x >= 0) && (nextpt.x < cost_size_x)) {  // within x bounds
				if ((nextpt.y >= 0) && (nextpt.y < cost_size_y)) { // within y bounds
					if (cost_map[nextpt.x*cost_size_y + nextpt.y] != OBSTACLE) { // not an obstacle
						if (G[nextpt.x][nextpt.y] < Gmin) { // if smallest G of neighbors
							Gmin = G[nextpt.x][nextpt.y];
							bestpt = nextpt;
							dir = k;
						} // end g
					} // end not obst
				} // end cost_size_y
			} // end cost_size_x
		} // end check of neighbors
		invbestpath.push_back(bestpt);
		currentpt = bestpt;
		dist_traveled += stepcost[dir];
	} // end while

	// invert path
	for (int k = invbestpath.size() - 1; k > -1; k--) {
		bestpath.push_back(invbestpath[k]);
	}

	return;
}


//int main() {
// srand ( time(NULL) );
//
//
//int cost_size_x = 1000;
//int cost_size_y = 1000;
//        bool* cost_map = new bool[cost_size_x*cost_size_y];
//        unsigned char* cost_map = new unsigned char[cost_size_x*cost_size_y];
//
//        for (int i=0;i<cost_size_x;i++) {
//                for (int j=0;j<cost_size_y;j++) {
//                        cost_map[i*cost_size_y + j] = 0;
//                        cost_map[i*cost_size_y + j] = 0;
//                }
//        }
//// add obstacles
//for ( int k = 0; k < 800000; k++) {
//        int x = rand()%cost_size_x;
//        int y = rand()%cost_size_y;
//        cost_map[x*cost_size_y + y] = true;
//}
//
//for (int k = 10; k< cost_size_x; k++) {
//        cost_map[500*cost_size_y+k] = true;
//}
//
//for (int k = 0; k< cost_size_x-10; k++) {
//        cost_map[300*cost_size_y+k] = true;
//}
//
//
//
//cost_map[3*cost_size_y+3] = false;
//cost_map[800*cost_size_y+800] = false;
//
//        //for (int i=0;i<10;i++) {
//        //        for (int j=0;j<10;j++) {
//        //                cout << cost_map[i*cost_size_y + j];
//        //        }
//        //        cout << endl;
//        //}
//
//
//        vector<Traj_pt_s> path;
//
//        //for (int i=0;i<cost_size_x;i++) {
//        //        for (int j=0;j<cost_size_y;j++) {
//        //                cout << cost_map[i*cost_size_y + j];
//        //        }
//        //        cout << endl;
//        //}
//
//        path = Astarpoint(3,3,800,800, cost_map, cost_size_x, cost_size_y);
//
//
//
//if (path.size() ==0) { cout << " no path available"; return -1;}
//
//        //for (int i=0;i< path.size(); i++) {
//        //        cout << "x: " << path[i].x << " y: " << path[i].y << endl;
//        //        cost_map[path[i].x *cost_size_y + path[i].y] = 1;
//        //}
//
//        //for (int i=0;i<cost_size_x;i++) {
//        //        for (int j=0;j<cost_size_y;j++) {
//        //                cout << cost_map[i*cost_size_y + j];
//        //        }
//        //        cout << endl;
//        //}
//
//double dist = 0;
//        for (int i=0;i< path.size()-1; i++) {
//                dist += sqrt(pow(path[i].x - path[i+1].x, 2) + pow(path[i].y - path[i+1].y,2));
//}
//
//cout << "dist = " << dist << endl;
//
//delete[] cost_map;
//delete[] cost_map;
//return dist;
//}
//

