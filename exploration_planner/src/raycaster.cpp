#include <exploration_planner/common_headers.h>
//#include "raycaster.h"


/* Function written by Jonathan Michael Butzke
 *inputs are map, location and obstacles
 * outputs: score
 * uses Bresenham line algorithm for rays
 *last updated 28 Jun 2009
 */

using namespace std;

//
//bool cast_old_single_ray(int x0, int y0, int x1, int y1, int & xend, int & yend, int & score, unsigned char cover_map[],  unsigned char cost_map[])  {
//    int xhat, yhat; // current position being checked
//    int cx, deltax, xstep, xt; // copy, delta & step size, temp
//    int cy, deltay, ystep, yt;
//    int error, st; // error and steep flag
//
//    // find largest delta for pixel steps
//    st = (abs(y1 - y0) > abs(x1 - x0)); // st is true if steep
//
//    // if deltay > deltax then swap x,y
//    if (st) {   (xt = y0); (y0 = x0); (x0 = xt); // swap(x0, y0);
//        (yt = y1); (y1 = x1); (x1 = yt);   } // swap(x1, y1);
//
//    deltax = abs(x1 - x0);
//    deltay = abs(y1 - y0);
//    error  = (deltax / 2);
//    yhat = y0;
//
//    // set steps in correct direction
//    if (x0 > x1) { xstep = -1; }
//    else         { xstep =  1; }
//
//    if (y0 > y1) { ystep = -1; }
//    else         { ystep =  1; }
//
//    for ((xhat = x0); (xhat != (x1 + xstep)); (xhat += xstep))  {
//        // if x,y swapped above, swap them back now
//        if (st) { (cx = yhat); (cy = xhat); }
//        else { cx = xhat; cy = yhat; }
//
//        //check to see if map grid is occupied
//        switch(cover_map[cx*coverage_size_y + cy]) 	{
//            case 255:
//            case 254:
//            case 253:
//            case 252:   
//            case KNOWN: {
//                    break;  
//                    }
//            case UNKNOWN:  {
//                       cover_map[cx*coverage_size_y+cy] = KNOWN;
//                       score += UNKSCORE;
//                       break; 
//                       }
//            case OBSTACLE: { xend = cx; yend = cy;  return true; } 
//            default: {
//                     score += cover_map[cx*coverage_size_y+cy];
//                     cover_map[cx*coverage_size_y+cy] = KNOWN;
//                     break;  
//                     }
//        } // switch on map
//
//        // take step
//        (error -= deltay); // converge toward end of line
//
//        if (error < 0) { // not done yet
//            (yhat += ystep);
//            (error += deltax);
//        } // if error
//    } //for xhat
//return false;
//} // cast single ray
//
//

void GPLAN::cast_single_ray(int x0, int y0, int x1, int y1, int & score, unsigned char cover_map[], const int16_t elev[])  {
	// function performs Bresenham line algorithm but enforces 4 connected movement instead of 8.	

	int xhat, yhat; // current position being checked
	int cx, deltax, xstep, xt; // copy, delta & step size, temp
	int cy, deltay, ystep, yt;
	int error, st; // error and steep flag
	int16_t height = elev[x0+ cost_size_x*y0] + sensor_height;

//xend = 0; yend = 0;

	// find largest delta for pixel steps
	st = (abs(y1 - y0) > abs(x1 - x0)); // st is true if steep

	// if deltay > deltax then swap x,y
	if (st) {   (xt = y0); (y0 = x0); (x0 = xt); // swap(x0, y0);
		(yt = y1); (y1 = x1); (x1 = yt);   } // swap(x1, y1);

	deltax = abs(x1 - x0);
	deltay = abs(y1 - y0);
	error  = (deltax / 2);
	yhat = y0;

	// set steps in correct direction
	if (x0 > x1) { xstep = -1; }
	else         { xstep =  1; }

	if (y0 > y1) { ystep = -1; }
	else         { ystep =  1; }

	for ((xhat = x0); (xhat != (x1 + xstep)); (xhat += xstep))  {
		// if x,y swapped above, swap them back now
		if (st) { (cx = yhat); (cy = xhat); }
		else { cx = xhat; cy = yhat; }

		//check to see if map grid is occupied
		if (height-elev[cx+cost_size_x*cy]>0) {
			score += KNOWN - cover_map[cx+coverage_size_x*cy];
			cover_map[cx+coverage_size_x*cy] = KNOWN;
		}
		else {
			//xend = cx; yend = cy;
			return;
		}

		//switch(cost_map[cx+cost_size_x*cy]) 	{
		//case 255:
		//case 254:
		//case 253:
		//case 252:  {
		//break;  
		//}
		//case OBSTACLE: { xend = cx; yend = cy;	
		//cout << " OBST SCORE: " << xend << "," << yend << ":" << score << endl;  
		//return true; } 
		//default: {
		//score += KNOWN - cover_map[cx+coverage_size_x*cy];
		//cover_map[cx+coverage_size_x*cy] = KNOWN;
		//break;  
		//}
		//} // switch on map

		// take step
		(error -= deltay); // converge toward end of line

		if (error < 0) { // not done yet

			(error += deltax);
			// if x,y swapped above, swap them back now
			if  ((xhat != x1) && (yhat != y1)) {
				xhat +=xstep;
				if (st) { (cx = yhat); (cy = xhat); }
				else { cx = xhat; cy = yhat; }
				xhat -=xstep;
				//check to see if map grid is occupied
				if (height-elev[cx+cost_size_x*cy]>0) {
					score += KNOWN - cover_map[cx+coverage_size_x*cy];
					cover_map[cx+coverage_size_x*cy] = KNOWN;
				}
				else {
					//xend = cx; yend = cy;
					return;
				}
				//switch(cost_map[cx+cost_size_x*cy]) 	{
				//    case 255:
				//    case 254:
				//    case 253:
				//    case 252:  {
				//        break;  
				//        }
				//    case OBSTACLE: { xend = cx; yend = cy;
				//                       //cout << " obst2 SCORE: "<< xend << "," << yend << ":" << score << endl; 
				//                       return true; } 
				//    default: {
				//        score += KNOWN - cover_map[cx+coverage_size_x*cy];
				//        cover_map[cx+coverage_size_x*cy] = KNOWN;
				//        break;  
				//        }
				//} // switch on map
			}
			(yhat += ystep);
		} // if error
	} //for xhat
	//cout << " SCORE: "<< xend << "," << yend << ":" << score << endl;
	return;
} // cast modified single ray

int GPLAN::cast_all_rays(int x0, int y0, unsigned char cover_map[], const int16_t elev_map[], int start_vec, int end_vec) {
	// input is position, maps, and limits of sweep
	// returns score, updates coverage map

	//int xend = x0+rayendpts[0].x, yend= y0+rayendpts[0].y; // obstacle locations
	int score = 0;	// score of casts = number of newly seen points
	//bool hit_obs = false;  // temp variable to hold return value for checking

	int current_vec = start_vec;

	while (current_vec != end_vec) {
		// determine all of the rays and for each ray add new points to list, update map
		//for (int i =0; i<NUMVECTORS; i++) {
		// set goal point to current endpoint and quadrant
			int x1 = x0 + rayendpts[current_vec].x;
		int y1 = y0 + rayendpts[current_vec].y;
		cast_single_ray(x0, y0, x1, y1, score, cover_map, elev_map);
		current_vec++;
			if (current_vec == NUMVECTORS) {current_vec = 0; }
	}
	return score;
} // cast all rays


