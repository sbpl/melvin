/*********************************************************************
*
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
*
* Author: Mike Phillips
*********************************************************************/

#include <exploration_planner/exploration_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>

using namespace std;
using namespace ros;


PLUGINLIB_REGISTER_CLASS(ExplorationPlanner, ExplorationPlanner, nav_core::BaseGlobalPlanner);

ExplorationPlanner::ExplorationPlanner()
  : initialized_(false), costmap_ros_(NULL){
}

ExplorationPlanner::ExplorationPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
  : initialized_(false), costmap_ros_(NULL){
  initialize(name, costmap_ros);
}


void ExplorationPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  if(!initialized_){
    ros::NodeHandle private_nh("~/"+name);
    ros::NodeHandle nh(name);
    
    ROS_INFO("Name is %s", name.c_str());

    double linear_velocity, angular_velocity;
    private_nh.param("linear_velocity", linear_velocity, 1.0);
    private_nh.param("angular_velocity", angular_velocity, 3.14);
    
    costmap_ros_ = costmap_ros;
    costmap_ros_->clearRobotFootprint();
    costmap_ros_->getCostmapCopy(cost_map_);

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

    

    //INIT PLANNER
    planner = new GPLAN();

    GP_MAP_DATA map_data;
    map_data.timestamp = ros::Time::now().toSec();
    map_data.cost_size_x = costmap_ros_->getSizeInCellsX();
    map_data.cost_size_y = costmap_ros_->getSizeInCellsY();
    map_data.elev_size_x = costmap_ros_->getSizeInCellsX();
    map_data.elev_size_y = costmap_ros_->getSizeInCellsY();
    map_data.coverage_size_x = costmap_ros_->getSizeInCellsX();
    map_data.coverage_size_y = costmap_ros_->getSizeInCellsY();
    map_data.cost_cell_size = costmap_ros_->getResolution();
    map_data.elev_cell_size = costmap_ros_->getResolution();
    map_data.coverage_cell_size = costmap_ros_->getResolution();

    GP_ROBOT_PARAMETER robot_params;
    robot_params.MAX_VELOCITY = linear_velocity; // maximum velocity in m/s
    robot_params.MAX_TURN_RATE = angular_velocity; // maximum turn rate in radians per second
    //robot_params.I_DIMENSION = footprint.size(); // number of points in perimeter
    robot_params.J_DIMENSION = 2; // number of dimensions (should be 2)
    robot_params.sensor_radius = 30; // sensing radius of robot in m
    robot_params.sensor_height = 33; // sensing height of robot in cm
/*
    robot_params.PerimeterArray = new double[robot_params.I_DIMENSION*robot_params.J_DIMENSION];
    for(int q=0; q<robot_params.I_DIMENSION; q++){
      robot_params.PerimeterArray[q*robot_params.J_DIMENSION] = footprint[q].x;
      robot_params.PerimeterArray[q*robot_params.J_DIMENSION+1] = footprint[q].y;
    }
*/
    robot_params.I_DIMENSION = 1; // number of points in perimeter
    robot_params.PerimeterArray = new double[2];
    robot_params.PerimeterArray[0] = 0;
    robot_params.PerimeterArray[1] = 0.60;

    GP_FULL_UPDATE full_update;
    full_update.timestamp = ros::Time::now().toSec();
    full_update.sent_cover_x = costmap_ros_->getSizeInCellsX();
    full_update.sent_cover_y = costmap_ros_->getSizeInCellsY();
    full_update.sent_cost_x = costmap_ros_->getSizeInCellsX();
    full_update.sent_cost_y = costmap_ros_->getSizeInCellsY();
    full_update.sent_elev_x = costmap_ros_->getSizeInCellsX();
    full_update.sent_elev_y = costmap_ros_->getSizeInCellsY();
    full_update.coverage_map = new unsigned char[costmap_ros_->getSizeInCellsX()*costmap_ros_->getSizeInCellsY()];
    full_update.cost_map = new unsigned char[costmap_ros_->getSizeInCellsX()*costmap_ros_->getSizeInCellsY()];
    full_update.elev_map = new int16_t[costmap_ros_->getSizeInCellsX()*costmap_ros_->getSizeInCellsY()];
    for(unsigned int x=0; x < costmap_ros_->getSizeInCellsX(); x++){
      for(unsigned int y=0; y < costmap_ros_->getSizeInCellsY(); y++){
        full_update.coverage_map[x + costmap_ros_->getSizeInCellsX()*y] = (cost_map_.getCost(x,y) == costmap_2d::NO_INFORMATION ? 0: 249);
        full_update.cost_map[x + costmap_ros_->getSizeInCellsX()*y] = (cost_map_.getCost(x,y) >= costmap_2d::LETHAL_OBSTACLE ? 250 : 0);
        full_update.elev_map[x + costmap_ros_->getSizeInCellsX()*y] = (cost_map_.getCost(x,y) >= costmap_2d::LETHAL_OBSTACLE ? 1000 : 0);
      }
    }
      
    planner->gplan_init(&map_data, &robot_params, &full_update);

    ROS_INFO("[exploration_planner] Initialized successfully");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    
    initialized_ = true;
  }
}
  
bool ExplorationPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){
  plan.clear();

  ROS_INFO("[exploration_planner] getting fresh copy of costmap");
  costmap_ros_->clearRobotFootprint();
  ROS_INFO("[exploration_planner] robot footprint cleared");

  costmap_ros_->getCostmapCopy(cost_map_);

  ROS_INFO("[exploration_planner] getting start point (%g,%g) goal point (%g,%g)",
           start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);

  double start_x = start.pose.position.x - cost_map_.getOriginX();
  double start_y = start.pose.position.y - cost_map_.getOriginY();
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);

  ROS_INFO("[exploration_planner] run planner");

  //CALL PLANNER
  GP_POSITION_UPDATE pos_update;
  pos_update.timestamp = ros::Time::now().toSec();
  pos_update.x = start_x;
  pos_update.y = start_y;
  pos_update.theta = theta_start;

  GP_SHORT_UPDATE short_update;
  short_update.timestamp = ros::Time::now().toSec();
  short_update.sent_cover_x = costmap_ros_->getSizeInCellsX();
  short_update.sent_cover_y = costmap_ros_->getSizeInCellsY();
  short_update.sent_cost_x = costmap_ros_->getSizeInCellsX();
  short_update.sent_cost_y = costmap_ros_->getSizeInCellsY();
  short_update.sent_elev_x = costmap_ros_->getSizeInCellsX();
  short_update.sent_elev_y = costmap_ros_->getSizeInCellsY();
  short_update.x_cover_start = 0;
  short_update.y_cover_start = 0;
  short_update.x_cost_start = 0;
  short_update.y_cost_start = 0;
  short_update.x_elev_start = 0;
  short_update.y_elev_start = 0;
  short_update.coverage_map = new unsigned char[cost_map_.getSizeInCellsX()*cost_map_.getSizeInCellsY()];
  short_update.cost_map = new unsigned char[cost_map_.getSizeInCellsX()*cost_map_.getSizeInCellsY()];
  short_update.elev_map = new int16_t[cost_map_.getSizeInCellsX()*cost_map_.getSizeInCellsY()];
  for(unsigned int x=0; x < cost_map_.getSizeInCellsX(); x++){
    for(unsigned int y=0; y < cost_map_.getSizeInCellsY(); y++){
      short_update.coverage_map[x + cost_map_.getSizeInCellsX()*y] = (cost_map_.getCost(x,y) == costmap_2d::NO_INFORMATION ? 0: 249);
      short_update.cost_map[x + cost_map_.getSizeInCellsX()*y] = (cost_map_.getCost(x,y) >= costmap_2d::LETHAL_OBSTACLE ? 250 : 0);
      short_update.elev_map[x + cost_map_.getSizeInCellsX()*y] = (cost_map_.getCost(x,y) >= costmap_2d::LETHAL_OBSTACLE ? 1000 : 0);
    }
  }

  vector<Traj_pt_s> gplan_path = planner->gplan_plan(&pos_update, &short_update);
  
  ROS_INFO("size of solution=%d", (int)gplan_path.size());
  ros::Time plan_time = ros::Time::now();

  //create a message for the plan 
  nav_msgs::Path gui_path;
  gui_path.poses.resize(gplan_path.size());
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = plan_time;

  unsigned trim_size;
  if(gplan_path.size() > 20)
    trim_size = gplan_path.size() - 20;
  else
    trim_size = gplan_path.size();

  for(unsigned int i=0; i<trim_size; i++){
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();

    pose.pose.position.x = gplan_path[i].xx + cost_map_.getOriginX();
    pose.pose.position.y = gplan_path[i].yy + cost_map_.getOriginY();
    pose.pose.position.z = start.pose.position.z;

    tf::Quaternion temp;
    temp.setEulerZYX(gplan_path[i].theta,0,0);
    pose.pose.orientation.x = temp.getX();
    pose.pose.orientation.y = temp.getY();
    pose.pose.orientation.z = temp.getZ();
    pose.pose.orientation.w = temp.getW();

    plan.push_back(pose);

    gui_path.poses[i].pose.position.x = plan[i].pose.position.x;
    gui_path.poses[i].pose.position.y = plan[i].pose.position.y;
    gui_path.poses[i].pose.position.z = plan[i].pose.position.z;
  }
  plan_pub_.publish(gui_path);

  return true;
}

