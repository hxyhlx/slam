#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_INTERFACE_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_INTERFACE_H

#include <string>
#include <cmath>
#include <iomanip>

#include <nav_msgs/Path.h>
#include "octomap_msgs/Octomap.h"
#include "octomap/ColorOcTree.h"
#include "octomap_msgs/conversions.h"

#include <geometry_msgs/PoseStamped.h>



namespace global_planner {

class GlobalPlannerInterface{
 public:
  
  GlobalPlannerInterface() {}
  ~GlobalPlannerInterface() {}
  
  virtual bool setPose(const geometry_msgs::PoseStamped& msg) = 0;
  virtual bool setGoal(const geometry_msgs::PoseStamped& msg) = 0;
  virtual void setFrame(std::string frame_id) = 0;
  virtual bool getGlobalPath() = 0; 
  virtual nav_msgs::Path getPathMsg() = 0;
  
  virtual nav_msgs::Path getPathOriginMsg() = 0;
  	
  virtual void updateFullOctomap(octomap::OcTree* tree) = 0;
  
  virtual void printPath(const nav_msgs::Path& msg) = 0;

};


}

#endif  // GLOBAL_PLANNER_GLOBAL_PLANNER__INTERFACE_H




