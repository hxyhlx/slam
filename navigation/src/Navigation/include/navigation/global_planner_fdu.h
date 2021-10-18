#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_FDU_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_FDU_H


#include "navigation/GLobalPlannerInterface.h"
#include <vector>
#include <ros/ros.h>

class  APoint;
class  CAstar;
namespace global_planner {

class GlobalPlannerFdu : public  global_planner::GlobalPlannerInterface
{
 public:
  
  GlobalPlannerFdu();
  ~GlobalPlannerFdu();

  virtual bool setPose(const geometry_msgs::PoseStamped& msg);
  virtual bool setGoal(const geometry_msgs::PoseStamped& msg);
  virtual void setFrame(std::string frame_id) { frame_id_ = frame_id; }
  virtual nav_msgs::Path getPathMsg() { return pathMsgsCut_;}
  virtual nav_msgs::Path getPathOriginMsg() { return pathMsgsRe_;}
  bool getGlobalPath(); 
  virtual void updateFullOctomap(octomap::OcTree* tree);
  virtual void printPath(const nav_msgs::Path& msg); 
 private:
	
	octomap::point3d convertPoseMsg2point(geometry_msgs::PoseStamped pose_msg);
	double nextYaw(octomap::point3d u, octomap::point3d v);
	geometry_msgs::PoseStamped createPoseMsg(const octomap::point3d& point, double yaw);
	nav_msgs::Path getPathMsg(const std::vector<octomap::point3d>& path);
	void  getPathRe(APoint* point);
	APoint* planPath(const octomap::OcTree &octree);
	void simplifyPath(octomap::OcTree octree);	
	void pubGps2Web();
	octomap::OcTree* octree_ = NULL;
	std::string frame_id_;
	  
  	geometry_msgs::PoseStamped cur_pos_;
 	bool cur_pos_received_{false};
 	geometry_msgs::PoseStamped cur_goal_;
  	bool cur_goal_received_{false};
	double octree_resolution_{1};

	std::vector<octomap::point3d> pathRe_; 
	std::vector<octomap::point3d> pathCut_;
	

	nav_msgs::Path pathMsgsRe_; 
	nav_msgs::Path pathMsgsCut_; 
	
	CAstar * star{NULL};
	APoint* start_init{NULL};
	APoint* goal_init{NULL};
	
};

}

#endif  // GLOBAL_PLANNER_GLOBAL_PLANNER__FDU_H




