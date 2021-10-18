#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_H

#include <math.h>     // abs
#include <algorithm>  // std::reverse
#include <limits>     // numeric_limits
#include <queue>      // std::priority_queue
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

//#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>  // getYaw createQuaternionMsgFromYaw

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

//#include <navigation/GlobalPlannerNodeConfig.h>
//#include <global_planner/PathWithRiskMsg.h>
#include "navigation/GLobalPlannerInterface.h"
#include "navigation/cell.h"
#include "navigation/common.h"
#include "navigation/node.h"
#include "navigation/search_tools.h"

namespace global_planner {

class GlobalPlanner : public  global_planner::GlobalPlannerInterface
{
 public:
 	
 GlobalPlanner();
  ~GlobalPlanner();
  virtual bool setPose(const geometry_msgs::PoseStamped& msg) ;
  virtual bool setGoal(const geometry_msgs::PoseStamped& msg) ;
  virtual void setFrame(std::string frame_id) ;
  virtual bool getGlobalPath() ; 
  virtual nav_msgs::Path getPathMsg() ;
  virtual nav_msgs::Path getPathOriginMsg(); 
  virtual void updateFullOctomap(octomap::OcTree* tree) ;  
  virtual void printPath(const nav_msgs::Path& msg) ;

  
  void getOpenNeighbors(const Cell& cell, std::vector<CellDistancePair>& neighbors, bool is_3D);
  double getEdgeDist(const Cell& u, const Cell& v);
  double getSingleCellRisk(const Cell& cell);
  double getAltPrior(const Cell& cell);
  bool isOccupied(const Cell& cell);
  bool isLegal(const Node& node);
  double getRisk(const Cell& cell);
  double getRisk(const Node& node);
  double getTurnSmoothness(const Node& u, const Node& v);
  double getEdgeCost(const Node& u, const Node& v);
  double riskHeuristic(const Cell& u, const Cell& goal);

  double smoothnessHeuristic(const Node& u, const Cell& goal);
  double altitudeHeuristic(const Cell& u, const Cell& goal);
  double getHeuristic(const Node& u, const Cell& goal);
  geometry_msgs::PoseStamped createPoseMsg(const Cell& cell, double yaw);
  nav_msgs::Path getPathMsg(const std::vector<Cell>& path);
  PathInfo getPathInfo(const std::vector<Cell>& path);
  NodePtr getStartNode(const Cell& start, const Cell& parent, const std::string& type);
  bool findPath(std::vector<Cell>& path);
  void setRobotRadius(double radius); 
  void setPath(const std::vector<Cell>& path);
public:
  octomap::OcTree* octree_ = NULL;
  std::vector<double> alt_prior_{1.0,    0.2,   0.1333, 0.1,   0.0833, 0.05,  0.033, 0.025, 0.0166,
                                 0.0125, 0.001, 0.001,  0.001, 0.001,  0.001, 0.001, 0.001, 0.001,
                                 0.001,  0.001, 0.001,  0.001, 0.001,  0.001, 0.001};


  std::unordered_map<Cell, double> risk_cache_;         // Cache of getRisk(Cell)
  std::unordered_map<Node, double> heuristic_cache_;    // Cache of

  std::unordered_set<Cell> occupied_;    // Cells which have at some point contained an obstacle point

  geometry_msgs::Point curr_pos_;
  geometry_msgs::PoseStamped curr_pose_pos_;
  geometry_msgs::PoseStamped curr_pose_goal_;
  bool cur_pos_received_{false};
  double curr_yaw_;
  bool cur_goal_received_{false};
  
  GoalCell goal_pos_= GoalCell(0.5, 0.5, 3.5);

  nav_msgs::Path pathMsgsRe_; 
  nav_msgs::Path pathMsgsCut_; 
  nav_msgs::Path pathMsgsReSmooth_; 
  nav_msgs::Path pathMsgsCutSmooth_; 
  // Dynamic reconfigure parameters
  int min_altitude_ = 1;
  int max_altitude_ = 30;
  double max_cell_risk_ = 0.5;
  double smooth_factor_ = 10.0;
  double vert_to_hor_cost_ = 1.0;  // The cost of changing between vertical and
                                   // horizontal motion (TODO: use it)
  double risk_factor_ = 10.0;
  double neighbor_risk_flow_ = 1.0;
  double explore_penalty_ = 0.8; //0.005
  double up_cost_ = 2.0;
  double down_cost_ = 1.0;
  double search_time_ = 0.5;  // The time it takes to find a path in worst case
  double min_overestimate_factor_ = 1.03;
  double max_overestimate_factor_ = 2; //2

  int max_iterations_ = 1000000;
  bool goal_is_blocked_ = false;
  bool current_cell_blocked_ = false;
  bool goal_must_be_free_ = true;  // If false, the planner may try to find a path close to the goal
  bool use_current_yaw_ = false;    // The current orientation is factored into the smoothness
  bool use_risk_heuristics_ = false;

  std::string default_node_type_ = "Node";
  std::string frame_id_ = "/map";

  double overestimate_factor_ = max_overestimate_factor_;
  std::vector<Cell> curr_path_;
  PathInfo curr_path_info_;
  
  double robot_radius_{0.5};
  double octree_resolution_;
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_GLOBAL_PLANNER_H
