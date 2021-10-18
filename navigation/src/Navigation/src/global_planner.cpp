#include "navigation/global_planner.h"

namespace global_planner {

// Returns the XY-angle between u and v, or if v is directly above/below u, it
// returns last_yaw
double nextYaw(Cell u, Cell v, double last_yaw) {
  int dx = v.xIndex() - u.xIndex();
  int dy = v.yIndex() - u.yIndex();
  if (dx == 0 && dy == 0) {
    return last_yaw;  // Going up or down
  }
  return std::atan2(dy, dx);
}

GlobalPlanner::GlobalPlanner() {  }
GlobalPlanner::~GlobalPlanner() {}


// Updates the current pose and keeps track of the path back
bool GlobalPlanner::setPose(const geometry_msgs::PoseStamped& new_pose) {
  curr_pose_pos_ = new_pose;
  curr_pos_ = new_pose.pose.position;
  curr_yaw_ = tf::getYaw(new_pose.pose.orientation);

  Cell s = Cell(curr_pos_); 
  current_cell_blocked_ = isOccupied(s); 
  if (current_cell_blocked_) 
  {
	ROS_INFO("Current position is occupied.");
	return false;
  }
  cur_pos_received_ = true;
  return true;

}

// Sets a new mission goal
bool GlobalPlanner::setGoal(const geometry_msgs::PoseStamped& msg){
	GoalCell goal = GoalCell(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 2.0);
	goal_pos_ = goal;
	
	Cell t = Cell(goal_pos_);
	if (goal_must_be_free_ && getRisk(t) > max_cell_risk_) 
	{
	  // If goal is occupied, no path is published
	  ROS_INFO("Goal position is occupied, risk of t: %3.2f", getRisk(t));
	  goal_is_blocked_ = true;
	  return false;
	}

	curr_pose_goal_ = msg;
	goal_is_blocked_ = false;
	cur_goal_received_ = true;
	return true;
}


void GlobalPlanner::setFrame(std::string frame_id) { frame_id_ = frame_id; }

// Returns false iff current path has an obstacle
// Going through the octomap can take more than 50 ms for 100m x 100m explored
// map

 void GlobalPlanner::updateFullOctomap(octomap::OcTree* tree)
 {
   risk_cache_.clear();
   if (octree_) 
   {
   	delete octree_;
   }
   octree_ = tree;
   octree_resolution_ = octree_->getResolution();
}

// TODO: simplify and return neighbors
// Fills neighbors with the 8 horizontal and 2 vertical non-occupied neigbors
void GlobalPlanner::getOpenNeighbors(const Cell& cell, std::vector<CellDistancePair>& neighbors, bool is_3D) {
  // It's long because it uses the minimum number of 'if's
  double x = cell.xIndex();
  double y = cell.yIndex();
  double z = cell.zIndex();
  Cell forw = Cell(std::tuple<int, int, int>(x + 1, y, z));
  Cell back = Cell(std::tuple<int, int, int>(x - 1, y, z));
  Cell left = Cell(std::tuple<int, int, int>(x, y - 1, z));
  Cell righ = Cell(std::tuple<int, int, int>(x, y + 1, z));
  Cell forw_left = Cell(std::tuple<int, int, int>(x + 1, y - 1, z));
  Cell forw_right = Cell(std::tuple<int, int, int>(x + 1, y + 1, z));
  Cell back_left = Cell(std::tuple<int, int, int>(x - 1, y - 1, z));
  Cell back_right = Cell(std::tuple<int, int, int>(x - 1, y + 1, z));
  Cell up = Cell(std::tuple<int, int, int>(x, y, z + 1));
  Cell down = Cell(std::tuple<int, int, int>(x, y, z - 1));

  neighbors.push_back(std::make_pair(forw, 1.0));
  neighbors.push_back(std::make_pair(forw_left, 1.41));
  neighbors.push_back(std::make_pair(forw_right, 1.41));
  neighbors.push_back(std::make_pair(back, 1.0));
  neighbors.push_back(std::make_pair(back_left, 1.41));
  neighbors.push_back(std::make_pair(back_right, 1.41));
  neighbors.push_back(std::make_pair(left, 1.0));
  neighbors.push_back(std::make_pair(righ, 1.0));
  // Vertical neighbors
  if (is_3D && z < max_altitude_) {
    neighbors.push_back(std::make_pair(up, up_cost_));
  }
  if (is_3D && z > min_altitude_) {
    neighbors.push_back(std::make_pair(down, down_cost_));
  }
}


// The distance between two adjacent cells
double GlobalPlanner::getEdgeDist(const Cell& u, const Cell& v) {
  double xy_diff = u.distance2D(v);
  double z_diff = v.zPos() - u.zPos();
  double up_diff = up_cost_ * std::max(z_diff, 0.0);
  double down_diff = down_cost_ * std::max(-z_diff, 0.0);
  return xy_diff + up_diff + down_diff;
}

// Risk without looking at the neighbors
double GlobalPlanner::getSingleCellRisk(const Cell& cell) {
  if (cell.zIndex() < 1 || !octree_) {
    return 1.0;  // Octomap does not keep track of the ground
  }
  // octomap::OcTreeNode* node = octree_->search(cell.xPos(), cell.yPos(),
  // cell.zPos());
  int octree_depth = std::min(16, 17 - int(CELL_SCALE + 0.1));
  octomap::OcTreeNode* node = octree_->search(cell.xPos(), cell.yPos(), cell.zPos(), octree_depth);
  if (node) {
    // TODO: posterior in log-space
    double log_odds = node->getValue();
    // double parentLogOdds = parent->getValue();
 //   double post_prob = posterior(getAltPrior(cell), octomap::probability(log_odds));
    // double post_prob = posterior(0.06, octomap::probability(log_odds));
    // // If the cell has been seen
    double post_prob = octomap::probability(log_odds);
    if (occupied_.find(cell) != occupied_.end()) {
      // If an obstacle has at some point been spotted it is 'known space'
      return post_prob;
    } else if (log_odds > 0) {
      // ROS_INFO("Cell %s is not in occupied_ but has > 50\% risk",
      // cell.asString().c_str());
      return post_prob;
    }
    // No obstacle spotted (all measurements hint towards it being free)
    return explore_penalty_ * post_prob;
  }
  // No measurements at all
  return explore_penalty_ * getAltPrior(cell);  // Risk for unexplored cells
}

double GlobalPlanner::getAltPrior(const Cell& cell) {
  int index = std::round(cell.zPos());

  if (index > alt_prior_.size() - 1) {
    return alt_prior_.back();
  } else if (index < 10) {
    return alt_prior_.front();
  } else {
    return alt_prior_.at(index);
  }
}

bool GlobalPlanner::isOccupied(const Cell& cell) { return getSingleCellRisk(cell) > 0.5; }

bool GlobalPlanner::isLegal(const Node& node) {
  bool temp = getRisk(node) < max_cell_risk_;
  if(!temp)
  {
  	//std::cout << "!!!!!!! getRisk = " << std::setprecision(3) << getRisk(node) << std::endl;
	//ROS_INFO("curr_pos_: %2.2f,%2.2f,%2.2f\t\n ", node.cell_.xPos(), node.cell_.yPos(), node.cell_.zPos());
  }
  return node.cell_.zPos() < max_altitude_ && temp;
  
}

//TODO:求和改为max？
double GlobalPlanner::getRisk(const Cell& cell) {
  if (risk_cache_.find(cell) != risk_cache_.end()) {
     return risk_cache_[cell];    //每次地图更新，risk_cache_进行清零
  }

  double risk = getSingleCellRisk(cell);
//  ROS_INFO("!!!!!!!Goal position is occupied, risk of t: %3.2f", risk);
 // ROS_INFO("curr_pos_: %2.2f,%2.2f,%2.2f\t\n ", cell.xPos(), cell.yPos(), cell.zPos()); 
  int radius = static_cast<int>(std::ceil(robot_radius_ / octree_resolution_));
  double temp ;
  for (const Cell& neighbor : cell.getFlowNeighbors(radius)) {
  	temp = getSingleCellRisk(neighbor);
	if(temp > risk)
	{
       risk = temp;
	 //  if(risk > 0.5)
	   // ROS_INFO("####curr_pos_: %2.2f,%2.2f,%2.2f\t\n ", cell.xPos(), cell.yPos(), cell.zPos());  	
	}
	
  }
  risk_cache_[cell] = risk;
  return risk;
}



//TODO:引入自适应因子k，当风险超过0.5时，系数急剧增大
double GlobalPlanner::getRisk(const Node& node) {
  double risk = 0.0;
  std::unordered_set<Cell> nodeCells = node.getCells();
  
  if(node.cell_ == node.parent_)
  {
    return getRisk(node.cell_);
  }
  
  for (const Cell& cell : nodeCells) {
    risk += getRisk(cell);
  }
  return risk / nodeCells.size() * node.getLength();
}


// Returns the risk of the quadratic Bezier curve defined by poses
// TODO: think about this


// Returns the amount of rotation needed to go from u to v
//TODO:选取代价函数，当前是平方函数（后续指数函数？）
double GlobalPlanner::getTurnSmoothness(const Node& u, const Node& v) {
  double turn = u.getRotation(v);
  return turn * turn;  // Squaring makes large turns more costly
}

// Returns the total cost of the edge from u to v
double GlobalPlanner::getEdgeCost(const Node& u, const Node& v) {
  double dist_cost = getEdgeDist(u.cell_, v.cell_);
  // double risk_cost = u.cell_.distance3D(v.cell_) * risk_factor_ *
  // getRisk(v.cell_);
  double risk_cost = risk_factor_ * getRisk(v) * 1;
  double smooth_cost = smooth_factor_ * getTurnSmoothness(u, v);

  return dist_cost + risk_cost + smooth_cost;
}

// Returns a heuristic for the cost of risk for going from u to goal
// The heuristic is the cost of risk through unknown environment
double GlobalPlanner::riskHeuristic(const Cell& u, const Cell& goal) {
  // return riskHeuristicReverseCache(u, goal);  // REVERSE_SEARCH

  if (u == goal) {
    return 0.0;
  }
  double unexplored_risk = (1.0 + 6.0 * neighbor_risk_flow_) * explore_penalty_ * risk_factor_;
  double xy_dist = u.diagDistance2D(goal) - 1.0;  // XY distance excluding the goal cell
  double xy_risk = xy_dist * unexplored_risk * getAltPrior(u);
  double z_risk =
      unexplored_risk ; //* std::abs(accumulated_alt_prior_[u.zIndex()] - accumulated_alt_prior_[goal.zIndex()])
  // TODO: instead of subtracting 1 from the xy_dist, subtract 1 from the
  // combined xy and z dist
  double goal_risk = getRisk(goal) * risk_factor_;
  return xy_risk + z_risk + goal_risk;
}


// Returns a heuristic for the cost of turning for going from u to goal
double GlobalPlanner::smoothnessHeuristic(const Node& u, const Cell& goal) {
  if (u.cell_.xIndex() == goal.xIndex() && u.cell_.yIndex() == goal.yIndex()) {
    return 0.0;  // directly above or below the goal
  }
  if (u.cell_.xIndex() == u.parent_.xIndex() && u.cell_.yIndex() == u.parent_.yIndex()) {
    // Vertical motion not directly above or below the goal, must change to
    // horizontal movement
    return smooth_factor_ * vert_to_hor_cost_;
  }

  double u_ang = (u.cell_ - u.parent_).angle();  // Current orientation
  double goal_ang = (goal - u.cell_).angle();    // Direction of goal
  double ang_diff = goal_ang - u_ang;
  ang_diff = std::fabs(angleToRange(ang_diff));     // Rotation needed
  double num_45_deg_turns = ang_diff / (M_PI / 4);  // Minimum number of 45-turns to goal

  // If there is height difference we also need to change to vertical movement
  // at least once
  // TODO: simplify
  int altitude_change = (u.cell_.zIndex() == goal.zIndex()) ? 0 : 1;

  return smooth_factor_ * (num_45_deg_turns + altitude_change);
}

// Returns a heuristic for the cost of reaching the altitude of goal
double GlobalPlanner::altitudeHeuristic(const Cell& u, const Cell& goal) {
  double diff = goal.zIndex() - u.zIndex();
  // Either multiply by up_cost_ or down_cost_, depending on if we are belove or
  // above the goal
  double cost = (diff > 0) ? (up_cost_ * std::abs(diff)) : (down_cost_ * std::abs(diff));
  return cost;
}

// Returns a heuristic of going from u to goal
double GlobalPlanner::getHeuristic(const Node& u, const Cell& goal) {
  if (heuristic_cache_.find(u) != heuristic_cache_.end()) {
    // return heuristic_cache_[u];    // TODO: Needs to account for different
    // overestimating factors
  }

  // Only overestimate the distance
  double heuristic = overestimate_factor_ * u.cell_.diagDistance2D(goal);
  heuristic += altitudeHeuristic(u.cell_, goal);  // Lower bound cost due to altitude change
  heuristic += smoothnessHeuristic(u, goal);      // Lower bound cost due to turning
  if (use_risk_heuristics_) {
    heuristic += riskHeuristic(u.cell_, goal);  // Risk through a straight-line path of unexplored space
  }

  heuristic_cache_[u] = heuristic;
  return heuristic;
}

geometry_msgs::PoseStamped GlobalPlanner::createPoseMsg(const Cell& cell, double yaw) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = frame_id_;
  pose_msg.pose.position = cell.toPoint();
  pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return pose_msg;
}


nav_msgs::Path GlobalPlanner::getPathMsg() 
{ 

   return pathMsgsCutSmooth_; 
}

nav_msgs::Path GlobalPlanner::getPathOriginMsg()
{
	pathMsgsRe_ = getPathMsg(curr_path_);

	std::vector<Cell> path_cut = simplifyPath(this, curr_path_,1.01,10);
	pathMsgsCut_ = getPathMsg(path_cut);
	printPath(pathMsgsRe_);
	printPath(pathMsgsCut_);
	std::cout << std::endl << std::endl;

	
	PathInfo path_info = getPathInfo(curr_path_);
      printf("(cost: %2.2f, dist: %2.2f, risk: %2.2f, smooth: %2.2f) \n", path_info.cost, path_info.dist,
             path_info.risk, path_info.smoothness);
	  
	 path_info = getPathInfo(path_cut);
	
      printf("(cost: %2.2f, dist: %2.2f, risk: %2.2f, smooth: %2.2f) \n", path_info.cost, path_info.dist,
             path_info.risk, path_info.smoothness);

	pathMsgsReSmooth_ = smoothPath(pathMsgsRe_);
    pathMsgsCutSmooth_ = smoothPath(pathMsgsCut_);
	
	return pathMsgsRe_;
}

nav_msgs::Path GlobalPlanner::getPathMsg(const std::vector<Cell>& path) {
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = frame_id_;

  if (path.size() == 0) {
    return path_msg;
  }

  // Use actual position instead of the center of the cell
  double last_yaw = curr_yaw_;
  path_msg.poses.push_back(curr_pose_pos_);
  for (int i = 1; i < path.size() - 1; ++i) {
    Cell p = path[i];
    double new_yaw = nextYaw(p, path[i + 1], last_yaw);
    // if (new_yaw != last_yaw) {   // only publish corner points
    path_msg.poses.push_back(createPoseMsg(p, new_yaw));
    // }
    last_yaw = new_yaw;
  }
  Cell last_point = path[path.size() - 1];  // Last point should have the same yaw as the previous point
  path_msg.poses.push_back(createPoseMsg(last_point, last_yaw));
  path_msg.poses.push_back(curr_pose_goal_);
  
  return path_msg;
}

/*
PathWithRiskMsg GlobalPlanner::getPathWithRiskMsg() {
  nav_msgs::Path path_msg = getPathMsg();
  PathWithRiskMsg risk_msg;
  risk_msg.header = path_msg.header;
  risk_msg.poses = path_msg.poses;

  for (const auto& pose : path_msg.poses) {
    double risk = getRisk(Cell(pose.pose.position));
    risk_msg.risks.push_back(risk);
  }
  return risk_msg;
}
*/
// Returns details of the cost of the path
PathInfo GlobalPlanner::getPathInfo(const std::vector<Cell>& path) {
  PathInfo path_info = {};
  for (int i = 2; i < path.size(); ++i) {
    Node curr_node = Node(path[i], path[i - 1]);
    Node last_node = Node(path[i - 1], path[i - 2]);
    double cell_risk = getRisk(curr_node);
    path_info.dist += getEdgeDist(last_node.cell_, curr_node.cell_);
    path_info.risk += risk_factor_ * cell_risk;
	std::cout << cell_risk << "  ";
    path_info.cost += getEdgeCost(last_node, curr_node);
    path_info.is_blocked |= cell_risk > max_cell_risk_;
    path_info.smoothness += smooth_factor_ * getTurnSmoothness(last_node, curr_node);
  }
  std::cout << std::endl;
  return path_info;
}

// Chooses the node-type of the search
NodePtr GlobalPlanner::getStartNode(const Cell& start, const Cell& parent, const std::string& type) {
  if (type == "Node") {
    return NodePtr(new Node(start, parent));
  }
  if (type == "NodeWithoutSmooth") {
    return NodePtr(new NodeWithoutSmooth(start, parent));
  }
  if (type == "SpeedNode") {
    return NodePtr(new SpeedNode(start, parent));
  } else {
    return NodePtr(new Node(start, parent));
  }
}

// Calls different search functions to find a path
//TODO:实时需要考虑当前位置的时延
bool GlobalPlanner::findPath(std::vector<Cell>& path) {
  // Start from a position thats a bit ahead [s = curr_pos + (search_time_ *
  // curr_vel_)]
  Cell s(curr_pos_);
  GoalCell t = goal_pos_;

  Cell parent_of_s = s;  // Ignore the current yaw
 
  ROS_INFO("Planning a path from %s to %s", s.asString().c_str(), t.asString().c_str());
  ROS_INFO("curr_pos_: %2.2f,%2.2f,%2.2f\t s: %2.2f,%2.2f,%2.2f", curr_pos_.x, curr_pos_.y, curr_pos_.z, s.xPos(),
           s.yPos(), s.zPos());

  bool found_path = false;
  double best_path_cost = INFINITY;
  overestimate_factor_ = max_overestimate_factor_;
  int iter_left = max_iterations_;
  int last_iter = 0;

  // reverseSearch(t); // REVERSE_SEARCH

  printf("Search              iter_time overest   num_iter  path_cost \n");
 // while (overestimate_factor_ >= min_overestimate_factor_ && iter_left > last_iter) 
 {
    std::vector<Cell> new_path;
    SearchInfo search_info;
    std::string node_type = default_node_type_;
    if (overestimate_factor_ > 1.5) {
      // Use a cheap search for higher overestimate
      // found_new_path = findPathOld(new_path, s, t, parent_of_s, true);  // No
      // need to search with smoothness
      node_type = "Node";
    }

    NodePtr start_node = getStartNode(s, parent_of_s, node_type);
    search_info = findSmoothPath(this, new_path, start_node, t, iter_left);
    printSearchInfo(search_info, node_type, overestimate_factor_);

    if (search_info.found_path) {
      PathInfo path_info = getPathInfo(new_path);
   //   printf("(cost: %2.2f, dist: %2.2f, risk: %2.2f, smooth: %2.2f) \n", path_info.cost, path_info.dist,
   //          path_info.risk, path_info.smoothness);
      if (true || path_info.cost <= best_path_cost) {
        // TODO: always use the newest path?
        best_path_cost = path_info.cost;
        path = new_path;
        found_path = true;
      }
    } 
 //   last_iter = search_info.num_iter;
 //   iter_left -= last_iter;
 //   overestimate_factor_ = (overestimate_factor_ - 1.0) / 4.0 + 1.0;
 
  }

  // Last resort, try 2d search at max_altitude_
  if (!found_path) {
    printf("No path found, search in 2D \n");
    max_iterations_ = 5000;
    found_path = find2DPath(this, path, s, t, parent_of_s, max_altitude_);
  }

  return found_path;
}

// Returns true iff a path needs to be published, either a new path or a path
// back The path is then stored in this.pathMsg
bool GlobalPlanner::getGlobalPath() 
{
  if(cur_pos_received_ && cur_goal_received_ && octree_)
  {	
      std::cout <<"path plan !!!!!!!!!!!!!!!!!!!!" <<std::endl;
	    // Both current position and goal are free, try to find a path
	  std::vector<Cell> path;
	  if (!findPath(path)) {
	      ROS_INFO("  Failed to find a path... ");
	      return false;
	    }	
		setPath(path);
	    return true;
   }
}

// Sets path to be the current path
void GlobalPlanner::setPath(const std::vector<Cell>& path) {
  curr_path_info_ = getPathInfo(path);
  curr_path_ = path;
}

void GlobalPlanner::printPath(const nav_msgs::Path& msg) {
  for (auto p : msg.poses) {
	double x = p.pose.position.x;
	double y = p.pose.position.y;
	double z = p.pose.position.z;
	printf("(%2.2f, %2.2f, %2.2f) -> ", x, y, z);
  }
  printf("\n\n");
}

void GlobalPlanner::setRobotRadius(double radius) { robot_radius_ = radius; }

}  // namespace global_planner
