#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H

#include <boost/bind.hpp>
#include <mutex>
#include <set>

#include <pcl_ros/transforms.h>
#include <ros/callback_queue.h>

#include "navigation/GLobalPlannerInterface.h"

class webserver;
namespace global_planner {

class GlobalPlannerNode {
 public:
  
  GlobalPlannerNode(std::string path);
  ~GlobalPlannerNode();

 private:
 	
	std::mutex mutex_;
	std::string octo_path_;

	std::string frame_id_{"/map"};
	double plannerloop_dt_{1.0};
	double mapupdate_dt_{1};
	ros::Time last_map_update_time_{0};
	octomap::OcTree *mOcTree_{NULL};
	octomap::OcTree octree_{1};

	ros::Subscriber octomap_full_sub_;
	ros::Subscriber clicked_point_sub_;    //模拟发布的目标位置
  	ros::Subscriber move_base_simple_sub_; //rviz发布的目标位置
	ros::Subscriber ground_truth_sub_;    //当前无人机位置

	ros::Publisher global_temp_path_pub_; 
	ros::Publisher global_goal_pub_;
	ros::Publisher global_pathRe_pub_;
	ros::Publisher global_pathCut_pub_;
	ros::Publisher octo_pub_;

	ros::Timer plannerloop_timer_;
	ros::CallbackQueue plannerloop_queue_;
	std::unique_ptr<ros::AsyncSpinner> plannerloop_spinner_;
	GlobalPlannerInterface *global_planner_;
	nav_msgs::Path pathCut_;

	webserver * webserver1{NULL};

	//::TransformListener listener_;
	void positionCallback(const geometry_msgs::PoseStamped& msg);
	void clickedPointCallback(const geometry_msgs::PoseStamped& msg);
	void moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg);
	void plannerLoopCallback(const ros::TimerEvent& event);
	void octomapFullCallback(const octomap_msgs::Octomap& msg);
	octomap::point3d convertPoseMsg2point(geometry_msgs::PoseStamped pose_msg);

	void printPath(const nav_msgs::Path& msg);
	void publishPath();
	bool planPath();
	void pubGps2Web();

};

}


#endif  // GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
