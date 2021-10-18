#include "navigation/global_planner_node.h"
#include "navigation/global_planner_fdu.h"
#include "navigation/global_planner.h"
#include "navigation/web.h"
#include "navigation/coorChange.h"


#define CHOOSE_FDU 0
namespace global_planner {
using namespace std;

GlobalPlannerNode::GlobalPlannerNode(string path)
	:octo_path_(path)

{
  // Subscribers
  ros::NodeHandle nh_;
  if(CHOOSE_FDU)
    global_planner_ = new GlobalPlannerFdu;
  else
  	global_planner_ = new GlobalPlanner;

  if(!SIMULATION)
  {
	webserver1 = new webserver;
  }
  
  //octomap_full_sub_ = nh_.subscribe("/octomap_full", 1, &GlobalPlannerNode::octomapFullCallback, this);
  ground_truth_sub_ = nh_.subscribe("/local_pose", 1, &GlobalPlannerNode::positionCallback, this);
  clicked_point_sub_ = nh_.subscribe("/clicked_point", 1, &GlobalPlannerNode::clickedPointCallback, this); //mock_data_node
  //move_base_simple_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &GlobalPlannerNode::moveBaseSimpleCallback, this);//rviz里面点击
 
  // Publishers
  global_pathRe_pub_ = nh_.advertise<nav_msgs::Path>("/pathRe", 10);
  global_pathCut_pub_ = nh_.advertise<nav_msgs::Path>("/pathCut", 10);
  global_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/global_goal", 10);
  octo_pub_ = nh_.advertise<octomap_msgs::Octomap>("/octomap/custom", 100,true);

  ros::TimerOptions plannerlooptimer_options(ros::Duration(plannerloop_dt_),
                                             boost::bind(&GlobalPlannerNode::plannerLoopCallback, this, _1),
                                             &plannerloop_queue_);
  plannerloop_timer_ = nh_.createTimer(plannerlooptimer_options);
  plannerloop_spinner_.reset(new ros::AsyncSpinner(1, &plannerloop_queue_));
  plannerloop_spinner_->start();
 
  octree_.readBinary(octo_path_);
  octomap_msgs::Octomap map;
  if (octomap_msgs::fullMapToMsg(octree_, map))
  {   
      map.header.frame_id = "/map";
      map.header.stamp = ros::Time::now();
      octo_pub_.publish(map);
	  cout << "octomap published" <<endl;
  }
  global_planner_->updateFullOctomap(&octree_);
  global_planner_->setFrame(frame_id_);

}

GlobalPlannerNode::~GlobalPlannerNode()
{
	if(global_planner_)
		delete global_planner_;
	if(webserver1)
	   delete webserver1;
}

void GlobalPlannerNode::printPath(const nav_msgs::Path& msg) {
  for (auto p : msg.poses) {
    double x = p.pose.position.x;
    double y = p.pose.position.y;
    double z = p.pose.position.z;
    printf("(%2.2f, %2.2f, %2.2f) -> ", x, y, z);
  }
  printf("\n\n");
}

// Check if the current path is blocked

void GlobalPlannerNode::octomapFullCallback(const octomap_msgs::Octomap& msg) 
{
 
  std::lock_guard<std::mutex> lock(mutex_);
  ros::Time current = ros::Time::now();
  // Update map at a fixed rate. This is useful on setting replanning rates for the planner.
  if ((current - last_map_update_time_).toSec() < mapupdate_dt_) {
	return;
  }
  last_map_update_time_ = ros::Time::now();
  mOcTree_ = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
  global_planner_->updateFullOctomap(mOcTree_);

  cout << "!!!!!!!!!!!!!!!!octomapFullCallback!!!!!!!!!!!!!!!!!!!" << mOcTree_ << endl;
}

void GlobalPlannerNode::clickedPointCallback(const geometry_msgs::PoseStamped& msg) 
{
  cout <<"goal is:(" <<msg.pose.position.x<<"," <<msg.pose.position.y<<","<<msg.pose.position.z<<")"<<endl;
  bool success = global_planner_->setGoal(msg);
  
  if(!success && webserver1 != NULL)
  {
	 string msg3="The goal is occupied! Can Not Find Way!";
	 cout<<msg3<<endl;  
	 
	 octomap::point3d point = convertPoseMsg2point(msg);
	 gps_p _gps=coor2gpsPx4(point);
	 vector<gps_p> gps;
	 gps.push_back(_gps);
	 webserver1->pub(webserver1->mosq,gps,msg3);
  }
  global_goal_pub_.publish(msg);
}

void GlobalPlannerNode::moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg) 
{
  cout <<"goal is:(" <<msg.pose.position.x<<"," <<msg.pose.position.y<<","<<msg.pose.position.z<<")"<<endl;
 // msg.pose.position.z = 16;
  global_planner_->setGoal(msg);

  bool success = global_planner_->setGoal(msg); 
  if(!success && webserver1 != NULL)
  {
	 string msg3="The goal is occupied! Can Not Find Way!";
	 cout<<msg3<<endl;  

	 octomap::point3d point = convertPoseMsg2point(msg);
	 gps_p _gps=coor2gpsPx4(point);
	 vector<gps_p> gps;
	 gps.push_back(_gps);
	 webserver1->pub(webserver1->mosq,gps,msg3);
  }
  
  global_goal_pub_.publish(msg);
}

// Sets the current position and checks if the current goal has been reached
void GlobalPlannerNode::positionCallback(const geometry_msgs::PoseStamped& msg) {
  // Update position
  cout << "local_pos call back" << endl;
  cout <<"start is:(" <<msg.pose.position.x<<"," <<msg.pose.position.y<<","<<msg.pose.position.z<<")"<<endl;
  bool success = global_planner_->setPose(msg);
  
  if(!success && webserver1 != NULL)
  {
	 string msg2="The start is occupied! Can Not Find Way!";
	 cout<<msg2<<endl;

	 octomap::point3d point = convertPoseMsg2point(msg);
	 gps_p _gps=coor2gpsPx4(point);
	 vector<gps_p> gps;
	 gps.push_back(_gps);
	 webserver1->pub(webserver1->mosq,gps,msg2);
  }  	
}

void GlobalPlannerNode::plannerLoopCallback(const ros::TimerEvent& event) 
{ 
  std::lock_guard<std::mutex> lock(mutex_);
 // if()
 // 	return;
  if(planPath())
  {
	ROS_INFO("success to find a path");
	publishPath();
	if(webserver1 != NULL)
	{
		pubGps2Web();
	}
  }
  
}

void GlobalPlannerNode::publishPath()
{
	nav_msgs::Path pathRe = global_planner_->getPathOriginMsg();
	global_pathRe_pub_.publish(pathRe);

	pathCut_ = global_planner_->getPathMsg();
	global_pathCut_pub_.publish(pathCut_);
}

// Plans a new path and publishes it
bool GlobalPlannerNode::planPath() {
  bool found_path = global_planner_->getGlobalPath();
  if (!found_path) {
	ROS_INFO("Failed to find a path");
  } 
  return found_path;
}

octomap::point3d GlobalPlannerNode::convertPoseMsg2point(geometry_msgs::PoseStamped pose_msg)
{
   return octomap::point3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
}

void GlobalPlannerNode::pubGps2Web() 
{
	cout<<"------Coor to GPS------"<<endl;	
	vector<gps_p> gps;
	for (int i = 0; i < pathCut_.poses.size(); i++)
	{
		geometry_msgs::PoseStamped pose = pathCut_.poses[i];
		octomap::point3d point = convertPoseMsg2point(pose);
		gps_p _gps=coor2gpsPx4(point);
		gps.push_back(_gps);
		cout<<"("<<setprecision(11)<<_gps.lon<<setprecision(11)<<","<<_gps.lat<<","<<_gps.alt<<")"<<endl; 
	}
	string msg0="";
	webserver1->pub(webserver1->mosq,gps,msg0);
} 

}

