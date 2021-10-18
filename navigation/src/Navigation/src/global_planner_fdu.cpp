
#include "navigation/global_planner_fdu.h"
#include "navigation/CAstar1.h"                //navigation/
#include <tf/tf.h>


namespace global_planner {

  using namespace std; 
  GlobalPlannerFdu::GlobalPlannerFdu() 
  {
	star = new CAstar();
  }
  
  GlobalPlannerFdu::~GlobalPlannerFdu()
  {
	if(star)
		delete star;
  }

  bool GlobalPlannerFdu::setPose(const geometry_msgs::PoseStamped& msg)
  {
    cur_pos_ = msg; 
	octomap::point3d cur_pos = convertPoseMsg2point(cur_pos_);
 	start_init =  star->init_pos(*octree_ , cur_pos); 
	
	if (start_init==nullptr)
		return false;
  	cur_pos_received_ = true;

	cout << "！！！！start is:("<<cur_pos.x()<<","<<cur_pos.y()<<","<<cur_pos.z()<<")"<<endl;
 	cout << "！！！！start_init is:"<<start_init->center<<" Distance="<<start_init->f<<endl;
	return true;
	
  }

  bool GlobalPlannerFdu::setGoal(const geometry_msgs::PoseStamped& msg)
  {
 	 cur_goal_ = msg;
	 octomap::point3d cur_goal = convertPoseMsg2point(cur_goal_);
     goal_init = star->init_pos(*octree_ , cur_goal);	
	 
	 if(goal_init==nullptr)
	 	return false;
  	 cur_goal_received_ = true;

	 cout << "！！！！goal is:("<<cur_goal.x()<<","<<cur_goal.y()<<","<<cur_goal.z()<<")"<<endl;
 	 cout << "！！！！goal_init is:"<<goal_init->center<<" Distance="<<goal_init->f<<endl;
	 return true;
  }

  void GlobalPlannerFdu::updateFullOctomap(octomap::OcTree* tree)
  {
    if (octree_) {
    delete octree_;
  }
  octree_ = tree;
  octree_resolution_ = octree_->getResolution();
  star->setRes(static_cast<int>(octree_resolution_));
 }

  
 double GlobalPlannerFdu::nextYaw(octomap::point3d u, octomap::point3d v) {
   double dx = v(0) - u(0);
   double dy = v(1) - u(1);
 
   return std::atan2(dy, dx);
 }
 
 octomap::point3d GlobalPlannerFdu::convertPoseMsg2point(geometry_msgs::PoseStamped pose_msg)
 {
 	return octomap::point3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
 }
 
 geometry_msgs::PoseStamped GlobalPlannerFdu::createPoseMsg(const octomap::point3d& point, double yaw) 
 {
   geometry_msgs::PoseStamped pose_msg;
   pose_msg.header.frame_id = frame_id_;
   pose_msg.pose.position.x = point(0);
   pose_msg.pose.position.y = point(1);
   pose_msg.pose.position.z = point(2);
   pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
   return pose_msg;
 }
 
 nav_msgs::Path GlobalPlannerFdu::getPathMsg(const std::vector<octomap::point3d>& path) 
 {
   nav_msgs::Path path_msg;
   path_msg.header.frame_id = frame_id_;
 
   if (path.size() == 0) {
 	return path_msg;
   }
 
   double new_yaw = 0;
   for (int i = 0; i < path.size() - 1; ++i) {
 	octomap::point3d p = path[i];
 	new_yaw = nextYaw(p, path[i + 1]);
 	// if (new_yaw != last_yaw) {	// only publish corner points
 	path_msg.poses.push_back(createPoseMsg(p, new_yaw));
 	// } 
   }
   octomap::point3d last_point = path[path.size() - 1];	// Last point should have the same yaw as the previous point
   path_msg.poses.push_back(createPoseMsg(last_point, new_yaw));
   return path_msg;
 }
 
 APoint* GlobalPlannerFdu::planPath(const octomap::OcTree &octree)
 { 	
 	cout <<"plan path" <<endl;
 	ros::Time begin = ros::Time::now();
 	auto point = star->findWay(start_init, goal_init, octree);
 	ros::Time end = ros::Time::now();
 	ros::Duration elapsed_time = end - begin;
 
 	cout <<"A* elapse time：" << elapsed_time.toSec()* 1000<<"ms"<< endl;
 	if (!point)
 	{
 		return 0;
 	}
 	return point;
 }
 
 void  GlobalPlannerFdu::getPathRe(APoint* point)
 {
    pathRe_.clear();
    octomap::point3d cur_goal = convertPoseMsg2point(cur_goal_);
    octomap::point3d cur_pos = convertPoseMsg2point(cur_pos_);
    pathRe_.insert(pathRe_.begin(), cur_goal);
 	//cout<<goal<<"="<<endl;
 	while (point)
 	{
 		pathRe_.insert(pathRe_.begin(), point->center);
 		point = point->parent;
 	} 	
 	pathRe_.insert(pathRe_.begin(),cur_pos);
 }
 
 void GlobalPlannerFdu::simplifyPath(octomap::OcTree octree)
 {
 	pathCut_.clear();
 	cout<<"Pruning Road!======================="<<endl;
 	octomap::point3d cur_goal = convertPoseMsg2point(cur_goal_);
 	octomap::point3d cur_pos = convertPoseMsg2point(cur_pos_);
 	pathCut_.push_back(cur_pos);
 	
 	std::vector <octomap::point3d> ray1;
 	std::vector<octomap::point3d>::iterator t,t1 ;
 
 	t=pathRe_.begin(); t1=pathRe_.begin()+1;
 	
 	for(; t!=pathRe_.end(),t1!=pathRe_.end(); t1++)
 	{
 		octree.computeRay(*t, *t1, ray1); 
 		for (octomap::point3d key : ray1) 
 		{
 
 		   octomap::OcTreeNode* node = octree.search(key);
 
 			if (node->getOccupancy()>0.5){//octree.isNodeOccupied(node)) {
 
 			   pathCut_.push_back(*(t1-1));
 			   cout<<*(t1-1)<<endl ;
 			   t=t1-1;
 			   break;
 			}
 		}
 	}
 	
 	pathCut_.push_back(cur_goal);	
 }


bool GlobalPlannerFdu::getGlobalPath()
{
  if(cur_pos_received_ && cur_goal_received_ && octree_)
  {
	cout <<"path plan !!!!!!!!!!!!!!!!!!!!" <<endl;
	APoint* point = planPath(*octree_);
	if (!point)
	{
		cout << "plan path fail !!!" << endl;
		return false;
	}
	else
	{
		getPathRe(point);
		simplifyPath(*octree_);
		pathMsgsRe_ = getPathMsg(pathRe_);
		cout << "path origin !!!" << endl;
		printPath(pathMsgsRe_);
		pathMsgsCut_ = getPathMsg(pathCut_);
		cout << "path cut !!!" << endl;
		printPath(pathMsgsCut_);
		return true;
	}
	
  }
}

void GlobalPlannerFdu::printPath(const nav_msgs::Path& msg) {
  for (auto p : msg.poses) {
	double x = p.pose.position.x;
	double y = p.pose.position.y;
	double z = p.pose.position.z;
	printf("(%2.2f, %2.2f, %2.2f) -> ", x, y, z);
  }
  printf("\n\n");
}



}





