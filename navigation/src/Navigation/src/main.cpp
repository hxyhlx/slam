#include "navigation/global_planner_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planner_node");
  
  global_planner::GlobalPlannerNode global_planner_node(argv[1]);

  ros::spin();
  return 0;
}
