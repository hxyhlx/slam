#ifndef GLOBAL_PLANNER_MOCK_DATA_NODE_H
#define GLOBAL_PLANNER_MOCK_DATA_NODE_H

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "octomap_msgs/Octomap.h"
#include "octomap/octomap.h"
#include "octomap/OcTree.h"
#include "octomap/ColorOcTree.h"
#include "octomap_msgs/conversions.h"
#include "navigation/web.h"

#include <stdlib.h>
#include <vector>

namespace global_planner {

class MockDataNode {
 public:
  MockDataNode(std::string path);
  ~MockDataNode();
  void createWall(int dist, int width, int height);
  void sendClickedPoint();
  void sendWebGisPoint();
  void sendMockData();
  void sendPcdData();
  void sendOctrData(octomap::OcTree &octree);
  
  bool getMsgFromMQTT();

  std::vector<float> points_{5.5, -0.5, 0.5, 5.5, 0.5, 0.5,  5.5, 1.5, 0.5, 5.5, -0.5, 1.5, 5.5, 0.5,
                             1.5, 5.5,  1.5, 1.5, 5.5, -0.5, 2.5, 5.5, 0.5, 2.5, 5.5,  1.5, 2.5};

 private:
  ros::Subscriber path_sub_;

  ros::Publisher local_position_pub_;
  ros::Publisher depth_points_pub_;
  ros::Publisher global_goal_pub_;
  std::string path_;
  webserver * webserver1{NULL};
  octomap::point3d start_;
  octomap::point3d goal_;
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_MOCK_DATA_NODE_H
