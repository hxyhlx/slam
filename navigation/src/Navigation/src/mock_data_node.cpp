#include "navigation/mock_data_node.h"
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<pcl/io/pcd_io.h>
#include "navigation/coorChange.h"
#include <string>

using namespace std;

string web_buff;
bool get_flag=0;  // ?? 不需要extern ??

namespace global_planner {

MockDataNode::MockDataNode(std::string path) 
	:path_(path)
{
  ros::NodeHandle nh;
 // depth_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_in", 10);
//  depth_points_pub_= nh.advertise<octomap_msgs::Octomap>("/octomap/custom", 100,true);
  local_position_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/local_pose", 100);
  global_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/clicked_point", 100);

  
 // createWall(5, 5, 6);
 //octomap::OcTree octree(1.0);
 //octree.readBinary(path_);
 // sendPcdData();
  ros::Duration(1.0).sleep();
  
  int num_loops = 0;
  ros::Rate rate(1);

  if(!SIMULATION)
  {
	webserver1 = new webserver;
  }
  while (ros::ok()) {
    if (num_loops++ == 10 && SIMULATION) {
      sendClickedPoint();
    }

	if(webserver1 != NULL && get_flag && getMsgFromMQTT())
	{
       sendWebGisPoint();
	}
 //   
    rate.sleep();
    ros::spinOnce();
  }
}

MockDataNode::~MockDataNode() { delete webserver1;}

void MockDataNode::createWall(int dist, int width, int height) {
  points_.clear();
  for (int m = 0; m < 2; m++){
  for (int i = -width; i <= width; ++i) {
    for (int j = 0; j <= height; ++j) {
      points_.push_back(dist*m + 0.5);
      points_.push_back(i + 0.5);
      points_.push_back(j + 0.5);
    }
  }
  }
}

void MockDataNode::sendClickedPoint() {
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "/map";   //world坐标系
  msg.pose.position.x = 30;  //7,-145,16
  msg.pose.position.y = 5;
  msg.pose.position.z = 5;
  global_goal_pub_.publish(msg);

  cout << "..........................................." << endl;
  cout << "current time is:" <<ros::Time::now().toSec() << endl;
  cout << "current time is:" <<msg.header.stamp.toSec() << endl;
  	
    // Send position
  geometry_msgs::PoseStamped pos;
  pos.header.frame_id = "/map";
  pos.pose.position.x = 9;
  pos.pose.position.y = -5;
  pos.pose.position.z = 5;
  local_position_pub_.publish(pos);
}


void MockDataNode::sendWebGisPoint()
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "/map";	//world坐标系
	msg.pose.position.x = goal_(0);
	msg.pose.position.y = goal_(1);
	msg.pose.position.z = goal_(2);
	global_goal_pub_.publish(msg);
	
	
	  // Send position
	geometry_msgs::PoseStamped pos;
	pos.header.frame_id = "/map";
	pos.pose.position.x = start_(0);
	pos.pose.position.y = start_(1);
	pos.pose.position.z = start_(2);
	local_position_pub_.publish(pos);

}

void MockDataNode::sendMockData() {
  // Create a PointCloud2
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "/map";
  // Fill some internals of the PoinCloud2 like the header/width/height ...
  cloud_msg.height = 1;
  cloud_msg.width = 4;
  // Set the point fields to xyzrgb and resize the vector with the following
  // command 4 is for the number of added fields. Each come in triplet: the name
  // of the PointField, the number of occurences of the type in the PointField,
  // the type of the PointField
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32, "rgb", 1, sensor_msgs::PointField::FLOAT32);
  // For convenience and the xyz, rgb, rgba fields, you can also use the
  // following overloaded function. You have to be aware that the following
  // function does add extra padding for backward compatibility though so it is
  // definitely the solution of choice for PointXYZ and PointXYZRGB 2 is for the
  // number of fields to add
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  // You can then reserve / resize as usual
  modifier.resize(1000);

  // Define some raw data we'll put in the PointCloud2
  int n = points_.size() / 3;
  uint8_t color_data[] = {40, 200, 120};

  // Define the iterators. When doing so, you define the Field you would like to
  // iterate upon and the type of you would like returned: it is not necessary
  // the type of the PointField as sometimes you pack data in another type (e.g.
  // 3 uchar + 1 uchar for RGB are packed in a float)
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you
  // can create iterators for those: they will handle data packing for you (in
  // little endian RGB is packed as *,R,G,B in a float and RGBA as A,R,G,B)
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
  // Fill the PointCloud2
  for (size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
    *iter_x = points_[3 * i + 0];
    *iter_y = points_[3 * i + 1];
    *iter_z = points_[3 * i + 2];
    *iter_r = color_data[0];
    *iter_g = color_data[1];
    *iter_b = color_data[2];
  }

  depth_points_pub_.publish(cloud_msg);
}
//原始点云数据
void MockDataNode::sendPcdData() {
  // Create a PointCloud2
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::PointCloud<pcl::PointXYZ> cloud;   
  pcl::io::loadPCDFile (path_, cloud);  
  pcl::toROSMsg(cloud,cloud_msg);// 转换成ROS下的数据类型 最终通过topic发布
  
  cloud_msg.header.frame_id = "/map";
  cloud_msg.header.stamp=ros::Time::now();
  depth_points_pub_.publish(cloud_msg);
}

//原始八叉树数据
void MockDataNode::sendOctrData(octomap::OcTree &octree) {
  // Create a PointCloud2
  octomap_msgs::Octomap map;
  if (octomap_msgs::fullMapToMsg(octree, map))
  {   
      map.header.frame_id = "/map";
      map.header.stamp = ros::Time::now();
      depth_points_pub_.publish(map);
  }
}

bool MockDataNode::getMsgFromMQTT()
{
	get_flag=0;
	gps_p s=webserver1->extract_coor(web_buff)[0];
	gps_p g=webserver1->extract_coor(web_buff)[1];
	start_ = gps2coorPx4(s);//(-52,-112,16);
	goal_ = gps2coorPx4(g);//(7,-145,16);

	std::vector<gps_p> gps;
	if(!((-956<start_.x())&&(start_.x()<766)&&(-855<start_.y())&&(start_.y()<715)&&
	(-956<goal_.x())&&(goal_.x()<766)&&(-855<goal_.y())&&(goal_.y()<715)))
	  {string msg1="ERROR: Out of Octomap! Please send again! ";
	  cout<<msg1<<endl; 
	  webserver1->pub(webserver1->mosq,gps,msg1);
	  return false;
	 }
	return true;
}



}  // namespace global_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "mock_data_node");
  std::string path = argv[1];
  global_planner::MockDataNode mock_data_node(path);
  ros::spin();
  return 0;
}
