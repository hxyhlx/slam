#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H

#include <array>
#include <random>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <test_mavros/pid_controller.h>

namespace testsetup {
/**
 * @brief Offboard controller tester
 *
 * Tests offboard position, velocity and acceleration control
 *
 */

typedef enum {
	POSITION,
	VELOCITY,
	ACCELERATION
} control_mode;
	
//FOLLOW跟踪模式
typedef enum {
	SQUARE,
	FOLLOW
} path_shape;

class OffboardControl {
public:
	OffboardControl();
	~OffboardControl();
	void init();
	void spin(int argc, char *argv[]); 
private:
	ros::NodeHandle nh_sp_{"~"};
	pidcontroller::PIDController pid_;
	double rate_;
	int num_of_tests_; //TODO: find a way to use this...
	bool use_pid_;
	
	double linvel_p_gain_;
	double linvel_i_gain_;
	double linvel_d_gain_;
	double linvel_i_max_;
	double linvel_i_min_;

	double yawrate_p_gain_;
	double yawrate_i_gain_;
	double yawrate_d_gain_;
	double yawrate_i_max_;
	double yawrate_i_min_;
	double kp_;
	double vel_limit_;

	control_mode mode_;
	path_shape shape_;

	ros::Publisher local_pos_sp_pub_;
	ros::Publisher vel_sp_pub_;
	ros::Publisher path_current_pub_;
	ros::Publisher path_setpoint_pub_;
	
	ros::Subscriber local_pos_sub_;
	ros::Subscriber state_sub_;
	ros::Subscriber model_sub_;
	ros::Subscriber move_base_simple_sub_;
	
	ros::ServiceClient arming_client_;
	ros::ServiceClient set_mode_client_;

	geometry_msgs::PoseStamped localpos_, mtarget_;
	geometry_msgs::TwistStamped vs_;
	mavros_msgs::State current_state_;
	nav_msgs::Path path_current_;
	nav_msgs::Path path_setpoint_;

	Eigen::Vector3d current_;
	double targetupdate_dt_{0.1};
	ros::Time last_state_time_;
	ros::Time last_request_;

	/**
	 * @brief Defines single position setpoint
	 */
	Eigen::Vector3d pos_setpoint(int tr_x, int tr_y, int tr_z);
	/**
	 * @brief Square path motion routine
	 */
	void square_path_motion(ros::Rate loop_rate, control_mode mode);
	/**
	 * @brief Defines the accepted threshold to the destination/target position
	 * before moving to the next setpoint./gazebo/modelmove_world/suv/model_move
	 */
	void wait_and_move(geometry_msgs::PoseStamped target);
	
	void pblish_target(geometry_msgs::PoseStamped target);

	void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg);
	
	void local_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg);
	void state_cb(const mavros_msgs::State::ConstPtr& msg);
	
	void moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg);

	bool wait_to_arm_offboard();		
	};	// namespace testsetup
}

#endif