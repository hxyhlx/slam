/**
 * @brief Offboard control test
 * @file offboard_control.h
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Andre Nguyen <andre-phu-van.nguyen@polymtl.ca>
 *
 * @addtogroup sitl_test
 * @{
 */
/*
 * Copyright 2015 Nuno Marques, Andre Nguyen.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

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
	OffboardControl() :
		local_pos_sp_pub(nh_sp.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10)),
		vel_sp_pub(nh_sp.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10)),
		path_current_pub_ (nh_sp.advertise<nav_msgs::Path>("/current_path", 10)),
		path_setpoint_pub_ (nh_sp.advertise<nav_msgs::Path>("/setpoint_path", 10)),
		
		move_base_simple_sub_(nh_sp.subscribe("/move_base_simple/goal", 1, &OffboardControl::moveBaseSimpleCallback, this)),

		local_pos_sub(nh_sp.subscribe("/mavros/local_position/pose", 10, &OffboardControl::local_pos_cb, this)),
		model_sub(nh_sp.subscribe("/gazebo/model_states", 10, &OffboardControl::modelStatesCallback, this)),
		state_sub(nh_sp.subscribe("/mavros/state", 10, &OffboardControl::state_cb, this)),
		arming_client(nh_sp.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming")),
		set_mode_client(nh_sp.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"))
	{ };

	void init() {
		/**
		 * @brief Setup of the test conditions
		 */
		nh_sp.param("rate", rate, 10.0);
		nh_sp.param("num_of_tests", num_of_tests, 10);
		nh_sp.param("use_pid", use_pid, true);

		if (use_pid) {

			// Linear velocity PID gains and bound of integral windup
			nh_sp.param("linvel_p_gain", linvel_p_gain, 0.4);
			nh_sp.param("linvel_i_gain", linvel_i_gain, 0.05);
			nh_sp.param("linvel_d_gain", linvel_d_gain, 0.12);
			nh_sp.param("linvel_i_max", linvel_i_max, 0.1);
			nh_sp.param("linvel_i_min", linvel_i_min, -0.1);

			// Yaw rate PID gains and bound of integral windup
			nh_sp.param("yawrate_p_gain", yawrate_p_gain, 0.011);
			nh_sp.param("yawrate_i_gain", yawrate_i_gain, 0.00058);
			nh_sp.param("yawrate_d_gain", yawrate_d_gain, 0.12);
			nh_sp.param("yawrate_i_max", yawrate_i_max, 0.005);
			nh_sp.param("yawrate_i_min", yawrate_i_min, -0.005);

			// Setup of the PID controllers
			pid.setup_linvel_pid(linvel_p_gain, linvel_i_gain, linvel_d_gain, linvel_i_max, linvel_i_min, nh_sp);
			pid.setup_yawrate_pid(yawrate_p_gain, yawrate_i_gain, yawrate_d_gain, yawrate_i_max, yawrate_i_min, nh_sp);
		}

		std::string mode_;
		nh_sp.param<std::string>("mode", mode_, "position");

		std::string shape_;
		nh_sp.param<std::string>("shape", shape_, "square");

		
		path_current.header.frame_id = "map";
		path_setpoint.header.frame_id = "map";

		if (mode_ == "position")
			mode = POSITION;
		else if (mode_ == "velocity")
			mode = VELOCITY;
		else if (mode_ == "acceleration")
			mode = ACCELERATION;
		else {
			ROS_ERROR_NAMED("sitl_test", "Control mode: wrong/unexistant control mode name %s", mode_.c_str());
			return;
		}

		if (shape_ == "square")
			shape = SQUARE;
		else if (shape_ == "follow")
			shape = FOLLOW;
	
	}
	
	/* -*- main routine -*- */

	void spin(int argc, char *argv[]) {
		init();
		ros::Rate loop_rate(rate);
		    // wait for FCU connection
    	while(ros::ok() && !current_state.connected){
        	ros::spinOnce();
        	loop_rate.sleep();
		}
	
		while(ros::ok() && !wait_to_arm_offboard()){
			loop_rate.sleep();
		}

		ROS_INFO("SITL Test: Offboard control test running!");

		ps.header.frame_id = "map";
		ps.pose.position.x = 0;
    	ps.pose.position.y = 0;
    	ps.pose.position.z = 10;
	//	local_pos_sp_pub.publish(ps);
		
		if (mode == POSITION) {
			ROS_INFO("Position control mode selected.");
		}
		else if (mode == VELOCITY) {
			ROS_INFO("Velocity control mode selected.");
		}
		else if (mode == ACCELERATION) {
			ROS_INFO("Acceleration control mode selected.");
			ROS_ERROR_NAMED("sitl_test", "Control mode: acceleration control mode not supported in PX4 current Firmware.");
			/**
			 * @todo: lacks firmware support, for now
			 */
			return;
		}
		
		if (shape == SQUARE) {
			ROS_INFO("Test option: square-shaped path...");
			square_path_motion(loop_rate, mode);
		}
		else if (shape == FOLLOW) {
			ROS_INFO("Test follow...");
			ros::spin();
			//wait_and_move(mtarget);
		}
		

	}

private:
	ros::NodeHandle nh_sp{"~"};
	pidcontroller::PIDController pid;
	double rate;
	int num_of_tests; //TODO: find a way to use this...
	bool use_pid;
	
	double linvel_p_gain;
	double linvel_i_gain;
	double linvel_d_gain;
	double linvel_i_max;
	double linvel_i_min;

	double yawrate_p_gain;
	double yawrate_i_gain;
	double yawrate_d_gain;
	double yawrate_i_max;
	double yawrate_i_min;

	control_mode mode;
	path_shape shape;

	ros::Publisher local_pos_sp_pub;
	ros::Publisher vel_sp_pub;
	ros::Publisher path_current_pub_;
	ros::Publisher path_setpoint_pub_;
	
	ros::Subscriber local_pos_sub;
	ros::Subscriber state_sub;
	ros::Subscriber model_sub;
	ros::Subscriber move_base_simple_sub_;
	
	ros::ServiceClient arming_client;
	ros::ServiceClient set_mode_client;

	geometry_msgs::PoseStamped localpos, ps,mtarget;
	geometry_msgs::TwistStamped vs;
	mavros_msgs::State current_state;
	nav_msgs::Path path_current;
	nav_msgs::Path path_setpoint;

	Eigen::Vector3d current;
	double targetupdate_dt_{0.1};
	ros::Time last_state_time_;
	ros::Time last_request;

	//std::array<double, 100> threshold;

	/* -*- helper functions -*- */

	/**
	 * @brief Defines single position setpoint
	 */
	Eigen::Vector3d pos_setpoint(int tr_x, int tr_y, int tr_z){
		/** @todo Give possibility to user define amplitude of movement (square corners coordinates)*/
		return Eigen::Vector3d(tr_x * 1.0f, tr_y * 1.0f, tr_z * 1.0f);	// meters
	}


	/**
	 * @brief Square path motion routine
	 */
	void square_path_motion(ros::Rate loop_rate, control_mode mode){
		uint8_t pos_target = 0;
		uint8_t pos_target_pre = -1;

		ROS_INFO("Testing...");

		while (ros::ok()) {
			wait_and_move(ps);
			
			if(pos_target_pre != pos_target)
			{
				path_setpoint.poses.push_back(ps);
				pos_target_pre = pos_target;
			}

			// motion routine
			switch (pos_target) {
			case 1:
				tf::pointEigenToMsg(pos_setpoint(0, 15, 10), ps.pose.position);
				ROS_INFO("Arrived 1 ...");
				break;
			case 2:
				tf::pointEigenToMsg(pos_setpoint(-20, 15, 10), ps.pose.position);
				ROS_INFO("Arrived 2 ...");
				break;
			case 3:
				tf::pointEigenToMsg(pos_setpoint(-20, -15, 10), ps.pose.position);
				ROS_INFO("Arrived 3 ...");
				break;
			case 4:
				tf::pointEigenToMsg(pos_setpoint(0, -15, 10), ps.pose.position);
				ROS_INFO("Arrived 4 ...");
				break;
			default:
				break;
			}

			if (pos_target == 4) {
				ROS_INFO("Test complete!");
				pos_target = 1;
				//ros::shutdown();
			}
			else
				++pos_target;

			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	/**
	 * @brief Defines the accepted threshold to the destination/target position
	 * before moving to the next setpoint./gazebo/modelmove_world/suv/model_move
	 */
	void wait_and_move(geometry_msgs::PoseStamped target){
		ros::Rate loop_rate(rate);
		bool stop = false;
		ros::Time last_time = ros::Time::now();
		Eigen::Vector3d dest;

		double distance;
		double err_th = 2;

		ROS_INFO("Next setpoint: accepted error threshold: %1.3f", err_th);

		while (ros::ok() && !stop ) {
			tf::pointMsgToEigen(target.pose.position, dest);
			tf::pointMsgToEigen(localpos.pose.position, current);

			distance = sqrt((dest - current).x() * (dest - current).x() +
					(dest - current).y() * (dest - current).y() +
					(dest - current).z() * (dest - current).z());

			if (distance <= err_th)
			{
				ROS_INFO("setpoint: %2.3f,%2.3f,%2.3f; current position:  %2.3f,%2.3f,%2.3f", current.x(),current.y(),current.z(),dest.x(),dest.y(),dest.z());
				stop = true;
			}

			if (mode == POSITION) {
				local_pos_sp_pub.publish(target);
			}
			else if (mode == VELOCITY) {
				if (use_pid)
					tf::vectorEigenToMsg(pid.compute_linvel_effort(dest, current, last_time), vs.twist.linear);
				else
					tf::vectorEigenToMsg(dest - current, vs.twist.linear);
				vel_sp_pub.publish(vs);

			}
			else if (mode == ACCELERATION) {
				// TODO
				return;
			}
			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}
	
	void pblish_target(geometry_msgs::PoseStamped target){
		bool stop = false;
		ros::Time last_time = ros::Time::now();
		Eigen::Vector3d dest;

		double distance;
		double err_th = 2;
		
		tf::pointMsgToEigen(target.pose.position, dest);
		tf::pointMsgToEigen(localpos.pose.position, current);

		distance = sqrt((dest - current).x() * (dest - current).x() +
					(dest - current).y() * (dest - current).y() +
					(dest - current).z() * (dest - current).z());

		if (distance <= err_th)
		{
			ROS_INFO("setpoint: %2.3f,%2.3f,%2.3f; current position:  %2.3f,%2.3f,%2.3f", current.x(),current.y(),current.z(),dest.x(),dest.y(),dest.z());
		}

		if (mode == POSITION) {
			local_pos_sp_pub.publish(target);
		}
		else if (mode == VELOCITY) {
			if (use_pid)
				tf::vectorEigenToMsg(pid.compute_linvel_effort(dest, current, last_time), vs.twist.linear);
			else
				tf::vectorEigenToMsg(dest - current, vs.twist.linear);
			vel_sp_pub.publish(vs);

		}
		else if (mode == ACCELERATION) {
			// TODO
			return;
		}

	}


	void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
	{
		if(shape != FOLLOW)
			return;
		ros::Time current = ros::Time::now();
		// Update map at a fixed rate. This is useful on setting replanning rates for the planner.
		if ((current - last_state_time_).toSec() < targetupdate_dt_) {
		  return;
		}
		last_state_time_ = ros::Time::now();

		int modelCount = msg->name.size();
		for(int modelInd = 0; modelInd < modelCount; ++modelInd)
		{
			if(msg->name[modelInd] == "suv")
			{
				mtarget.header.stamp = ros::Time::now();
				mtarget.pose = msg->pose[modelInd];
				mtarget.pose.position.z = mtarget.pose.position.z + 10;
				path_setpoint.poses.push_back(mtarget);
				pblish_target(mtarget);
				break;
			}
		}
	}
	
	void local_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg){
	
		localpos = *msg;
		path_current.poses.push_back(localpos);
		path_current_pub_.publish(path_current);
		path_setpoint_pub_.publish(path_setpoint);
	}

	void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
	}
	
	void moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg){		
		std::cout <<"goal is:(" <<msg.pose.position.x<<"," <<msg.pose.position.y<<","<<msg.pose.position.z<<")"<<std::endl;
		mtarget = msg;
		mtarget.pose.position.z = 15;
		path_setpoint.poses.push_back(mtarget);	
		wait_and_move(mtarget);
	}

	bool wait_to_arm_offboard()
	{
	    
		mavros_msgs::SetMode offb_set_mode;
    	offb_set_mode.request.custom_mode = "OFFBOARD";
    	mavros_msgs::CommandBool arm_cmd;
    	arm_cmd.request.value = true;
		
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
			else
			{					
               last_request = ros::Time::now();
			   return false;
			}

        }
			
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
			else
			{					
			   last_request = ros::Time::now();
			   return false;
			}

        }
		return true;   

	}

	
	};	// namespace testsetup
}
