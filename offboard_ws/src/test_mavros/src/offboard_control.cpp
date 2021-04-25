
#include "test_mavros/offboard_control.h"

namespace testsetup {

	OffboardControl::OffboardControl() {
		
		local_pos_sp_pub_ = nh_sp_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
		vel_sp_pub_ = nh_sp_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
		path_current_pub_ = nh_sp_.advertise<nav_msgs::Path>("/current_path", 10);
		path_setpoint_pub_ = nh_sp_.advertise<nav_msgs::Path>("/setpoint_path", 10);
		
		move_base_simple_sub_ = nh_sp_.subscribe("/move_base_simple/goal", 1, &OffboardControl::moveBaseSimpleCallback, this);
		local_pos_sub_ = nh_sp_.subscribe("/mavros/local_position/pose", 10, &OffboardControl::local_pos_cb, this);
		model_sub_ = nh_sp_.subscribe("/gazebo/model_states", 10, &OffboardControl::modelStatesCallback, this);
		state_sub_ = nh_sp_.subscribe("/mavros/state", 10, &OffboardControl::state_cb, this);
		arming_client_ = nh_sp_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
		set_mode_client_ = nh_sp_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	}
	
	OffboardControl::~OffboardControl() {}

	void OffboardControl::init() {
		/**
		 * @brief Setup of the test conditions
		 */
		nh_sp_.param("rate", rate_, 10.0);
		nh_sp_.param("num_of_tests", num_of_tests_, 10);
		nh_sp_.param("use_pid", use_pid_, true);

		if (use_pid_) {

			// Linear velocity PID gains and bound of integral windup
			nh_sp_.param("linvel_p_gain", linvel_p_gain_, 0.4);
			nh_sp_.param("linvel_i_gain", linvel_i_gain_, 0.05);
			nh_sp_.param("linvel_d_gain", linvel_d_gain_, 0.12);
			nh_sp_.param("linvel_i_max", linvel_i_max_, 0.1);
			nh_sp_.param("linvel_i_min", linvel_i_min_, -0.1);

			// Yaw rate PID gains and bound of integral windup
			nh_sp_.param("yawrate_p_gain", yawrate_p_gain_, 0.011);
			nh_sp_.param("yawrate_i_gain", yawrate_i_gain_, 0.00058);
			nh_sp_.param("yawrate_d_gain", yawrate_d_gain_, 0.12);
			nh_sp_.param("yawrate_i_max", yawrate_i_max_, 0.005);
			nh_sp_.param("yawrate_i_min", yawrate_i_min_, -0.005);

			// Setup of the PID controllers
			pid_.setup_linvel_pid(linvel_p_gain_, linvel_i_gain_, linvel_d_gain_, linvel_i_max_, linvel_i_min_, nh_sp_);
			pid_.setup_yawrate_pid(yawrate_p_gain_, yawrate_i_gain_, yawrate_d_gain_, yawrate_i_max_, yawrate_i_min_, nh_sp_);
		}

		std::string mode;
		nh_sp_.param<std::string>("mode", mode, "position");

		std::string shape;
		nh_sp_.param<std::string>("shape", shape, "square");

		
		path_current_.header.frame_id = "map";
		path_setpoint_.header.frame_id = "map";

		if (mode == "position")
			mode_ = POSITION;
		else if (mode == "velocity")
			mode_ = VELOCITY;
		else if (mode == "acceleration")
			mode_ = ACCELERATION;
		else {
			ROS_ERROR_NAMED("sitl_test", "Control mode: wrong/unexistant control mode name %s", mode.c_str());
			return;
		}

		if (shape == "square")
			shape_ = SQUARE;
		else if (shape == "follow")
			shape_ = FOLLOW;
	
	}
	
	/* -*- main routine -*- */

	void OffboardControl::spin(int argc, char *argv[]) {
		init();
		ros::Rate loop_rate(rate_);
		    // wait for FCU connection
    	while(ros::ok() && !current_state_.connected){
        	ros::spinOnce();
        	loop_rate.sleep();
		}
	
		while(ros::ok() && !wait_to_arm_offboard()){
			loop_rate.sleep();
		}

		ROS_INFO("SITL Test: Offboard control test running!");

		mtarget_.header.frame_id = "map";
		mtarget_.pose.position.x = 0;
    	mtarget_.pose.position.y = 0;
    	mtarget_.pose.position.z = 10;
	//	local_pos_sp_pub.publish(ps);
		
		if (mode_ == POSITION) {
			ROS_INFO("Position control mode selected.");
		}
		else if (mode_ == VELOCITY) {
			ROS_INFO("Velocity control mode selected.");
		}
		else if (mode_ == ACCELERATION) {
			ROS_INFO("Acceleration control mode selected.");
			ROS_ERROR_NAMED("sitl_test", "Control mode: acceleration control mode not supported in PX4 current Firmware.");
			/**
			 * @todo: lacks firmware support, for now
			 */
			return;
		}
		
		if (shape_ == SQUARE) {
			ROS_INFO("Test option: square-shaped path...");
			square_path_motion(loop_rate, mode_);
                            // wait_and_move(mtarget_);
		}
		else if (shape_ == FOLLOW) {
			ROS_INFO("Test follow...");
			ros::spin();
			//wait_and_move(mtarget);
		}		
	}



	Eigen::Vector3d OffboardControl::pos_setpoint(int tr_x, int tr_y, int tr_z){
		/** @todo Give possibility to user define amplitude of movement (square corners coordinates)*/
		return Eigen::Vector3d(tr_x * 1.0f, tr_y * 1.0f, tr_z * 1.0f);	// meters
	}


	/**
	 * @brief Square path motion routine
	 */
	void OffboardControl::square_path_motion(ros::Rate loop_rate, control_mode mode){
		uint8_t pos_target = 0;
		uint8_t pos_target_pre = -1;

		ROS_INFO("Testing...");

		while (ros::ok()) {
			wait_and_move(mtarget_);
			
			if(pos_target_pre != pos_target)
			{
				path_setpoint_.poses.push_back(mtarget_);
				pos_target_pre = pos_target;
			}

			// motion routine
			switch (pos_target) {
			case 1:
				tf::pointEigenToMsg(pos_setpoint(0, 10, 10), mtarget_.pose.position);
				ROS_INFO("Arrived 1 ...");
				break;
			case 2:
				tf::pointEigenToMsg(pos_setpoint(-20, 10, 10), mtarget_.pose.position);
				ROS_INFO("Arrived 2 ...");
				break;
			case 3:
				tf::pointEigenToMsg(pos_setpoint(-20, -10, 10), mtarget_.pose.position);
				ROS_INFO("Arrived 3 ...");
				break;
			case 4:
				tf::pointEigenToMsg(pos_setpoint(0, -10, 10), mtarget_.pose.position);
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
	void OffboardControl::wait_and_move(geometry_msgs::PoseStamped target){
		ros::Rate loop_rate(rate_);
		bool stop = false;
		ros::Time last_time = ros::Time::now();
		Eigen::Vector3d dest;

		double distance;
		double err_th = 2;

		ROS_INFO("Next setpoint: accepted error threshold: %1.3f", err_th);

		while (ros::ok() && !stop ) {
			tf::pointMsgToEigen(target.pose.position, dest);
			tf::pointMsgToEigen(localpos_.pose.position, current_);

			distance = sqrt((dest - current_).x() * (dest - current_).x() +
					(dest - current_).y() * (dest - current_).y() +
					(dest - current_).z() * (dest - current_).z());

			if (distance <= err_th)
			{
				ROS_INFO("setpoint: %2.3f,%2.3f,%2.3f; current position:  %2.3f,%2.3f,%2.3f", current_.x(),current_.y(),current_.z(),dest.x(),dest.y(),dest.z());
				stop = true;
			}

			if (mode_ == POSITION) {
				local_pos_sp_pub_.publish(target);
			}
			else if (mode_ == VELOCITY) {
				if (use_pid_)
					tf::vectorEigenToMsg(pid_.compute_linvel_effort(dest, current_, last_time), vs_.twist.linear);
				else
					tf::vectorEigenToMsg(dest - current_, vs_.twist.linear);
				vel_sp_pub_.publish(vs_);

			}
			else if (mode_ == ACCELERATION) {
				// TODO
				return;
			}
			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}
	
	void OffboardControl::pblish_target(geometry_msgs::PoseStamped target){
		bool stop = false;
		ros::Time last_time = ros::Time::now();
		Eigen::Vector3d dest;

		double distance;
		double err_th = 2;
		
		tf::pointMsgToEigen(target.pose.position, dest);
		tf::pointMsgToEigen(localpos_.pose.position, current_);

		distance = sqrt((dest - current_).x() * (dest - current_).x() +
					(dest - current_).y() * (dest - current_).y() +
					(dest - current_).z() * (dest - current_).z());

		if (distance <= err_th)
		{
			ROS_INFO("setpoint: %2.3f,%2.3f,%2.3f; current position:  %2.3f,%2.3f,%2.3f", current_.x(),current_.y(),current_.z(),dest.x(),dest.y(),dest.z());
		}

		if (mode_ == POSITION) {
			local_pos_sp_pub_.publish(target);
		}
		else if (mode_ == VELOCITY) {
			if (use_pid_)
				tf::vectorEigenToMsg(pid_.compute_linvel_effort(dest, current_, last_time), vs_.twist.linear);
			else
				tf::vectorEigenToMsg(dest - current_, vs_.twist.linear);
			vel_sp_pub_.publish(vs_);

		}
		else if (mode_ == ACCELERATION) {
			// TODO
			return;
		}

	}


	void OffboardControl::modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
	{
		if(shape_ != FOLLOW)
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
				mtarget_.header.stamp = ros::Time::now();
				mtarget_.pose = msg->pose[modelInd];
				mtarget_.pose.position.z = mtarget_.pose.position.z + 10;
				path_setpoint_.poses.push_back(mtarget_);
				pblish_target(mtarget_);
				break;
			}
		}
	}
	
	void OffboardControl::local_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg){
	
		localpos_ = *msg;
		path_current_.poses.push_back(localpos_);
		path_current_pub_.publish(path_current_);
		path_setpoint_pub_.publish(path_setpoint_);
	}

	void OffboardControl::state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state_ = *msg;
	}
	
	void OffboardControl::moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg){		
		std::cout <<"goal is:(" <<msg.pose.position.x<<"," <<msg.pose.position.y<<","<<msg.pose.position.z<<")"<<std::endl;
		mtarget_ = msg;
		mtarget_.pose.position.z = 15;
		path_setpoint_.poses.push_back(mtarget_);	
		wait_and_move(mtarget_);
	}

	bool OffboardControl::wait_to_arm_offboard()
	{
	    
		mavros_msgs::SetMode offb_set_mode;
    	offb_set_mode.request.custom_mode = "OFFBOARD";
    	mavros_msgs::CommandBool arm_cmd;
    	arm_cmd.request.value = true;

	    if( !current_state_.armed &&
	        (ros::Time::now() - last_request_ > ros::Duration(1.0))){
	        if( arming_client_.call(arm_cmd) &&
	            arm_cmd.response.success){
	            ROS_INFO("Vehicle armed");
	        }
			else
			{					
			   last_request_ = ros::Time::now();
			   return false;
			}
       }
		
        if( current_state_.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request_ > ros::Duration(1.0))){
            if( set_mode_client_.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
			else
			{					
               last_request_ = ros::Time::now();
			   return false;
			}
        }			
		return true;   
	}
	
}
