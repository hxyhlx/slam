/**
 * @brief SITL test node
 * @file sitl_test_node.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */


#include <test_mavros/offboard_control.h>

int main(int argc, char *argv[])
{
	ROS_INFO("SITL Test node started...");

	if (strcmp(argv[1],"offboard_control") == 0)
	{
		ros::init(argc, argv, "offboard_control");
		testsetup::OffboardControl offboard_control;
		offboard_control.spin(argc, argv);
	}

	return 0;
}
