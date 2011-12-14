/***************************************************************************
    begin                : 2011-06-14
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"

// Services
#include "drive/GetTemperatures.h"
#include "drive/GetBrakes.h"
#include "drive/SetBrakes.h"

// Messages / Topics
#include "drive/speed.h"
#include "drive/advances.h"
#include "drive/temperatures.h"

#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drive_client");

	ros::NodeHandle n;

	if(argc == 2 && std::string(argv[1]).compare("getbrakes") == 0)
	{
		// get brakes from drive_server
		ros::ServiceClient client = n.serviceClient<drive::GetBrakes>("drive/get_brakes");
		drive::GetBrakes srv;

		if(client.call(srv))
		{
			ROS_INFO("brakes enabled: %d", srv.response.brakesEnabled);
		}
		else
		{
			ROS_ERROR("Failed to call service drive::GetBrakes");
			return 1;
		}
	}
	else if(argc == 2 && std::string(argv[1]).compare("gettemperatures") == 0)
	{
		// get temps from drive_server
		ros::ServiceClient client = n.serviceClient<drive::GetTemperatures>("drive/get_temperatures");
		drive::GetTemperatures srv;

		if(client.call(srv))
		{
			ROS_INFO("motor temperatures: left %.2f, right %.2f", srv.response.tempLeft, srv.response.tempRight);
		}
		else
		{
			ROS_ERROR("Failed to call service drive/get_temperatures");
			return 1;
		}
	}
	else if(argc == 3 && std::string(argv[1]).compare("setbrakes") == 0)
	{
		// set brakes!
		bool enableBrakes = false;
		if(std::string(argv[2]).compare("on") == 0 || std::string(argv[2]).compare("1") == 0)
		{
			enableBrakes = true;
		}
		else if(std::string(argv[2]).compare("off") == 0 || std::string(argv[2]).compare("0") == 0)
		{
			enableBrakes = false;
		}
		else
		{
			ROS_ERROR("Couldn't parse brake-value \"on\", \"off\", \"1\" or \"0\" from \"%s\"", argv[2]);
			return 1;
		}

		ros::ServiceClient client = n.serviceClient<drive::SetBrakes>("drive/set_brakes");
		drive::SetBrakes srv;
		srv.request.enableBrakes = enableBrakes;

		if(client.call(srv))
		{
			ROS_INFO("setting brakes to %d, success: %d", srv.request.enableBrakes, srv.response.success);
		}
		else
		{
			ROS_ERROR("Failed to call service drive/set_brakes");
			return 1;
		}

	}
	else
	{
		ROS_INFO("usage for getting brakes: drive_client getbrakes");
		ROS_INFO("usage for setting brakes: drive_client setbrakes on|off");
		ROS_INFO("usage for getting temperatures: drive_client gettemperatures");
		return 1;
	}

	return 0;
}