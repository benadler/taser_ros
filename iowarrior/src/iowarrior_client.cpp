/***************************************************************************
    begin                : 2011-06-02
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "iowarrior/GetPower.h"
#include "iowarrior/SetPower.h"

// converts string to streamable value, and returns true on success and false otherwise.
template <class T>
bool from_string(T &Value,const std::string &str,std::ios_base & (*f)(std::ios_base&))
{
	std::istringstream stream(str);
	stream>>f>>Value;
	return (!stream.fail()) && stream.get()==std::istringstream::traits_type::eof();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "iowarrior_client");

	ros::NodeHandle n;


	if(argc == 2)
	{
		// just one argument, lets GET the ports status!
		ros::ServiceClient client = n.serviceClient<iowarrior::GetPower>("iowarrior/get_power");
		iowarrior::GetPower srv;
		srv.request.port = std::string(argv[1]);
		if(client.call(srv))
		{
			ROS_INFO("Status of port %s: %d", srv.response.port.c_str(), srv.response.status);
		}
		else
		{
			ROS_ERROR("Failed to call service iowarrior::get_power");
			return 1;
		}
	}
	else if(argc == 3)
	{
		// two arguments, SET the ports status!
		ros::ServiceClient client = n.serviceClient<iowarrior::SetPower>("iowarrior/set_power");
		iowarrior::SetPower srv;

		srv.request.port = std::string(argv[1]);

		// parse the cmdline argument into a boolean
		bool status;
		std::string statusString(argv[2]);
		// convert to lower case
		std::transform(statusString.begin(), statusString.end(), statusString.begin(), static_cast<int (*)(int)>(std::tolower));

		if(
		    statusString.compare("true") == 0
		    || statusString.compare("on") == 0
		    || statusString.compare("1") == 0
		    || statusString.compare("enable") == 0
		    || statusString.compare("enabled") == 0
		)
		{
			status = true;
			statusString = "on";
		}
		else if(
		    statusString.compare("false") == 0
		    || statusString.compare("off") == 0
		    || statusString.compare("0") == 0
		    || statusString.compare("disable") == 0
		    || statusString.compare("disabled") == 0
		)
		{
			status = false;
			statusString = "off";
		}
		else
		{
			ROS_ERROR("Couldn't parse status value \"%s\" into a boolean, try true, on, 1, enable, enabled or false, off, 0, disable, disabled", argv[2]);
			return 1;
		}

		ROS_INFO("Going to switch port %s %s", srv.request.port.c_str(), statusString.c_str());

		srv.request.status = status;

		if(client.call(srv))
		{
			ROS_INFO("Status of port %s: %d", srv.response.port.c_str(), srv.response.status);
		}
		else
		{
			ROS_ERROR("Failed to call service iowarrior::set_power");
			return 1;
		}
	}
	else
	{
		ROS_INFO("usage: iowarrior_client portname [true|false]");
		return 1;
	}

	return 0;
}