/***************************************************************************
    begin                : 2011-06-07
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"
#include "battery/GetVoltage.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "battery_client");

	ros::NodeHandle n;

	if(argc == 1)
	{
		// just one argument, lets GET the voltage!
		ros::ServiceClient client = n.serviceClient<battery::GetVoltage>("battery/get_voltage");
		battery::GetVoltage srv;
		if(client.call(srv))
		{
			ROS_INFO("Voltage of battery %.2f", srv.response.voltage);
		}
		else
		{
			ROS_ERROR("Failed to call service battery::get_voltage");
			return 1;
		}
	}
	else
	{
		ROS_INFO("usage: battery_client");
		return 1;
	}

	return 0;
}