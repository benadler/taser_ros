/***************************************************************************
    begin                : 2011-06-02
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"

#include "iowarrior.h"

#include "iowarrior/GetPower.h"
#include "iowarrior/SetPower.h"

IoWarrior ioWarrior;

bool getPower(iowarrior::GetPower::Request &req, iowarrior::GetPower::Response &res )
{
	res.status = ioWarrior.isPortPowered(req.port);
	res.port = req.port;

	ROS_INFO("request: getpower: port=%s, sending back response: %d", req.port.c_str(), res.status);

	return true;
}

bool setPower(iowarrior::SetPower::Request &req, iowarrior::SetPower::Response &res)
{
	ROS_INFO("request: setpower: port=%s, status=%d, setting port.", req.port.c_str(), req.status);

	if(!ioWarrior.setPortPower(req.port, req.status))
	{
		ROS_ERROR("request: setpower: port=%s, status=%d, setting port failed.", req.port.c_str(), req.status);
	}

	res.port = std::string("test");//req.port;
	res.status = ioWarrior.isPortPowered(req.port);

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "iowarrior_server");
	ros::NodeHandle n;

	if(!ioWarrior.readPortNamesFromParameterServer())
	{
		ROS_ERROR("Couldn't read iowarrior port names from parameter server, exiting.");
		return 1;
	}

	if(!ioWarrior.open())
	{
		ROS_ERROR("Couldn't open iowarrior, exiting.");
		return 1;
	}

	ros::ServiceServer serviceGetPower = n.advertiseService("iowarrior/get_power", getPower);
	ros::ServiceServer serviceSetPower = n.advertiseService("iowarrior/set_power", setPower);
	ROS_INFO("Your wish is my command.");
	ros::spin();

	return 0;
}



