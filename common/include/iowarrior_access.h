#ifndef IOWARRIOR_ACCESS_H
#define IOWARRIOR_ACCESS_H

#include "ros/ros.h"
#include "iowarrior/SetPower.h"
#include "iowarrior/GetPower.h"

bool setIoWarrior(const std::string &portName, const bool& enable)
{
	if(enable)
		ROS_INFO("Enabling %s power on iowarrior.", portName.c_str());
	else
		ROS_INFO("Disabling %s power on iowarrior.", portName.c_str());
	
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<iowarrior::SetPower>("iowarrior/set_power");
	iowarrior::SetPower srvIoWarrior;
	srvIoWarrior.request.port = portName;
	srvIoWarrior.request.status = enable;

	if(client.call(srvIoWarrior) && srvIoWarrior.response.status)
	{
		ROS_INFO("Setting %s power succeeded", portName.c_str());
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call service iowarrior/set_power, setting %s power failed", portName.c_str());
		return false;
	}
}

bool getIoWarrior(const std::string &portName, bool& enabled)
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<iowarrior::GetPower>("iowarrior/get_power");
	iowarrior::GetPower srvIoWarrior;
	srvIoWarrior.request.port = portName;

	if(client.call(srvIoWarrior))
	{
		ROS_INFO("iowarrior: %s power is %d", srvIoWarrior.response.port.c_str(), srvIoWarrior.response.status);
		enabled = srvIoWarrior.response.status;
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call service iowarrior/get_power, getting %s power failed", portName.c_str());
		return false;
	}
}

#endif