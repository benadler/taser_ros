/***************************************************************************
    begin                : 2011-06-14
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"
#include "iowarrior_access.h" // for [get|set]IoWarrior()

#include <drive.h>

// Services
#include "drive/GetTemperatures.h"
#include "drive/GetBrakes.h"
#include "drive/SetBrakes.h"

// Messages / Topics
#include "drive/speed.h"
#include "drive/advances.h"
#include "drive/temperatures.h"

DriveChain driveChain;
ros::Publisher pub_temperatures, pub_advances;

void publishTemperatures(const ros::TimerEvent&)
{
	drive::temperatures msgTemperatures;
	driveChain.getMotorTemperatures(msgTemperatures.tempLeft, msgTemperatures.tempRight);
	if(msgTemperatures.tempLeft < 80.0 && msgTemperatures.tempRight < 80.0 && msgTemperatures.tempLeft > 0.0 &&  msgTemperatures.tempRight > 0.0)
	{
		ROS_DEBUG("publishTemperatures(): will now publish temperatures of left %.2f, right %.2f", msgTemperatures.tempLeft, msgTemperatures.tempRight);
		pub_temperatures.publish(msgTemperatures);
	}
	else
	{
		ROS_ERROR("publishTemperatures(): couldn't get temperatures from drive.");
	}
}

void publishAdvances(const ros::TimerEvent&)
{
	drive::advances msgAdvances;
	driveChain.getMotorAdvances(msgAdvances.advanceLeft, msgAdvances.advanceRight);
	ROS_DEBUG("publishAdvances(): will now publish advances of left %.6f, right %.6f", msgAdvances.advanceLeft, msgAdvances.advanceRight);
	pub_advances.publish(msgAdvances);
}

bool get_temperatures(drive::GetTemperatures::Request &req, drive::GetTemperatures::Response &res)
{
	driveChain.getMotorTemperatures(res.tempLeft, res.tempRight);

	ROS_INFO("request: get_temperatures: sending back temperatures: left %.2f, right %.2f", res.tempLeft, res.tempRight);

	return true;
}

bool get_brakes(drive::GetBrakes::Request &req, drive::GetBrakes::Response &res)
{
	res.brakesEnabled = driveChain.getEmergencyStop();

	ROS_INFO("request: get_brakes: sending back response, emergency stop: %d", res.brakesEnabled);

	return true;
}

bool set_brakes(drive::SetBrakes::Request &req, drive::SetBrakes::Response &res)
{
	driveChain.setEmergencyStop(req.enableBrakes);
	
	res.success = driveChain.getEmergencyStop() == req.enableBrakes ? true : false;

	ROS_INFO("request: set_brakes: sending back response, success: %d", res.success);

	return res.success;
}



void set_speed_callback(const drive::speed::ConstPtr& msg)
{
	driveChain.setMotorSpeeds((int)(msg->speedLeft * 1000000.0), (int)(msg->speedRight * 1000000.0));
  ROS_DEBUG("set_speed_callback(): setting speeds %.2f / %.2f according to message on topic", msg->speedLeft, msg->speedRight);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drive_server");
	ros::NodeHandle n;
	
	// Switch on the motor fans power-supplies!
	if(!setIoWarrior("motor_fans", true))
	{
		ROS_FATAL("Couldn't enable motor_fans-power, exiting");
		exit(1);
	}

	ROS_INFO("Initializing DriveChain...");
	driveChain.initialize();
	ROS_INFO("Starting motors...");
	driveChain.startMotors();
	ROS_INFO("Activating emergency stop...");
	driveChain.setEmergencyStop(true);

  // publish messages
	pub_temperatures = n.advertise<drive::temperatures>("drive/get_temperatures", 1);
	pub_advances = n.advertise<drive::advances>("drive/advances", 1000);

	// offer services
	ros::ServiceServer serviceBrakesGet = n.advertiseService("drive/get_brakes", get_brakes);
	ros::ServiceServer serviceBrakesSet = n.advertiseService("drive/set_brakes", set_brakes);
	ros::ServiceServer serviceTemperaturesGet = n.advertiseService("drive/get_temperatures", get_temperatures);
	
	// subscribe to topics to set the speed
	ros::Subscriber sub_speed = n.subscribe("drive/set_speed", 1, set_speed_callback);
	
	// Set timers for publishing stuff
// 	ros::Timer t1 = n.createTimer(ros::Duration(0.05), publishAdvances);
// 	ros::Timer t2 = n.createTimer(ros::Duration(10), publishTemperatures);

	ROS_INFO("Your wish is my command.");

	ros::spin();

	return 0;
}



