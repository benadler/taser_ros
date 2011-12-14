/***************************************************************************
    begin                : 2011-06-14
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"

#include "remotecontrol/joy.h"
#include "drive/speed.h"
#include "drive/SetBrakes.h"
#include "iowarrior/SetPower.h"

#define SPEED_BASE 1.0 // thats meters per second
#define SPEED_STEERING 0.5 // thats meters per second

// global variables
// enum PositioningMode {Relative, Absolute};
ros::Publisher pub_drive_speed;
// float panMin = 1.0, panMax = 1.0, tiltMin = 1.0, tiltMax = 1.0;
// PositioningMode posMode = Relative;

void remotecontrol_values_callback(const remotecontrol::joy::ConstPtr& msg)
{
	drive::speed msgSpeed;
	msgSpeed.speedLeft = SPEED_BASE * msg->axes[0];
	msgSpeed.speedRight = SPEED_BASE * msg->axes[0];
	
	msgSpeed.speedLeft += SPEED_STEERING * msg->axes[1];
	msgSpeed.speedRight -= SPEED_STEERING * msg->axes[1];
	
	ROS_DEBUG("remotecontrol_values_callback(): publishing speeds of %.2f %.2f.", msgSpeed.speedLeft, msgSpeed.speedRight);
	pub_drive_speed.publish(msgSpeed);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "remotecontrol_to_drive");
	ros::NodeHandle n;

	// subscribe to topics from remotecontrol
	ros::Subscriber sub_remotecontrol_values = n.subscribe("joy", 1, remotecontrol_values_callback);

	// send messages on the position_absolute
	pub_drive_speed = n.advertise<drive::speed>("drive/set_speed", 1);
	
	ROS_INFO("Disabling brakes on drivechain.");
	
	ros::ServiceClient client;
	
	client = n.serviceClient<drive::SetBrakes>("drive/set_brakes");
	drive::SetBrakes srvDrive;
	srvDrive.request.enableBrakes = false;

	if(client.call(srvDrive) && srvDrive.response.success)
	{
		ROS_INFO("Disabling brakes succeeded");
	}
	else
	{
		ROS_ERROR("Failed to call service drive/set_brakes");
		return 1;
	}

	ROS_INFO("Forwarding remotecontrol values to drive.");

	ros::spin();

	return 0;
}



