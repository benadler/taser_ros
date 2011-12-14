/***************************************************************************
    begin                : 2011-06-10
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"

#include "remotecontrol/joy.h"
#include "ptu/position.h"
#include "ptu/GetRanges.h"

// global variables
enum PositioningMode {Relative, Absolute};
ros::Publisher pub_ptu_abs, pub_ptu_rel;
float panMin = 1.0, panMax = 1.0, tiltMin = 1.0, tiltMax = 1.0;
PositioningMode posMode = Relative;

void remotecontrol_values_callback(const remotecontrol::joy::ConstPtr& msg)
{
	ptu::position msgPosition;

	if(posMode == Absolute)
	{
		if(msg->axes[1] < 0)
			msgPosition.pan = msg->axes[1] * panMin;
		else
			msgPosition.pan = -msg->axes[1] * panMax;
	
		if(msg->axes[0] < 0)
			msgPosition.tilt = msg->axes[0] * tiltMin;
		else
			msgPosition.tilt = -msg->axes[0] * tiltMax;
	
		ROS_DEBUG("remotecontrol_values_callback(): will now publish absolute ptu position of %.2f / %.2f", msgPosition.pan, msgPosition.tilt);
		pub_ptu_abs.publish(msgPosition);
	}
	else
	{
		msgPosition.pan = -msg->axes[1] * 2.0;
		msgPosition.tilt = -msg->axes[0] * 2.0;
		
		ROS_DEBUG("remotecontrol_values_callback(): will now publish relative ptu position of %.2f / %.2f", msgPosition.pan, msgPosition.tilt);
		pub_ptu_rel.publish(msgPosition);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "remotecontrol_to_ptu");
	ros::NodeHandle n;

	// subscribe to topics from remotecontrol
	ros::Subscriber sub_remotecontrol_values = n.subscribe("/joy", 1, remotecontrol_values_callback);

	// send messages on the position_absolute
	pub_ptu_abs = n.advertise<ptu::position>("ptu/set_position_absolute", 1);
	pub_ptu_rel = n.advertise<ptu::position>("ptu/set_position_relative", 1);
	
	if(argc == 2 && std::string(argv[1]).compare("abs") == 0)
	{
		posMode = Absolute;

		// The remotecontrol delivers values between -1 and 1. Ask the ptu for its range, then set up the scale
		ros::ServiceClient client = n.serviceClient<ptu::GetRanges>("ptu/get_ranges");
		ptu::GetRanges srv;
		if(client.call(srv) && srv.response.success)
		{
			panMin = srv.response.panMin;
			panMax = srv.response.panMax;
			tiltMin = srv.response.tiltMin;
			tiltMax = srv.response.tiltMax;

			ROS_INFO("PTU range is pan %.2f to %.2f, tilt %.2f to %.2f", panMin, panMax, tiltMin, tiltMax);
			ROS_INFO("Starting to forward remotecontrol values to ptu as absolute values");
		}
		else
		{
			ROS_ERROR("Failed to call service ptu/get_ranges, cannot set up scale, exiting");
			return 1;
		}
	}
	else if(argc == 2 && std::string(argv[1]).compare("rel") == 0)
	{
		posMode = Relative;
		ROS_INFO("Starting to forward remotecontrol values to ptu as relative offsets");
	}
	else
	{
		ROS_INFO("usage: remotecontrol_to_ptu [abs|rel]");
		return 1;
	}

	ros::spin();

	return 0;
}



