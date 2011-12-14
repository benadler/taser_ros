/***************************************************************************
    begin                : 2011-06-09
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"
#include "remotecontrol/joy.h"

#include <remotecontrol.h>

RemoteControl mRemoteControl;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "remotecontrol_server");
	ros::NodeHandle n;

	if(!mRemoteControl.initialize())
	{
		ROS_ERROR("Couldn't initialize remote control, exiting.");
		return 1;
	}

	ros::Publisher pub_remotecontrol = n.advertise<remotecontrol::joy>("joy", 1);

	ros::Rate loop_rate(30); // Hz, hopefully

	ROS_INFO("Starting to publish remotecontrol_values");

	while (ros::ok())
	{
		remotecontrol::joy msgJoy;
		msgJoy.axes.resize(2);
		mRemoteControl.getValues(msgJoy.axes[0], msgJoy.axes[1]);

		ROS_DEBUG("main(): will now publish values of axis0 %.4f, axis1 %.4f", msgJoy.axes[0], msgJoy.axes[1]);

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		pub_remotecontrol.publish(msgJoy);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}



