/***************************************************************************
    begin                : 2011-06-07
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include "battery/GetVoltage.h"

#include <battery.h>

Battery mBattery;
ros::Publisher pub_battery;

bool getVoltage(battery::GetVoltage::Request &req, battery::GetVoltage::Response &res)
{
	res.voltage = mBattery.getVoltage();

	ROS_INFO("request: getpower: sending back response: %2.2f", res.voltage);

	return true;
}

void publishVoltageCallback(const ros::TimerEvent&)
{
	std_msgs::Float32 msgVoltage;
	msgVoltage.data = mBattery.getVoltage();

	ROS_DEBUG("main(): will now publish voltage of %2.2fV", msgVoltage.data);

	/**
	 * The publish() function is how you send messages. The parameter
	 * is the message object. The type of this object must agree with the type
	 * given as a template parameter to the advertise<>() call, as was done
	 * in the constructor above.
	 */
	pub_battery.publish(msgVoltage);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "battery_server");
	ros::NodeHandle n;

	if(!mBattery.initialize())
	{
		ROS_ERROR("Couldn't initialize battery, exiting.");
		return 1;
	}

	pub_battery = n.advertise<std_msgs::Float32>("battery/voltage", 1, true); // latch it!

	ros::ServiceServer serviceGetVoltage = n.advertiseService("battery/get_voltage", getVoltage);

	/**
	 * As you can see above, we offer both a service and publish messages on our topic.
	 *
	 * With services, you usually use ros::spin() to enter the event-loop and never return.
	 *
	 * When publishing on a topic, you usually use ros::Rate and ros::spinOnce().
	 *
	 * When doing both, use ros::spin() to handle the service and a ros::timer to call the
	 * publishing method at the given interval.
	 */

	ros::Timer timer = n.createTimer(ros::Duration(10), publishVoltageCallback);

	ROS_INFO("Starting to publish battery/voltage");

	ros::spin();

	return 0;
}



