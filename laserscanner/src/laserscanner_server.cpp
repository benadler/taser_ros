/***************************************************************************
    begin                : 2011-06-09
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"

#include "laserscanner.h"

#include "iowarrior_access.h" // for switchIoWarrior()

#include <sensor_msgs/LaserScan.h>

#define DEG2RAD(DEG) ((DEG)*((PI)/(180.0)))

LaserScanner mLaserScanner;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserscanner_server");
	ros::NodeHandle n;
	
	// Switch on the LaserScanner power-supplies and wait if not already switched on
	bool laserScannerPower = false;
	if(
		!(getIoWarrior("lrf_front", laserScannerPower) && laserScannerPower)
		||
		!(getIoWarrior("lrf_rear", laserScannerPower) && laserScannerPower)
	)
	{
		if(!setIoWarrior("lrf_front", true) || !switchIoWarrior("lrf_rear", true))
		{
			ROS_FATAL("Couldn't enable laserscanner-power, exiting");
			exit(1);
		}
		ROS_INFO("Had to switch on laserscanner by myself. Waiting 10s for them to come up.");
		usleep(10000000);
	}

	if(!mLaserScanner.initialize())
	{
		ROS_ERROR("Couldn't set up UDP socket, exiting.");
		return 1;
	}

	ros::Publisher pub_laserscanner = n.advertise<laserscanner::LaserScan>("LaserScan", 1);

	// Not needed, the select() in laserscanner's readPendingDatagrams will block until data is ready
// 	ros::Rate loop_rate(20); // Hz, hopefully

	ROS_INFO("Starting to publish laserscans");

	laserscanner::LaserScan msgScan;
	msgScan.angle_min = DEG2RAD(-90.0f);
	msgScan.angle_max = DEG2RAD(90.0f);
	msgScan.angle_increment = DEG2RAD(0.5f);
	
	// time_increment is amount of seconds between rays. This is fscked up in LMS200
	// because with 0.5 deg resolution and 75 rounds per second motor speed, the LMS
	// scans the 180 deg window twice, with a 0.5 deg offset in between scans. So data
	// collection works kind of interleaved. Here, we pretend sequential scanning
	msgScan.time_increment = 0.00001846f;
	msgScan.scan_time = 0.02666666;
	
	msgScan.range_min = 0.1f;
	msgScan.range_max = 20.0f;
	
	while (ros::ok())
	{
		mLaserScanner.getValues(msgJoy.axes[0], msgJoy.axes[1]);

		ROS_DEBUG("main(): will now publish values of axis0 %.4f, axis1 %.4f", msgJoy.axes[0], msgJoy.axes[1]);

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		pub_laserscanner.publish(msgScan);

		ros::spinOnce();

// 		loop_rate.sleep();
	}

	return 0;
}



