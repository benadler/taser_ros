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

#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))

LaserScanner mLaserScanner;

int main(int argc, char **argv)
{
	// Make sure we were called with either "front" or "rear" as argument
	if(argc != 3 || !(std::string(argv[2]).compare("front") != 0 || std::string(argv[2]).compare("rear") != 0))
	{
		ROS_INFO("usage: laserscanner_server interface [front|rear]");
		ROS_INFO("This will create a node publishing values of either the front or rear.");
		ROS_INFO("laserscanner attached to the given network interface. So, you need two");
		ROS_INFO("laserscanner_server nodes for TASER.");
		return 1;
	}

  // Create a node with a name depending on the selected scanner
  std::string nodename("laserscanner_server_");
	nodename.insert(nodename.size(), argv[2]);
	ros::init(argc, argv, nodename);
	ros::NodeHandle n;

	const std::string interface(argv[1]);

	ROS_INFO("WARNING: The rabbit powercore box IP address is hard-coded to 192.168.0.2.");
	ROS_INFO("         Please make sure %s is in the same subnet.", interface.c_str());

	std::string selectedScanner(argv[2]);
	selectedScanner.insert(0, "lrf_");

	// Switch on the LaserScanner power-supplies and wait if not already switched on
	bool laserScannerPower = false;
	if(!(getIoWarrior(selectedScanner, laserScannerPower) && laserScannerPower))
	{

		ROS_INFO("enabling laserscanner %s", selectedScanner.c_str());
		if(!setIoWarrior(selectedScanner, true))
		{
			ROS_FATAL("Couldn't enable laserscanner-power, exiting");
			exit(1);
		}

		ROS_INFO("Had to switch on laserscanner by myself. Waiting 10s for it to come up.");
		usleep(20000000);
	}
	
	if(!mLaserScanner.initialize(interface))
		ROS_FATAL("Couldn't initialize laserscanner %s on interface %s, exiting", selectedScanner.c_str(), interface.c_str());

	ros::Publisher pub_laserscanner = n.advertise<sensor_msgs::LaserScan>("LaserScan", 1);

	ROS_INFO("Starting to publish laserscans for %s attached to %s", selectedScanner.c_str(), interface.c_str());

	sensor_msgs::LaserScan msgScan;
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
		mLaserScanner.getScan(selectedScanner, msgScan.ranges);

		ROS_DEBUG("main(): publishing %d rays from %s scanner.", selectedScanner.c_str());

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



