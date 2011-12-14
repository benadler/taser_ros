#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <net/if.h> // for if_nametoindex()
#include <errno.h> // for errno()
#include <sys/ioctl.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>

#include <ros/ros.h>


/// @brief This class receives Laserscans via UDP packets and stores them in LaserScan objects
///
/// We use a small device that collects laserscan data from two Sick LMS 200 scanners via
/// serial port, processes the data and sends us each scan in a UDP multicast packet.
/// The data-format and all other required information can be obtained from Hannes Bistry's
/// diploma-thesis, available at tams-www.informatik.uni-hamburg.de, filename is DA_bistry.pdf.
///
/// Packets from the forward scanner have an address of 0x80 (simple measurement data) or 0x90
/// (extracted marks and area violations). The rear scanner's packets use 0x81 and 0x91, resp.

#define SCANDATA_FRONT 0x80
#define SCANDATA_REAR 0x81
#define MARKDATA_FRONT 0x90
#define MARKDATA_REAR 0x91

enum AlarmLevel
{
	green,	// 0 good to go
	yellow,	// 1 someone's there
	red,		// 2 roadkill
 	invalid // 3 well, invalid.
};

class LaserScanner
{
	private:
		int mSocketDescriptor;
		
		unsigned char *laserDataBuffer;
		size_t laserDataBufferLength;

// 		AlarmLevel alarmLevelFront, alarmLevelRear;	///< used to save the scanner's current alarm levels (red=2/yellow=1/green=0)

		/// This method handles incoming UDP packets and populates the data given as parameters.
		bool readPendingDatagram(void);

	public:
		LaserScanner(void);
		~LaserScanner(void);

// 		AlarmLevel getAlarmLevel(const RobotSide side) const;
};

#endif
