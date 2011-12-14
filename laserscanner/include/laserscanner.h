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

class LaserScanner
{
	private:
		int mSocketDescriptor;

		unsigned char *laserDataBuffer;
		size_t laserDataBufferLength;

	public:
		LaserScanner(void);
		~LaserScanner(void);
		
		bool initialize(const std::string& interface);
		
		bool getScan(const std::string& scanner, std::vector<float>& ranges);
};

#endif
