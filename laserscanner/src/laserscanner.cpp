#include "laserscanner.h"

LaserScanner::LaserScanner(void)
{
	ROS_INFO("LaserScanner::LaserScanner()");

	// this level is invalid, but will be updated immediately.
// 	alarmLevelFront = alarmLevelRear = invalid;

	laserDataBuffer = 0;
	laserDataBufferLength = 0;

	if((mSocketDescriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
		ROS_FATAL("LaserScanner::LaserScanner(): socket() failed.");

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons(2222);
	sa.sin_addr.s_addr = INADDR_ANY;

	int on = 1;
	setsockopt(mSocketDescriptor, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	if(bind(mSocketDescriptor, (struct sockaddr*) &sa, sizeof(sa)) < 0)
		ROS_FATAL("LaserScanner::LaserScanner(): bind() failed.");

	struct ip_mreqn mreq;
	memset(&mreq, 0, sizeof(mreq));
	mreq.imr_multiaddr.s_addr = inet_addr("224.0.0.23");
	mreq.imr_address.s_addr = INADDR_ANY;
	mreq.imr_ifindex = if_nametoindex("eth0");

	if(setsockopt(mSocketDescriptor, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
		ROS_FATAL("LaserScanner::LaserScanner(): setsockopt() failed.");
}

LaserScanner::~LaserScanner(void)
{
	ROS_INFO("LaserScanner::~LaserScanner()");
	close(mSocketDescriptor);
}

bool LaserScanner::readPendingDatagram(void)
{
	fd_set rfds;
	size_t bytesReady; // how many bytes are going to be read by recv()
	ssize_t bytesRead;
	
	FD_ZERO(&rfds);
	FD_SET(mSocketDescriptor, &rfds);
	
	// Wait until data is ready
	if(select(mSocketDescriptor+1, &rfds, NULL, NULL, NULL) < 0)
	{
	    ROS_FATAL("LaserScanner::readPendingDatagrams(): error: select() failed: %s\n", strerror(errno));
	    close(mSocketDescriptor);
	    return false;
	}
	
	// Find out how many bytes are waiting...
	if(ioctl(mSocketDescriptor, FIONREAD, &bytesReady) < 0)
	{
	    ROS_FATAL("LaserScanner::readPendingDatagrams(): error: ioctl() failed: %s\n", strerror(errno));
	    close(mSocketDescriptor);
	    return false;
	}
	
	// Then make sure the buffer is big enough to handle them
	if(laserDataBufferLength < bytesReady)
	{
		if (!(laserDataBuffer = (unsigned char*) realloc(laserDataBuffer, laserDataBufferLength = bytesReady+1)))
		{
			ROS_FATAL("LaserScanner::readPendingDatagrams(): error: out of memory\n");
			close(mSocketDescriptor);
			return false;
	  }
	}

	if((bytesRead = recv(mSocketDescriptor, laserDataBuffer, laserDataBufferLength, 0)) < 0)
	{
	  ROS_FATAL("LaserScanner::readPendingDatagrams(): error: recv() failed: %s\n", strerror(errno));
	  close(mSocketDescriptor);
	  return false;
	}
	
	assert(bytesReady == (unsigned)bytesRead);

	// Set the last byte to zero.
	laserDataBuffer[bytesRead] = 0;
	
	if(laserDataBuffer[0] != 0x02)
	{
    ROS_ERROR("STX (Start-Of-Text) not found, Hannes LaserBox is sending weird stuff!");
    return false;
	}

	const unsigned char scannerId = laserDataBuffer[1];

	ROS_INFO("LaserScanner::readPendingDatagram(): received a packet from scanner 0x%02x", scannerId);

	if((unsigned char)laserDataBuffer[4] == 0xb0 && bytesRead == 732)
	{
		// Telegram containing normal scandata.
		// In the old non-ros days, we'd cconvert this into the platform-frame,
		// but bow we just pump out the values as specified in LaserScan.msg
// 		if(scannerId == SCANDATA_FRONT)
// 			convertDatagramToScan(sideFront);
// 		if(scannerId == SCANDATA_REAR)
// 			convertDatagramToScan(sideRear);


	}
	else if((unsigned char)laserDataBuffer[4] == 0xb3)
	{
		// Telegram containing extracted marks and alarm levels.
		// Not used in ROS so far.
		/*
		if(scannerId == MARKDATA_FRONT)
		{
			updateLaserMarks(datagram, sideFront);
			AlarmLevel alarmLevelFrontNew = (AlarmLevel)datagram.at((datagram.at(2)|(datagram.at(3)<<8))+4-1);
			if(alarmLevelFront != alarmLevelFrontNew)
				ROS_INFO("LaserScanner::readPendingDatagram(): new alarm level front: %d", alarmLevelFrontNew);

			alarmLevelFront = alarmLevelFrontNew;
		}

		if(scannerId == MARKDATA_REAR)
		{
			updateLaserMarks(datagram, sideRear);
			AlarmLevel alarmLevelRearNew = (AlarmLevel)datagram.at((datagram.at(2)|(datagram.at(3)<<8))+4-1);

			if(alarmLevelRear != alarmLevelRearNew)
				ROS_INFO("LaserScanner::readPendingDatagram(): new alarm level rear: %d", alarmLevelRearNew);
				
			alarmLevelRear = alarmLevelRearNew;
		}
		*/
	}
	else
	{
		ROS_ERROR("LaserScanner::readPendingDatagram(): unknown packet, ignoring.");
	}
	
	return true;
}