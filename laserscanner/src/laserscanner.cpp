#include "laserscanner.h"

LaserScanner::LaserScanner(void)
{
	ROS_INFO("LaserScanner::LaserScanner()");

	mSocketDescriptor = -1;

	laserDataBuffer = NULL;
	laserDataBufferLength = 0;
}

bool LaserScanner::initialize(const std::string& interface)
{
	if((mSocketDescriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
	{
		ROS_ERROR("LaserScanner::LaserScanner(): socket() failed.");
		return false;
	}

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons(2222);
	sa.sin_addr.s_addr = INADDR_ANY;

	int on = 1;
	setsockopt(mSocketDescriptor, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	if(bind(mSocketDescriptor, (struct sockaddr*) &sa, sizeof(sa)) < 0)
	{
		ROS_ERROR("LaserScanner::LaserScanner(): bind() failed.");
		close(mSocketDescriptor);
		return false;
	}

	struct ip_mreqn mreq;
	memset(&mreq, 0, sizeof(mreq));
	mreq.imr_multiaddr.s_addr = inet_addr("224.0.0.23");
	mreq.imr_address.s_addr = INADDR_ANY;
	mreq.imr_ifindex = if_nametoindex(interface.c_str());

	if(setsockopt(mSocketDescriptor, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
	{
		ROS_ERROR("LaserScanner::LaserScanner(): setsockopt() failed.");
		close(mSocketDescriptor);
		return false;
	}
	
	return true;
}

LaserScanner::~LaserScanner(void)
{
	ROS_INFO("LaserScanner::~LaserScanner()");
	close(mSocketDescriptor);
}

bool LaserScanner::getScan(const std::string& scanner, std::vector<float>& ranges)
{
	if(mSocketDescriptor < 0)
		ROS_FATAL("LaserScanner::getScan(): error: not yet initialized.");
	
	fd_set rfds;
	size_t bytesReady; // how many bytes are going to be read by recv()
	ssize_t bytesRead;
	
	FD_ZERO(&rfds);
	FD_SET(mSocketDescriptor, &rfds);
	
	// Wait until data is ready
	ROS_INFO("LaserScanner::getScan(): waiting for data packet to arrive...");
	if(select(mSocketDescriptor+1, &rfds, NULL, NULL, NULL) < 0)
	{
	    ROS_FATAL("LaserScanner::getScan(): error: select() failed: %s\n", strerror(errno));
	    close(mSocketDescriptor);
	    return false;
	}
	
	// Find out how many bytes are waiting...
	if(ioctl(mSocketDescriptor, FIONREAD, &bytesReady) < 0)
	{
	    ROS_FATAL("LaserScanner::getScan(): error: ioctl() failed: %s\n", strerror(errno));
	    close(mSocketDescriptor);
	    return false;
	}
	
	// Then make sure the buffer is big enough to handle them
	if(laserDataBufferLength < bytesReady)
	{
		if (!(laserDataBuffer = (unsigned char*) realloc(laserDataBuffer, laserDataBufferLength = bytesReady+1)))
		{
			ROS_FATAL("LaserScanner::getScan(): error: out of memory\n");
			close(mSocketDescriptor);
			return false;
	  }
	}

	if((bytesRead = recv(mSocketDescriptor, laserDataBuffer, laserDataBufferLength, 0)) < 0)
	{
	  ROS_FATAL("LaserScanner::getScan(): error: recv() failed: %s\n", strerror(errno));
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

  const unsigned char idOfScanDataFromDesiredScanner = scanner.compare("front") == 0 ? SCANDATA_FRONT : SCANDATA_REAR;

	const unsigned char scannerId = laserDataBuffer[1];

	ROS_INFO("LaserScanner::getScan(): received a packet from scanner 0x%02x", scannerId);

	if((unsigned char)laserDataBuffer[4] == 0xb0 && bytesRead == 732 && scannerId == idOfScanDataFromDesiredScanner)
	{
		// Telegram containing normal scandata from the desired scanner
		// In the old non-ros days, we'd cconvert this into the platform-frame,
		// but bow we just pump out the values as specified in LaserScan.msg
		unsigned char* data = laserDataBuffer + 7;

    for(int i = -180; i < 181; i++)
		{
        unsigned short dist = *(data++);
        dist |= (*(data++)&0x1f)<<8;
        if(dist <= 0x1ff7)
				{
//             _scanner[idx]->_scanDist[cnt] = (float)dist/1000.0; // in Meter
					ranges.push_back((float)dist/1000.0);
//             _scanner[idx]->_scanAngle[cnt] = i*0.5*M_PI/180.0;
        }
    }
    
    return true;
	}
	else
	{
		// Either the packet is from the wrong scanner, or it contains mark data, or something else is wrong.
		// We need to wait for the next packet and hope it contains the desired data.
		ROS_DEBUG("LaserScanner::getScan(): received a packet from scanner 0x%02x with %d bytes, wrong packet, retrying.", scannerId, bytesRead);
		return getScan(scanner, ranges);
	}
	
	return false;
}