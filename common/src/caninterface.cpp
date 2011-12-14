#include "caninterface.h"

CanInterface::CanInterface (const canid_t& receiveFilterId)
{
	ROS_INFO("CanInterface::Can()");
	mReceiveFilterId = receiveFilterId;
}

CanInterface::~CanInterface (void)
{
	ROS_INFO("CanInterface::~Can()");
}

CanFrame CanInterface::createFrame(canid_t id, __u8 d0, __u8 d1, __u8 d2, __u8 d3, __u8 d4, __u8 d5, __u8 d6, __u8 d7)
{
	CanFrame frame;
	frame.can_id = id;
	frame.can_dlc = 8;
	frame.data[0] = d0;
	frame.data[1] = d1;
	frame.data[2] = d2;
	frame.data[3] = d3;
	frame.data[4] = d4;
	frame.data[5] = d5;
	frame.data[6] = d6;
	frame.data[7] = d7;
	
	return frame;
}

bool CanInterface::initialize(void)
{
	mSocketDescriptor = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);

	if(mSocketDescriptor < 0)
	{
		ROS_ERROR("CanInterface::initialize(): couldn't open socket to CAN interface");
		return false;
	}

	struct sockaddr_can addr;
	addr.can_family = AF_CAN;

	// get the interface name from the ros parameter server
	ros::NodeHandle nh;
	std::string interfaceName;
	nh.param<std::string>("can_interface_name", interfaceName, "can0");

	ROS_INFO("CanInterface::initialize(): using can net device %s.", interfaceName.c_str());

	struct ifreq ifr;
	strcpy(ifr.ifr_name, interfaceName.c_str());
	if(ioctl(mSocketDescriptor, SIOCGIFINDEX, &ifr) != 0)
	{
		ROS_ERROR("CanInterface::initialize(): ioctl() failed to set interface index, is the CAN device ready? Try \"modprobe pcan\"?");
		return false;
	}

	addr.can_ifindex = ifr.ifr_ifindex;

	if(bind(mSocketDescriptor, (struct sockaddr *)&addr, sizeof(addr)) != 0)
	{
		ROS_ERROR("CanInterface::initialize(): failed to bind socket to interface.");
		return false;
	}

	// Tell our socket to only receive packets from the IOBOARD by using a can_id filter
	// A filter matches, when <received_can_id> & mask == can_id & mask
	// TODO: check that this works!
	if(mReceiveFilterId)
	{
		struct can_filter rfilter;
		rfilter.can_id   = mReceiveFilterId;
		rfilter.can_mask = mReceiveFilterId;
		setsockopt(mSocketDescriptor, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
	}

	return true;
}

bool CanInterface::send(const CanFrame& frame)
{
	// Most of the time we send a message with only
	// - ID and
	// - CMD in the first byte

	ROS_DEBUG("CanInterface::send(): Sending can frame");
	int bytesTransferred = write(mSocketDescriptor, &frame, sizeof(struct can_frame));

	if(bytesTransferred != sizeof(struct can_frame))
	{
		ROS_ERROR("CanInterface::send(): Couldn't send can frame. Does \"ifconfig\" list canX? Else try \"ifconfig canX up\"");
		return false;
	}

	return true;
}

bool CanInterface::receive(CanFrame *frame, const uint16_t& timeout, const uint8_t expectedCommand)
{
	// what time is it now?
	struct timeval start;
	gettimeofday(&start, 0x0);
	
	while(true)
	{
		struct timeval now;
		gettimeofday(&now, 0x0);

		int elapsed = (now.tv_sec - start.tv_sec)*1000000 + (now.tv_usec - start.tv_usec);
		if(elapsed/1000 > timeout)
		{
			ROS_ERROR("CanInterface::receive(): timeout receiving CAN packet with command %d, returning false after %d msecs", expectedCommand, elapsed / 1000);
			return false;
		}

		// Try to read a packet
		CanFrame receivedFrame;
		int bytesTransferred = read(mSocketDescriptor, &receivedFrame, sizeof(struct can_frame));

		if(bytesTransferred < 0 || bytesTransferred < (int)sizeof(struct can_frame))
		{
			ROS_DEBUG("CanInterface::receive(): couldn't receive packet, bytesRead: %d of %d. Timeout is %d, elapsed %d, retrying", bytesTransferred, sizeof(struct can_frame), timeout, elapsed/1000);
			usleep(timeout * 1000/*ms_to_us*/ / 10);
			continue;
		}

		// Now check whether the received packet contains the right ID and the right CMD
		if((receivedFrame.can_id & mReceiveFilterId) != receivedFrame.can_id)
		{
			ROS_DEBUG("CanInterface::receive(): Received a packet with can_id %d, should be impossible due to CAN_RAW_FILTER. Retrying.", receivedFrame.can_id);
			usleep(timeout * 1000/*ms_to_us*/ / 10);
			continue;
		}
		
		if(expectedCommand != 0 && (receivedFrame.data[7] >> 2) != expectedCommand)
		{
			ROS_WARN("CanInterface::receive(): Received a packet with reply to command %d, but I'm told to receive a packet with command %d. Retrying", (receivedFrame.data[7] >> 2), expectedCommand);
			usleep(timeout * 1000/*ms_to_us*/ / 10);
			continue;
		}

		memset((void*)frame->data, 0, 8);
		frame->can_id = receivedFrame.can_id;
		frame->can_dlc = receivedFrame.can_dlc;
		memcpy((void*)frame->data, (void*)receivedFrame.data, receivedFrame.can_dlc);

		return true;
	}

	ROS_ERROR("CanInterface::receive(): I should never be here!");
	return false;
}
