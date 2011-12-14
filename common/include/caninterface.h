#ifndef CANINTERFACE_H
#define CANINTERFACE_H

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <ros/ros.h>

#include <sys/time.h>

/// This class abstracts a socket-can interface
/// Warning from tschere: Do not send two commands to ID_IO too quickly.

typedef can_frame CanFrame;

class CanInterface
{
private:
	int mSocketDescriptor;
	canid_t mReceiveFilterId;
public:

	CanInterface(const canid_t& receiveFilterId = 255);
	~CanInterface(void);

	bool initialize();
	
	bool send(const CanFrame& frame);
	
	// This method will try to receive a CAN frame into @frame for @tiemout milliseconds. If you pass
	// it a @expectedCommand, it will only return sucecssfully if the received a packet with the same
	// command.
	bool receive(CanFrame *frame, const uint16_t& timeout = 250, const uint8_t expectedCommand = 0);
	
	static CanFrame createFrame(
		canid_t id,
		__u8 d0 = 0,
		__u8 d1 = 0,
		__u8 d2 = 0,
		__u8 d3 = 0,
		__u8 d4 = 0,
		__u8 d5 = 0,
		__u8 d6 = 0,
		__u8 d7 = 0);

};

#endif
