/***************************************************************************
    begin                : 2011-06-07
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include <battery.h>

Battery::Battery() : mCanInterface(CAN_ID_IO_REPLY)
{
	ROS_INFO("Battery::Battery()");
}

bool Battery::initialize()
{
	ROS_INFO("Battery::initialize()");
	
	if(!mCanInterface.initialize())
	{
		ROS_INFO("Battery::initialize(): initializing CAN failed, returning.");
		return false;
	}
	
	CanFrame frame = CanInterface::createFrame(CAN_ID_IO_CMD, CMD_IOBOARD_CONNECT);
	mCanInterface.send(frame);

	if(mCanInterface.receive(&frame, 250))
	{
		ROS_DEBUG("Battery::initialize(): received reply to CMDD_IOBOARD_CONNECT, can_id %d", frame.can_id);
	}
	else
	{
		ROS_ERROR("Battery::initialize(): receiving can packet failed.");
		return false;
	}
	
	return true;
}

Battery::~Battery()
{
	ROS_INFO("Battery::~Battery()");
}

float Battery::getVoltage (void)
{
	ROS_DEBUG("Battery::getVoltage()");

	float voltage = -1.0;

	CanFrame frame = CanInterface::createFrame(CAN_ID_IO_CMD, CMD_IOBOARD_GETVBATT);
	mCanInterface.send(frame);

	if(mCanInterface.receive(&frame, 250))
	{
		// Ok, we've got the correct packet. Reconstruct the voltage
		const int voltageRaw = (frame.data[0] << 8) | (frame.data[1]);

		// this conversion was found in canplatform.cc and simply copied.
		voltage = (-0.00000010537 * voltageRaw * voltageRaw) + (0.012578 * voltageRaw) - 314;
		ROS_DEBUG("Battery::getVoltage(): voltageRaw is %d, real voltage is %2.2fV", voltageRaw, voltage);
	}
	else
	{
		ROS_ERROR("Battery::getVoltage(): receiving can packet failed, returning invalid voltage.");
	}
	
	return voltage;
}
