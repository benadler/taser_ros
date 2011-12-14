/***************************************************************************
    begin                : 2011-06-09
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "remotecontrol.h"

RemoteControl::RemoteControl() : mCanInterface(CAN_ID_IO_REPLY)
{
	ROS_INFO("RemoteControl::RemoteControl()");
}

bool RemoteControl::initialize()
{
	ROS_INFO("RemoteControl::initialize()");

	if(!mCanInterface.initialize())
	{
		ROS_INFO("RemoteControl::initialize(): initializing CAN failed, returning.");
		return false;
	}
	
	CanFrame frame = CanInterface::createFrame(CAN_ID_IO_CMD, CMD_IOBOARD_CONNECT);
	mCanInterface.send(frame);

	if(mCanInterface.receive(&frame, 250))
	{
		ROS_DEBUG("RemoteControl::initialize(): received reply to CMDD_IOBOARD_CONNECT, can_id %d", frame.can_id);
	}
	else
	{
		ROS_ERROR("RemoteControl::initialize(): receiving can packet failed.");
		return false;
	}
	
	return true;
}

RemoteControl::~RemoteControl()
{
	ROS_INFO("RemoteControl::~RemoteControl()");
}

bool RemoteControl::getValues(float &speed, float &steering)
{
	ROS_DEBUG("RemoteControl::getValues()");
	
	
	CanFrame frame = CanInterface::createFrame(CAN_ID_IO_CMD, CMD_IOBOARD_GETJOYVAL);
	mCanInterface.send(frame);

	if(mCanInterface.receive(&frame, 250))
	{
			// steering is frame.data[0], usually between -30 and 30, sometimes up to 33.
			int8_t steering_int = frame.data[0];
			if(steering_int > +36) steering_int = +36;
			if(steering_int < -36) steering_int = -36;
			if(abs(steering_int) < 4) steering_int = 0; // avoid creeping due to r/c calibration errors.
			steering = (float)steering_int / 36.0;

			int8_t speed_int = -frame.data[1];
			if(speed_int > +35) speed_int = +35;
			if(speed_int < -35) speed_int = -35;
			if(abs(speed_int) < 4) speed_int = 0; // avoid creeping due to r/c calibration errors.
			speed = (float)speed_int / 35.0;

			// speed to the power-of-three for better handling (expo-value)
			speed = speed * speed * speed;

			ROS_DEBUG("RemoteControl::getValues(): speed %.3f, steering %.3f", speed, steering);
		return true;
	}
	else
	{
		ROS_ERROR("Battery::getVoltage(): receiving can packet failed, returning invalid voltage.");
		return false;
	}
}
