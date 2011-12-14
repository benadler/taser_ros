/***************************************************************************
    begin                : 2011-06-09
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#ifndef REMOTECONTROL_H
#define REMOTECONTROL_H

#include <ros/ros.h>

#include <caninterface.h>
#include <cmd_ioboard.h>

/// Messages from the remotecontrol
/// signed char x = message.getData(0); x is the steering, -35 is left, 35 is right
/// signed char y = message.getData(1); y is the speed, -35 is full speed (pulling the trigger), 35 is right.

class RemoteControl
{
private:
	CanInterface mCanInterface;

public:
	RemoteControl();
	~RemoteControl();

	bool initialize();

	// speed is -1.0 to 1.0 cubic, steering is -1.0 to 1.0.
	bool getValues(float &speed, float &steering);
};

#endif
