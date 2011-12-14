/***************************************************************************
    begin                : 2011-06-09
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#ifndef BATTERY_H
#define BATTERY_H

#include <ros/ros.h>
#include <caninterface.h>
#include <cmd_ioboard.h>

/// @class Battery
/// @brief The Battery-class represents the battery, which is connected to the CAN
/// This class only offers one meaningful method, which is getVoltage().

class Battery
{
private:

	CanInterface mCanInterface;

public:
	Battery ();
	~Battery ();

	bool initialize();

	///
	/// This method asks the IOBoard on the CAN for the battery-voltage,
	/// converts the received number to a meaningful value (volts) and
	/// returns it as a float.
	/// @return Battery Voltage in volts
	///
	float getVoltage (void);
};

#endif
