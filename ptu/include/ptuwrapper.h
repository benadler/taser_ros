/***************************************************************************
    begin                : 2011-06-09
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#ifndef PTUWRAPPER_H
#define PTUWRAPPER_H

#include <ros/ros.h>

#include <ptu.h>

/*
 * ALL ANGLES ARE RADIANS!
 */

#define RAD2DEG(RAD) ((RAD)*180/M_PI)
#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))

class PtuWrapper
{
private:
	PTU* mPtu;
	std::string mDeviceFile;

public:
	PtuWrapper();
	~PtuWrapper();

	bool initialize();
	bool waitForCompletion();
	bool stop();
	bool getPositionRanges(float& panMin, float& panMax, float& tiltMin, float& tiltMax);
	bool getAcceleration(float& pan, float& tilt);
	bool setAcceleration(float pan, float tilt);
	bool getMaximumSpeed(float& pan, float& tilt);
	bool setMaximumSpeed(float pan, float tilt);
	bool setPositionAbsolute(float pan, float tilt);
	bool setPositionRelative(float pan, float tilt);
	bool getPositionAbsolute(float& pan, float& tilt);
	
	/*
	 * Gets the desired position (might be different from current position when PTU is still moving to desired position)
	 */
	bool getPositionDesired(float &pan, float &tilt);
};

#endif
