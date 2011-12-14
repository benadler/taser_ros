/***************************************************************************
    begin                : 2011-06-09
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ptuwrapper.h"

PtuWrapper::PtuWrapper()
{
	ROS_INFO("PtuWrapper::PtuWrapper()");
}

bool PtuWrapper::initialize()
{
	// get parameters from the ros parameter server
	ros::NodeHandle nh;

	double spdPan, spdTilt;
	double accPan, accTilt;

	nh.param<std::string>("/ptu/device_file", mDeviceFile, "/dev/ttyS0");
	nh.param<double>("/ptu/spdPan", spdPan, 20.0);
	nh.param<double>("/ptu/spdTilt", spdTilt, 20.0);
	nh.param<double>("/ptu/accPan", accPan, 100.0);
	nh.param<double>("/ptu/accTilt", accTilt, 100.0);

	ROS_INFO("PtuWrapper::initialize(): using device file %s, speed %f / %f deg/s and acceleration %f / %f deg/s^2 for pan/tilt.", mDeviceFile.c_str(), spdPan, spdTilt, accPan, accTilt);

	mPtu = new PTU(mDeviceFile);

	try
	{
		mPtu->init();
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::initialize(): couldn't initialize PTU, error: %s", e.text().c_str());
		return false;
	}

	setMaximumSpeed(spdPan, spdTilt);
	usleep(100000);
	setAcceleration(accPan, accTilt);
	usleep(100000);

	// This probably doesn't need to be configurable from the outside.
	mPtu->setHoldPowerMode(PTU_PAN,  HPM_LOW); // HPM_OFF, HPM_LOW, HPM_REGULAR
	usleep(100000);
	mPtu->setHoldPowerMode(PTU_TILT, HPM_LOW); // HPM_OFF, HPM_LOW, HPM_REGULAR
	usleep(100000);
	mPtu->setMovePowerMode(PTU_PAN,  MPM_HIGH); // MPM_LOW, MPM_REGULAR, MPM_HIGH
	usleep(100000);
	mPtu->setMovePowerMode(PTU_TILT, MPM_HIGH); // MPM_LOW, MPM_REGULAR, MPM_HIGH

	return true;
}

PtuWrapper::~PtuWrapper()
{
	ROS_INFO("PtuWrapper::~PtuWrapper()");
	delete mPtu;
}

bool PtuWrapper::waitForCompletion()
{
	try
	{
		mPtu->waitForCompleted();
		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::waitForCompletion(): couldn't wait for completion, error: %s", e.text().c_str());
		return false;
	}
}

bool PtuWrapper::stop()
{
	try
	{
		mPtu->stop();
		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::stop(): couldn't stop, error: %s", e.text().c_str());
		return false;
	}
}

bool PtuWrapper::getPositionRanges(float& panMin, float& panMax, float& tiltMin, float& tiltMax)
{
	try
	{
		panMin = DEG2RAD(mPtu->getMinPosition(PTU_PAN, PTU_DEGREE));
		panMax = DEG2RAD(mPtu->getMaxPosition(PTU_PAN, PTU_DEGREE));

		tiltMin = DEG2RAD(mPtu->getMinPosition(PTU_TILT, PTU_DEGREE));
		tiltMax = DEG2RAD(mPtu->getMaxPosition(PTU_TILT, PTU_DEGREE));

		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::getPositionRanges(): couldn't get position ranges, error: %s", e.text().c_str());
		return false;
	}
}

bool PtuWrapper::getAcceleration(float& pan, float& tilt)
{
	try
	{
		pan = DEG2RAD(mPtu->getAccel(PTU_PAN, PTU_DEGREE));
		tilt = DEG2RAD(mPtu->getAccel(PTU_TILT, PTU_DEGREE));
		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::setAcceleration(): couldn't set acceleration to %f / %f, error: %s", pan, tilt, e.text().c_str());
		return false;
	}
}

bool PtuWrapper::setAcceleration(float pan, float tilt)
{
	try
	{
		mPtu->setAccel(RAD2DEG(pan), RAD2DEG(tilt), PTU_DEGREE);
		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::setAcceleration(): couldn't set acceleration to %f / %f, error: %s", pan, tilt, e.text().c_str());
		return false;
	}
}

bool PtuWrapper::getMaximumSpeed(float& pan, float& tilt)
{
	try
	{
		pan = DEG2RAD(mPtu->getSpeed(PTU_PAN, PTU_DEGREE));
		tilt = DEG2RAD(mPtu->getSpeed(PTU_TILT, PTU_DEGREE));
		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::getSpeed(): couldn't get speeds, error: %s", e.text().c_str());
		return false;
	}
}

bool PtuWrapper::setMaximumSpeed(float pan, float tilt)
{
	try
	{
		mPtu->setSpeed(RAD2DEG(pan), RAD2DEG(tilt), PTU_DEGREE);
		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::setSpeed(): couldn't set speed to %f / %f, error: %s", pan, tilt, e.text().c_str());
		return false;
	}
}

bool PtuWrapper::setPositionAbsolute(float pan, float tilt)
{
	try
	{
		mPtu->setPosition(RAD2DEG(pan), RAD2DEG(tilt), PTU_DEGREE);
		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::setPositionAbsolute(): couldn't set position to %f / %f, error: %s", pan, tilt, e.text().c_str());
		return false;
	}
}

bool PtuWrapper::setPositionRelative(float pan, float tilt)
{
	try
	{
		mPtu->setRelPosition(RAD2DEG(pan), RAD2DEG(tilt), PTU_DEGREE);
		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::setPositionRelative(): couldn't set relative position to %f / %f, error: %s", pan, tilt, e.text().c_str());
		return false;
	}
}

bool PtuWrapper::getPositionAbsolute(float& pan, float& tilt)
{
	try
	{
		pan = DEG2RAD(mPtu->getPosition(PTU_PAN, PTU_DEGREE));
		tilt = DEG2RAD(mPtu->getPosition(PTU_TILT, PTU_DEGREE));
		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::getPositionAbsolute(): couldn't get positions, error: %s", e.text().c_str());
		return false;
	}
}

bool PtuWrapper::getPositionDesired(float &pan, float &tilt)
{
	try
	{
		pan = DEG2RAD(mPtu->getDesPosition(PTU_PAN, PTU_DEGREE));
		tilt = DEG2RAD(mPtu->getDesPosition(PTU_TILT, PTU_DEGREE));
		return true;
	}
	catch(PTUException e)
	{
		ROS_ERROR("PtuWrapper::getPositionDesired(): couldn't get desired positions, error: %s", e.text().c_str());
		return false;
	}
}
