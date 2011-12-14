/***************************************************************************
    begin                : 2011-06-02
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#ifndef IOWARRIOR_H
#define IOWARRIOR_H

#include "ros/ros.h"

// For Parameter lists
#include <XmlRpcValue.h>
#include <string> // std::string
#include <algorithm> // std::transform
#include <cctype> // std::tolower
#include <map> // std::map

#include <iowkit.h>

/**
 * TAMS TASER configuration:
 * 	port 0.0, bit 0, value   1: PTU,			(relay from-left-to-right 1)
 * 	port 0.1, bit 1, value   2: BARRETT-Hand LEFT LED	(relay from-left-to-right 2)
 *	port 0.2, bit 2, value   4: nothing			(relay from-left-to-right 3)
 *	port 0.3, bit 3, value   8: nothing			(relay n/c, switches red LED on iowarrior board)
 *	port 0.4, bit 4, value  16: motorFans			(relay from-left-to-right 4)
 *	port 0.5, bit 5, value  32: nothing			(relay from-left-to-right 5)
 *	port 0.6, bit 6, value  64: lidarFront			(relay from-left-to-right 6)
 *	port 0.7, bit 7, value 128: lidarRear			(relay from-left-to-right 7)
 */

class IoWarrior
{
private:

	IOWKIT_HANDLE mIoWarriorHandle;
	uint32_t mPortMask;
	std::map<uint8_t, std::string> mPortNames;

	int ipow(int base, int exp);
	std::string printBits(uint32_t n);

	void changePortMask(uint8_t port, bool enabled);

	bool sendPortMaskToIoWarrior();

	std::list<uint8_t> getPortNumbers(std::string portname);

	bool retrievePortStatesFromIoWarrior();

	bool isPortPowered(const int8_t port);

	bool setPortPower(std::list<uint8_t> portNumbers, const bool on);

public:
	IoWarrior();
	~IoWarrior();

	bool open();
	void close();

	bool readPortNamesFromParameterServer();

	std::string getPortName(const uint8_t portNumber);

	bool isPortPowered(const std::string port);
	bool setPortPower(const std::string port, const bool on)
	{
		return setPortPower(getPortNumbers(port), on);
	};
};

#endif
