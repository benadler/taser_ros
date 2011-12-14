/***************************************************************************
    begin                : 2011-06-02
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "iowarrior.h"

IoWarrior::IoWarrior()
{
	mIoWarriorHandle = NULL;

	ROS_INFO("IoWarrior::IoWarrior(): using \"%s\" to access USB devices.", IowKitVersion());
}

IoWarrior::~IoWarrior()
{
	IowKitCloseDevice(mIoWarriorHandle);
}

bool IoWarrior::readPortNamesFromParameterServer()
{
	ROS_INFO("IoWarrior::readPortNamesFromParameterServer(): reading port names from parameter server...");

	// Try to read port names from the parameter sever
	XmlRpc::XmlRpcValue portNameList;
	ros::NodeHandle nh;
	nh.getParam("/iowarrior/port_names", portNameList);

	if(portNameList.getType() != XmlRpc::XmlRpcValue::TypeArray)
	{
		ROS_ERROR("IoWarrior: readPortNamesFromParameterServer: reading port names from parameter server failed, /iowarrior/portNames is not an array. Please set it to an array with strings of objects names, in the same order as connected on the iowarrior.");
		return false;
	}

	for(int32_t i = 0; i < portNameList.size(); ++i)
	{
		if(portNameList[i].getType() != XmlRpc::XmlRpcValue::TypeString)
		{
			ROS_ERROR("IoWarrior::readPortNamesFromParameterServer(): reading port names from parameter server failed, /iowarrior/portNames contains non-strings. It should contain a list of strings in the order in which the devices are connected to the iowarrior.");
			return false;
		}

		std::string name = static_cast<std::string>(portNameList[i]);
		ROS_INFO("IoWarrior::readPortNamesFromParameterServer(): port %d is named \"%s\"", i, name.c_str());

		// convert to lower case
		std::transform(name.begin(), name.end(), name.begin(), static_cast<int (*)(int)>(std::tolower));
		mPortNames.insert(std::pair<uint8_t, std::string>(i, name));
	}

	return true;
}

bool IoWarrior::open()
{
	mIoWarriorHandle = IowKitOpenDevice();
	if(mIoWarriorHandle == NULL)
	{
		ROS_ERROR("IoWarrior::open(): couldn't open any iowarrior devices, quitting.");
		return false;
	}

	ROS_INFO("IoWarrior::open(): found %ld iowarrior(s)", IowKitGetNumDevs());

	IowKitSetTimeout(mIoWarriorHandle, 1000);
	IowKitSetWriteTimeout(mIoWarriorHandle, 1000);

	if(IowKitGetProductId(mIoWarriorHandle) == IOWKIT_PRODUCT_ID_IOW24)
		ROS_INFO("IoWarrior::open(): using an iowarrior 24.");
	else if(IowKitGetProductId(mIoWarriorHandle) == IOWKIT_PRODUCT_ID_IOW40)
		ROS_INFO("IoWarrior::open(): using an iowarrior 40.");
	else
		ROS_INFO("IoWarrior::open(): using an unknown iowarrior.");

	if(IowKitGetRevision(mIoWarriorHandle) < IOW_NON_LEGACY_REVISION) ROS_INFO("IoWarrior::open(): using an iowarrior with legacy firmware.");

//   char* buffer[18];
//   IowKitGetSerialNumber(mIoWarriorHandle, buffer);
//   ROS_INFO("Serial number of iowarrior board: %ws", buffer);

	ROS_ASSERT(retrievePortStatesFromIoWarrior());

	return true;
}

bool IoWarrior::isPortPowered(const std::string portname)
{
	std::list<uint8_t> portNumbers = getPortNumbers(portname);

	if(!portNumbers.size())
	{
		ROS_ERROR("IoWarrior::isPortPowered(): portname %s does not exist, returning false", portname.c_str());
		return false;
	}
	else if(portNumbers.size() > 1)
	{
		ROS_WARN("IoWarrior::isPortPowered(): portname %s has more than one physical port, returning result of first physical port", portname.c_str());
	}

	return isPortPowered(portNumbers.front());
};

bool IoWarrior::isPortPowered(const int8_t portNumber)
{
	retrievePortStatesFromIoWarrior();

	if((mPortMask & ipow(2, portNumber)) == 0)
	{
		ROS_INFO("IoWarrior::isPortPowered(): port %d is powered on.", portNumber);
		return true;
	}
	else
	{
		ROS_INFO("IoWarrior::isPortPowered(): port %d is powered off.", portNumber);
		return false;
	}
}

bool IoWarrior::setPortPower(std::list<uint8_t> portNumbers, const bool on)
{
	if(portNumbers.empty()) return false;

	while(!portNumbers.empty())
	{
		changePortMask(portNumbers.front(), on);
		portNumbers.pop_front();
	}

	ROS_ASSERT(sendPortMaskToIoWarrior());

	return true;
}

bool IoWarrior::sendPortMaskToIoWarrior()
{
	ROS_INFO("IoWarrior::sendPortMaskToIoWarrior(): going to send portMask:\t%s", printBits(mPortMask).c_str());

	IOWKIT_REPORT rep;
	IOWKIT56_IO_REPORT rep56;

	switch (IowKitGetProductId(mIoWarriorHandle))
	{
		// Write simple value to IOW24
	case IOWKIT_PRODUCT_ID_IOW24:
		memset(&rep, 0xff, sizeof(IOWKIT_REPORT));
		rep.ReportID = 0;
		rep.Bytes[0] = (UCHAR) mPortMask;
		return IowKitWrite(mIoWarriorHandle, IOW_PIPE_IO_PINS, (char*)&rep, IOWKIT24_IO_REPORT_SIZE) == IOWKIT24_IO_REPORT_SIZE;
		// Write simple value to IOW40
	case IOWKIT_PRODUCT_ID_IOW40:
		memset(&rep, 0xff, sizeof(IOWKIT_REPORT));
		rep.ReportID = 0;
		rep.Bytes[3] = (UCHAR) mPortMask;
		return IowKitWrite(mIoWarriorHandle, IOW_PIPE_IO_PINS, (char*)&rep, IOWKIT40_IO_REPORT_SIZE) == IOWKIT40_IO_REPORT_SIZE;
		// Write simple value to IOW56
	case IOWKIT_PRODUCT_ID_IOW56:
		memset(&rep56, 0xff, sizeof(IOWKIT56_IO_REPORT));
		rep56.ReportID = 0;
		rep56.Bytes[6] = (UCHAR) mPortMask;
		return IowKitWrite(mIoWarriorHandle, IOW_PIPE_IO_PINS, (char*)&rep56, IOWKIT56_IO_REPORT_SIZE) == IOWKIT56_IO_REPORT_SIZE;
	default:
		return false;
	}
}

std::list<uint8_t> IoWarrior::getPortNumbers(std::string portname)
{
	std::list<uint8_t> portNumbers;

	// see if portname is a number. If yes, append it.
	std::istringstream ss(portname);
	int number;
	if((ss >> number) && number < 32)
	{
		ROS_INFO("IoWarrior::getPortNumbers(): port \"%s\" has number %d", portname.c_str(), number);
		portNumbers.push_back(number);
	}

	// convert to lower case
	std::transform(portname.begin(), portname.end(), portname.begin(), static_cast<int (*)(int)>(std::tolower));

	std::map<uint8_t, std::string>::iterator it;
	for(it=mPortNames.begin(); it != mPortNames.end(); it++)
	{
		if((*it).second.compare(portname) == 0)
		{
			ROS_INFO("IoWarrior::getPortNumbers(): port \"%s\" has number %d", portname.c_str(), (*it).first);
			portNumbers.push_back((*it).first);
		}
	}

	if(std::string("all").compare(portname) == 0)
		for(int i=0; i<32; i++)
			portNumbers.push_back(i);

	return portNumbers;
}

std::string IoWarrior::getPortName(const uint8_t portNumber)
{
	std::map<uint8_t,std::string>::iterator it;

	it=mPortNames.find(portNumber);

	if(it != mPortNames.end())
	{
		return mPortNames[portNumber];
	}
	else
	{
		ROS_WARN("IoWarrior::getPortName(): port number %d is not defined!", portNumber);
		return "UNDEFINED";
	}

}

std::string IoWarrior::printBits(uint32_t n)
{
	unsigned int i;
	i = 1<<(sizeof(n) * 8 - 1);
	std::string result;

	while (i > 0)
	{
		if (n & i)
			result.append("1");//printf("1");
		else
			result.append("0");//printf("0");
		i >>= 1;
	}

	return result;
}

// fast exponentiation algorithm
int IoWarrior::ipow(int base, int exp)
{
	int result = 1;
	while (exp)
	{
		if (exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
	}

	return result;
}

void IoWarrior::changePortMask(uint8_t port, bool enabled)
{
//   ROS_INFO("IoWarrior::changePortMask():: port %d, mPortMask before: %s, set to %d", port, printBits(mPortMask).c_str(), enabled);

	if(enabled)
	{
		// enabling means setting that bit to 0. For this, we want to AND the mPortMask
		// with a mask that is always 1, except for the specified port/bit. To get such
		// a mask, we create the inverse and then invert :)

		// e.g. for port 3 its 0000000100
		uint32_t mask = ipow(2, port);

		// invert to make it 1111111011
		mask = ~mask;

//     ROS_INFO("IoWarrior::changePortMask(): port %d, power-on: ANDing with mask:\t%s, intvalue %d", port, printBits(mask).c_str(), mask);

		mPortMask = mPortMask & mask;
	}
	else
	{
		// disabling means setting that bit to 1. For this, we want to OR the mPortMask
		// with a mask that is always 0, except for the specified port/bit.

		// e.g. for port 3 its 0000000100
		uint32_t mask = ipow(2, port);
//     ROS_INFO("IoWarrior::changePortMask(): port %d, power-off: ORing with mask:\t%s, intvalue %d", port, printBits(mask).c_str(), mask);

		mPortMask = mPortMask | mask;
	}

//   ROS_INFO("IoWarrior::changePortMask(): port %d, portMask after:\t%s", port, printBits(mPortMask).c_str());
}

bool IoWarrior::retrievePortStatesFromIoWarrior()
{

	uint32_t newMask = 128;
	if(IowKitReadImmediate(mIoWarriorHandle, (DWORD*)&newMask))
	{
//     ROS_INFO("immediate(): YES newMask:\t%s", printBits(newMask).c_str());
		mPortMask = newMask;
		return true;
	}
	else
	{
		ROS_ERROR("IoWarrior::retrievePortStatesFromIoWarrior(): IoWKitReadImmediate() failed, so the current port status might be wrong.");
		return false;
	}
	return true;
}
