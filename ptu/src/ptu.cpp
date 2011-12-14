/***************************************************************************
                          ptu.cpp  -  description
                             -------------------
    begin                : Mon Jan 13 2003
    copyright            : (C) 2003 by Bernd Roessler
    email                : roessler@informatik.uni-hamburg.de
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// #include "tools.h" ben adler: pasted into ptu.h
#include "ptu.h"
#include "timer.h"
#include "serialinterface.h"

PTU::PTU(const std::string devName)
{
	com = new SerialInterface(devName);
}

PTU::~PTU()
{
	com->Close();
	delete (com);
}

void PTU::init() throw (PTUException)
{
	try
	{
		std::cout << "PTU Programming Interface V 0.9.1" <<std::endl;
		std::cout << "(c)2003 Bernd Roessler\n"<<std::endl;
		std::cout << "(c)2004 Tim Baier & Daniel Westhoff\n"<<std::endl;
		std::cout << "Initializing "<<std::flush;

		com->Open();
		com->Unsetc_cflag(PARENB);
		com->Unsetc_iflag(INPCK|PARMRK);
		com->Flush();

		//Set the baud rate
		com->SetInputBaudRate(B38400);
		bool finish = false;
		for (int k=0; k<=2 && !finish; k++)
		{
			com->SetOutputBaudRate(B9600+k);
			try
			{
				std::cout <<"."<<std::flush;
				com->Write("@(38400,0,F) ");
				read(0.2);
				finish = true;
			}
			catch (PTUException e)
			{
				if (k == 2)
				{
					throw PTUException(PTUException::EXCP_READ);
				}
			}
		}

		com->SetOutputBaudRate(B38400);
		com->Flush();
		for (int i=0; i<3; i++)
		{
			sleep(1);   //the PTU needs its time
			std::cout << "."<<std::flush;
		}
		std::cout << std::endl;

		send("ED"); //Disable host command echoing
		send("FT"); //Enable terse ASCII feedback:
		send("LE"); //Enable position limits

		std::string vers = send("V");
		std::cout << std::endl<< "Firmware: "+std::string(vers,2,vers.size())<<std::endl;
		std::cout << "Initialization finished successfully." <<std::endl;


		//Query pan/tilt axis resolution:
		std::string resStr;
		resStr = send("PR ");
		panResolution = 3600./ANSWER_TO_DOUBLE(resStr);
		resStr = send("TR ");
		tiltResolution = 3600./ANSWER_TO_DOUBLE(resStr);

		//Query pan/tilt max/min position:
		resStr = send("PN");
		minPP = ANSWER_TO_INT(resStr);
		resStr = send("PX");
		maxPP = ANSWER_TO_INT(resStr);
		resStr = send("TN");
		minTP = ANSWER_TO_INT(resStr);
		resStr = send("TX");
		maxTP = ANSWER_TO_INT(resStr);

		setSpeed(0.0,0.0);
		setIndependentControl();
		setSpeed(100.,100.);

	}
	catch (SerialInterfaceException e)
	{
		switch (e.err)
		{
		case (SerialInterfaceException::EXCP_ALLREADY_OPEN):
		{
			throw PTUException (PTUException::EXCP_ALLREADY_INITIALIZED);
			break;
		}
		default:
		{
			throw PTUException (PTUException::EXCP_INITIALIZATION_FAILED);
			break;
		}
		}
	}
}

HoldPowerMode PTU::getHoldPowerMode (const PtuAxis axis)
{
	std::string retStr;
	switch (axis)
	{
	case PTU_PAN:
	{
		retStr = send("PH");
	}
	default:
	{
		retStr = send("TH");
	}
	}
	if (retStr.find("OFF") == 9)
		return HPM_OFF;
	else if (retStr.find("LOW") == 9)
		return HPM_LOW;
	return HPM_REGULAR;
}

MovePowerMode PTU::getMovePowerMode (const PtuAxis axis)
{
	std::string retStr;
	switch (axis)
	{
	case PTU_PAN:
	{
		retStr = send("PM");
	}
	default:
	{
		retStr = send("TM");
	}
	}
	if (retStr.find("HIGH") == 9)
		return MPM_HIGH;
	else if (retStr.find("LOW") == 9)
		return MPM_LOW;
	return MPM_REGULAR;
}

//**********************************************************************
// private:
//**********************************************************************
std::string PTU::send (std::string s) throw (PTUException)
{
	try
	{

		com->Write(s+" ");
		TRACE2("Sent: ",s);
		if(s == "A")
			return errorCheck(0.0);
		else
			return errorCheck();
	}
	catch (SerialInterfaceException e)
	{
		if (e.err == SerialInterfaceException::EXCP_NOTOPEN)
			throw PTUException (PTUException::EXCP_NOT_INITIALIZED);
	}
}

std::string PTU::read(double timeout) throw (PTUException)
{
	/*struct timeval tstart;
	struct timezone tzone;
	gettimeofday (&tstart,&tzone);*/
	std::string s = "";
	TIMER::SetStartTime();

	while(s[s.size()-1] != '\n')
	{
		try
		{
			s += com->Read();
		}
		catch (SerialInterfaceException e)
		{
			switch(e.err)
			{
			default:
				break;
			}
		}
		if (TIMER::TimeElapsed() >= timeout && timeout != 0.0)
			throw (PTUException(PTUException::EXCP_READ));
	}
	return s;
}

std::string PTU::errorCheck() throw (PTUException)
{
	std::string s = read();
	TRACE2("Received: ",s);
	if (s[0] == '!')
	{
		throw (PTUException(s.substr(2)));
	}
	return s;
}

std::string PTU::errorCheck(double timeout) throw (PTUException)
{
	std::string s = read(timeout);
	TRACE2("Received: ",s);
	if (s[0] == '!')
	{
		throw (PTUException(s.substr(2)));
	}
	return s;
}

void PTU::command(const PtuAxis axis, const double param, const PtuUnit unit, const std::string command) throw (PTUException)
{
	int pos;
	std::string c;

	switch (axis)
	{
	case PTU_PAN:
	{
		c = "P"+command;
		break;
	}
	default:
	{
		c = "T"+command;
		break;
	}
	}
	switch (unit)
	{
	case PTU_DEGREE:
	{
		if (axis == PTU_PAN)
			pos = PAN_DEG2POS(param);
		else
			pos = TILT_DEG2POS(param);
		break;
	}
	case PTU_TICKS:
	{
		pos = (int) param;
	}
	default:
		break;
	}
	send(c+lexical_cast<std::string,int>(pos));
}

double PTU::query(const PtuAxis axis, const PtuUnit unit, const std::string command) throw (PTUException)
{
	std::string c;

	switch (axis)
	{
	case PTU_PAN:
	{
		c = "P"+command;
		break;
	}
	default:
	{
		c = "T"+command;
		break;
	}
	}
	std::string resStr = send(c);
	int pos = ANSWER_TO_INT(resStr);
	switch (unit)
	{
	case PTU_DEGREE:
	{
		if (axis == PTU_PAN)
			return PAN_POS2DEG(pos);
		else
			return TILT_POS2DEG(pos);
		break;
	}
	case PTU_TICKS:
	{
		return (double) pos;
	}
	default:
		break;
	}
}

double PTU::limitPos(const PtuAxis axis, const PtuUnit unit, const MAXMIN minmax)
{
	double pos = 0.;
	if(minmax == MAX)
	{
		switch (unit)
		{
		case PTU_DEGREE:
		{
			if (axis == PTU_PAN)
				pos = PAN_POS2DEG(maxPP);
			else
				pos = TILT_POS2DEG(maxTP);
			break;
		}
		case PTU_TICKS:
		{
			if (axis == PTU_PAN)
				pos = (double) maxPP;
			else
				pos = (double) maxTP;
		}
		default:
			break;
		}
	}
	else
	{
		switch (unit)
		{
		case PTU_DEGREE:
		{
			if (axis == PTU_PAN)
				pos = PAN_POS2DEG(minPP);
			else
				pos = TILT_POS2DEG(minTP);
			break;
		}
		case PTU_TICKS:
		{
			if (axis == PTU_PAN)
				pos = (double) minPP;
			else
				pos = (double) minTP;
		}
		default:
			break;
		}
	}
	return pos;
}

void PTU::hpm(const PtuAxis axis, const HoldPowerMode mode)
{
	bool pan;
	(axis == PTU_PAN) ? pan =true : pan =false;
	switch (mode)
	{
	case HPM_OFF:
	{
		pan ? send("PHO") : send("THO");
		break;
	}
	case HPM_LOW:
	{
		pan ? send("PHL") : send("THL");
		break;
	}
	default:
	{
		pan ? send("PHR") : send("THR");
		break;
	}
	}
}

void PTU::mpm(const PtuAxis axis, const MovePowerMode mode)
{
	bool pan;
	(axis == PTU_PAN) ? pan =true : pan =false;
	switch (mode)
	{
	case MPM_LOW:
	{
		pan ? send("PML") : send("TML");
		break;
	}
	case MPM_HIGH:
	{
		pan ? send("PMH") : send("TMH");
		break;
	}
	default:
	{
		pan ? send("PMR") : send("TMR");
		break;
	}
	}
}

