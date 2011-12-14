// *******************************************************************************
// (c) Bernd Roessler (2002)
//     inspired by Markus Ferch
// *******************************************************************************

/// Header for SerialInterface class
/**
*	@file SerialInterface.h
*/


/**
* @mainpage Serial-Library Documentation
*
*	@section auth Author
* <b>Bernd Roessler</b> <br>
* TAMS <br>
* Faculty of Computer Science  <br>
* University of Hamburg, Germany  <br>
* mail: roessler@informatik.uni-hamburg.de
*	@section date Date
*	(c) 2003
*	@section intro Introduction
*	This library provides a C++ class for easy communication over a serial interface on Linux/Solaris.
*	@section install Installation
*	<i>./configure</i> followed by <i>make install</i>
*/


#ifndef __SERIALINTERFACE_H__
#define __SERIALINTERFACE_H__

#include <iostream>
#include <string>
#include <termios.h>
#include <assert.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
//#include <iostream>

#ifdef __sparc__
#include <poll.h>
#endif

#ifndef TEMP_FAILURE_RETRY
#define TEMP_FAILURE_RETRY(expression)           \
  (__extension__                                 \
    ({ long int __result;                        \
       do __result = (long int) (expression);    \
       while (__result == -1L && errno == EINTR);\
       __result; }))
#endif

static const char* SerialInterfaceException_info[] =
{
	"Opening of serial port failed.",
	"Device is not a serial port.",
	"Determining original termios settings failed.",
	"Setting baud-rate for input failed.",
	"Setting baud-rate for output failed.",
	"Setting parameters for termios failed.",
	"Flushing failed.",
	"Undefined exception.",
	"Reading failed (check errno).",
	"Writing failed (check errno).",
	"Serial port is not open.",
	"Serial port is allready open."
};

/// Exception used by SerialInterface
/**
*	\class SerialInterfaceException
*/
class SerialInterfaceException
{

public:
	/**
	* Defines the different errors.
	*/
	enum error
	{
		EXCP_OPEN_FAILED, /**< Opening of serial port failed.*/
		EXCP_NOT_TTY, /**< Device is not a serial port.*/
		EXCP_TERMIOS_GET_FAILED, /**< Determining original termios settings failed.*/
		EXCP_BAUD_IN_FAILED, /**< Setting baud-rate for input failed.*/
		EXCP_BAUD_OUT_FAILED, /**< Setting baud-rate for output failed.*/
		EXCP_TERMIOS_SET_FAILED, /**< Setting parameters for termios failed.*/
		EXCP_FLUSHING_FAILED, /**< Flushing failed.*/
		EXCP_UNDEF, /**< Undefined exception.*/
		EXCP_READ, /**< Reading failed (check errno).*/
		EXCP_WRITE, /**< Writing failed (check errno).*/
		EXCP_NOTOPEN, /**< Serial port is not open.*/
		EXCP_ALLREADY_OPEN /**< Serial port is allready open.*/
	} err; /**< Holds the error.*/

private:
	std::string msg;

public:
	SerialInterfaceException (const error e)
	{
		msg = SerialInterfaceException_info[e];
		err = e;
	};

	/**
	* Prints out the error message.
	*/
	const void show ()
	{
		std::cout << msg << std::endl;
	};

	/**
	* Operator for error message output.
	*/
	friend std::ostream& operator<<(std::ostream& os,const SerialInterfaceException& e)
	{
		os << e.msg << std::endl;
		return os;
	}

	/**
	* Operator for comparing the error.
	*/
	bool operator==(error e)
	{
		return (err == e);
	}
};

/// Class for communication with a serial interface.
/**
*	\class SerialInterface
*	Class for communication with a serial interface.
*	This is a wrapper for termios.
*/
#define MAX_BUFFER_SIZE 100
class SerialInterface
{

private:
	struct termios Original_termios;
	struct termios Actual_termios;
	char* buffer;
	int fd;
	std::string devName;
	bool isOpen;
	/*
	* Applies all changes made to the parameters of the termios structure.
	*/
	void ApplyChanges() throw (SerialInterfaceException);

public:
	/**
	*	@param devName defines the device name (e.g. /dev/ttyS0).
	*	@param maxBufferSize defines the maximum possible size in bytes of transferred data.
	*/
	SerialInterface (const std::string devName, unsigned int maxBufferSize = 100);

	/**
	*	Opens the interface.
	* @param flag parameters for opening the interface.
	* For different parameters than the default check the manpage of <i>open</i>.
	* @throw SerialInterfaceException
	* @see Close()
	*/
	void Open (int flag = O_RDWR | O_NOCTTY | O_NDELAY) throw (SerialInterfaceException);
	//void Open (int flag = O_RDWR | O_SYNC) throw (SerialInterfaceException);

	virtual ~SerialInterface (void);

	/**
	*	Closes the interface.
	* @see Open(int flag)
	*/
	void Close();

	/**
	*	Reads the available set of data from the interface until <i>count</i> bytes are read or timout is reached.
	*	@throw SerialInterfaceException
	*	@param count number of bytes to be read.
	*	@param timeout defines the global timeout for a read operation in seconds.
	*	@returns A string with the data from the interface.
	* @see Write(const string& str)
	*/
	std::string Read(int timeout = 0, unsigned int count = MAX_BUFFER_SIZE) throw (SerialInterfaceException);

	/**
	* Reads the available set of data from the interface ("C-Style").
	*	@throw SerialInterfaceException
	* @param buf buffer to store received data.
	* @param count number of bytes to be read.
	* @param timeout defines the global timeout for a read operation.
	* @returns The number of bytes read (on success), on error -1.
	* @see Write(const void *buf, size_t count)
	*/
	ssize_t Read (void *buf, size_t count, int timeout = 0) throw (SerialInterfaceException);

	/**
	*	Writes a set of data to the interface.
	*	@throw SerialInterfaceException
	*	@param str the string containing the data to be transferred.
	* @see Read(unsigned int count, int blocking)
	*/
	void Write(const std::string& str) throw(SerialInterfaceException);

	/**
	*	Writes a set of data to the interface ("C-Style").
	*	@throw SerialInterfaceException
	*	@param buf buffer with data to be transferred.
	* @param count number of bytes to be transferred.
	* @returns The number of bytes written (on success), on error -1.
	* @see Read(void *buf, size_t count, int blocking)
	*/
	ssize_t Write (const void *buf, size_t count) throw (SerialInterfaceException);

	/**
	* Sets special bits for the c_cflag (control) attribute of the termios structure.
	* @param CSIZE Bit mask for data bits
	  	* @param CS5 5 data bits
	  	* @param CS6 6 data bits
	  	*	@param CS7 7 data bits
	  	*	@param CS8 8 data bits
	  	*	@param CSTOPB 2 stop bits (1 otherwise)
	  	*	@param CREAD Enable receiver
	  	*	@param PARENB Enable parity bit
	  	*	@param PARODD Use odd parity instead of even
	  	*	@param HUPCL Hangup (drop DTR) on last close
	  	*	@param CLOCAL Local line - do not change "owner" of port
	   	* @see Unsetc_cflag(tcflag_t c_cN)
	   	*/
	void Setc_cflag (tcflag_t c_cS = CREAD | CS8 | PARENB | HUPCL | CLOCAL) throw (SerialInterfaceException);

	/**
	* Unsets special bits for the c_cflags (control) attribute of the termios structure.
	* @see Setc_cflag(tcflag_t c_cS)
	*/
	//void Unsetc_cflag (tcflag_t c_cN = PARODD | CSTOPB | CRTSCTS | CSIZE);
	void Unsetc_cflag (tcflag_t c_cN = PARODD | CSTOPB | CRTSCTS) throw (SerialInterfaceException);

	/**
	* Sets special bits for the c_lflag (local) attribute of the termios structure.
	* @param ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
	  	* @param ICANON Enable canonical input (else raw)
	  	* @param ECHO Enable echoing of input characters
	  	* @param ECHOE Echo erase character as BS SP BS (only in combination with ICANON)
	  	* @param ECHOK Echo NL after kill character (only in combination with ICANON)
	  	* @param ECHONL Echo NL (only in combination with ICANON)
	  	* @param NOFLSH Disable flushing of input buffers after interrupt or quit characters
	  	* @param IEXTEN Enable extended functions
	  	* @param TOSTOP Send SIGTTOU for background output
	   	* @see Unsetc_lflag(tcflag_t c_lN)
	*/
	void Setc_lflag (tcflag_t c_lS = 0x0) throw (SerialInterfaceException);

	/**
	* Unsets special bits for the c_lflags (local) attribute of the termios structure.
	* @see Setc_lflag(tcflag_t c_lS)
	*/
	void Unsetc_lflag (tcflag_t c_lN = ECHO | ECHONL | ICANON | ISIG) throw (SerialInterfaceException);

	/**
	* Sets special bits for the c_iflag (input) attribute of the termios structure.
	* @param INPCK Enable parity check
	  	* @param IGNPAR Ignore parity errors
	  	* @param PARMRK Mark parity errors
	  	* @param ISTRIP Strip parity bits
	  	* @param IXON Enable software flow control (outgoing)
	  	* @param IXOFF Enable software flow control (incoming)
	  	* @param IXANY Allow any character to start flow again
	  	* @param IGNBRK Ignore break condition
	  	* @param BRKINT Send a SIGINT when a break condition is detected
	  	* @param INLCR Map NL to CR
	  	* @param IGNCR Ignore CR
	  	* @param ICRNL Map CR to NL
	  	* @param IMAXBEL Echo BEL on input line too long
	   	* @see Unsetc_iflag(tcflag_t c_iN)
	*/
	void Setc_iflag (tcflag_t c_iS = IGNBRK | INPCK | PARMRK) throw (SerialInterfaceException);

	/**
	* Unsets special bits for the c_iflags (input) attribute of the termios structure.
	* @see Setc_iflag(tcflag_t c_iS)
	*/
	void Unsetc_iflag (tcflag_t c_iN = ISTRIP | BRKINT | IGNCR | ICRNL | IGNPAR | INLCR | IXOFF | IXON | IXANY | IMAXBEL) throw (SerialInterfaceException);

	/**
	* Sets special bits for the c_oflag (output) attribute of the termios structure.
	* @param OPOST Postprocess output (not set = raw output)
	* @param ONLCR Map NL to CR-NL
	* @param OCRNL Map CR to NL
	* @param NOCR No CR output at column 0
	* @param ONLRET NL performs CR function
	* @param OFILL Use fill characters for delay
	* @param NLDLY Mask for delay time needed between lines
	* @param NL0 No delay for NLs
	* @param NL1 Delay further output after newline for 100 milliseconds
	* @param CRDLY Mask for delay time needed to return carriage to left column
	* @param CR0 No delay for CRs
	* @param CR1 Delay after CRs depending on current column position
	* @param CR2 Delay 100 milliseconds after sending CRs
	* @param CR3 Delay 150 milliseconds after sending CRs
	* @param TABDLY Mask for delay time needed after TABs
	* @param TAB0 No delay for TABs
	* @param TAB1 Delay after TABs depending on current column position
	* @param TAB2 Delay 100 milliseconds after sending TABs
	* @param TAB3 Expand TAB characters to spaces
	* @param VTDLY Mask for delay time needed after VTs
	* @param VT0 No delay for VTs
	* @param VT1 Delay 2 seconds after sending VTs
	* @param FFDLY Mask for delay time needed after FFs
	* @param FF0 No delay for FFs
	* @param FF1 Delay 2 seconds after sending FFs
	   	* @see Unsetc_oflag(tcflag_t c_oN)
	*/
	void Setc_oflag (tcflag_t c_oS = CR0) throw (SerialInterfaceException);

	/**
	* Unsets special bits for the c_oflag (output) attribute of the termios structure.
	* @see Setc_oflag(tcflag_t c_oS)
	*/
	void Unsetc_oflag (tcflag_t c_oN = OPOST | OLCUC | ONLCR | OCRNL| OFILL | ONOCR | ONLRET) throw (SerialInterfaceException);

	/**
	* Sets the control characters c_cc.
	* Do not change anything here unless you know what you are doing. For timeout settings use the SetTimeout() method.
	*/
	void Setc_cc (cc_t c_cc[NCCS]) throw (SerialInterfaceException);

	/**
	* Sets the baudrate for input and output.
	* @param B0 0 baud (drop DTR)
	* @param B50 50 baud
	* @param B75 75 baud
	* @param B110 110 baud
	* @param B134 134.5 baud
	* @param B150 150 baud
	* @param B200 200 baud
	* @param B300 300 baud
	* @param B600 600 baud
	* @param B1200 1200 baud
	* @param B1800 1800 baud
	* @param B2400 2400 baud
	* @param B4800 4800 baud
	* @param B9600 9600 baud
	* @param B19200 19200 baud
	* @param B38400 38400 baud
	* @param B57600 57,600 baud
	* @param B76800 76,800 baud
	* @param B115200 115,200 baud
	* @param B230400
	* @see SetInputBaudRate
	* @see SetOutputBaudRate
	*/
	void SetBaudRate (speed_t BAUT_RATE) throw (SerialInterfaceException);

	/**
	* Sets the baudrate for input.
	* For available baudrates see SetBaudRate().
	* @see SetBaudRate
	*/
	void SetInputBaudRate (speed_t BAUT_RATE_I) throw (SerialInterfaceException);

	/**
	* Sets the baudrate for output.
	* For available baudrates see SetBaudRate().
	* @see SetBaudRate
	*/
	void SetOutputBaudRate (speed_t BAUT_RATE_O) throw (SerialInterfaceException);

	/*
	* Sets the timeout in 1/10 seconds for reading operations.
	* Note that in blocking mode the Read-methods could perhaps wait upto timeout*(desired bytes)/10 seconds.
	* Therefore for blocking mode a timeout of 0 is recommended.
	* For canonical input mode the timeout is ignored.
	* @param t timeout in 1/10 seconds
	*/
	//void SetTimeout(unsigned int t) throw (SerialInterfaceException);

	/**
	 * Waits until all output written has been transmitted.
	 * @returns 0 on success, -1 on failure.
	 */
	int Drain (void);

	/**
	 * Discards data written, but not transmitted,
	 * or data received but not read, depending on the value of queue.
	 * @param TCIFLUSH flushes data received but not read.
	 * @param TCOFLUSH flushes data written but not transmitted.
	 * @param TCIOFLUSH flushes both data received but not read, and data
	 * written but not transmitted.
	 */
	int Flush (int queue = TCIOFLUSH);

	int Select (long sec, long usec);

	int Select (void);


};


#endif
