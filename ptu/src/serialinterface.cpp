#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sched.h>
#include <timer.h>
#include "serialinterface.h"

/*
	SerialInterface
	Stellt eine Klasse zur Kommunikation ueber RS232-Schnittstellen bereit.
	Author: Bernd Roessler & Markus Ferch
*/

SerialInterface::SerialInterface(const std::string name, unsigned int maxBufferSize)
{
	isOpen = false;
	devName = name;
	buffer = new char[maxBufferSize];
}

void SerialInterface::Open (int flag) throw (SerialInterfaceException)
{
	if (isOpen)
		throw SerialInterfaceException(SerialInterfaceException::EXCP_ALLREADY_OPEN);

#ifdef __sparc__
	/*
	 * Sparc and serial lines a chapter of its own
	 */
	sched_yield ();
	sched_yield ();
	close (fd);
	sched_yield ();
	sched_yield ();
#endif

	fd = open (devName.c_str(), flag);

	if (fd == -1)
	{
		throw SerialInterfaceException (SerialInterfaceException::EXCP_OPEN_FAILED);
	}

	if (!isatty (fd))
	{
		close (fd);
		throw SerialInterfaceException (SerialInterfaceException::EXCP_NOT_TTY);
	}

	isOpen = true;

	// ** Getting the original termios settings **
	if (tcgetattr (fd, &Original_termios) < 0)
	{
		close (fd);
		isOpen = false;
		throw SerialInterfaceException (SerialInterfaceException::EXCP_TERMIOS_GET_FAILED);
	}

	Actual_termios = Original_termios;

	SetBaudRate (B9600);
	Setc_cflag();
	Unsetc_cflag();
	Setc_lflag();
	Unsetc_lflag();
	Setc_iflag();
	Unsetc_iflag();
	Setc_oflag();
	Unsetc_oflag();
	Actual_termios.c_cc[VMIN] = 0;  //1
	Actual_termios.c_cc[VTIME] = 1; //0

	// ** Make the structure active **
	if (tcsetattr (fd, TCSAFLUSH, &Actual_termios) < 0)
	{
		close (fd);
		isOpen = false;
		throw SerialInterfaceException
		(SerialInterfaceException::EXCP_TERMIOS_SET_FAILED);
	}

	// ** flushing the queues **
	if (tcflush (fd, TCIOFLUSH) < 0)
	{
		close (fd);
		isOpen = false;
		throw SerialInterfaceException (SerialInterfaceException::EXCP_FLUSHING_FAILED);
	}

#if 0
	printf ("c_iflag: %i %i\n", Actual_termios.c_iflag, Original_termios.c_iflag);
	printf ("c_oflag: %i %i\n", Actual_termios.c_oflag, Original_termios.c_oflag);
	printf ("c_cflag: %i %i\n", Actual_termios.c_cflag, Original_termios.c_cflag);
	printf ("c_lflag: %i %i\n", Actual_termios.c_lflag, Original_termios.c_lflag);

	printf ("c_cc[VEOL]  : %i %i\n", Actual_termios.c_cc[VEOL], Original_termios.c_cc[VEOL]);
	printf ("c_cc[VERASE]: %i %i\n", Actual_termios.c_cc[VERASE], Original_termios.c_cc[VERASE]);
	printf ("c_cc[VKILL] : %i %i\n", Actual_termios.c_cc[VKILL], Original_termios.c_cc[VKILL]);
	printf ("c_cc[VMIN]  : %i %i\n", Actual_termios.c_cc[VMIN], Original_termios.c_cc[VMIN]);
	printf ("c_cc[VTIME] : %i %i\n", Actual_termios.c_cc[VTIME], Original_termios.c_cc[VTIME]);
#endif
}

void SerialInterface::Close()
{
	if (isOpen)
	{
		tcsetattr (fd, TCSANOW, &Original_termios);
		close(fd);
		isOpen = false;
	}
}

SerialInterface::~SerialInterface ()
{
	Close();
	delete[] buffer;
}

ssize_t SerialInterface::Read (void *buf, size_t N, int timeout) throw (SerialInterfaceException)
{
	if (!isOpen)
		throw SerialInterfaceException(SerialInterfaceException::EXCP_NOTOPEN);

	fcntl(fd, F_SETFL, FNDELAY);

	int r = 0;
	unsigned char *Buf = reinterpret_cast < unsigned char *>(buf);

	if (N == 0)
		return 0;

	int anz = 0;
	TIMER::SetStartTime();
	do
	{
		r = read (fd, Buf, N);
		if (r < 0)
		{
			if (errno == EINTR || errno == EAGAIN)
				r=0;
			else
				throw SerialInterfaceException (SerialInterfaceException::EXCP_READ);
			//return r;
		}
		anz += r;
		N -= r;
		Buf += r;
	}
	while (N > 0 && TIMER::TimeElapsed()<timeout);

	return anz;
}

/*
Dies ist generell ein nicht blockierendes Lesen.
Es wird aber solange versucht zu lesen bis der Timout überschritten wurde
oder N Zeichen gelesen worden sind.
Die Einheit des Timeout sind momentan Sekunden.
*/
std::string SerialInterface::Read(int timeout, unsigned int N) throw (SerialInterfaceException)
{
	if (!isOpen)
		throw SerialInterfaceException(SerialInterfaceException::EXCP_NOTOPEN);

	fcntl(fd, F_SETFL, FNDELAY);

	int r = 0;

	//unsigned char *Buf = reinterpret_cast < unsigned char *>(buffer);
	if (N <= 0)
		return std::string("");

	int anz = 0;
	char* Buf = buffer;
	TIMER::SetStartTime();
	do
	{
		r = read (fd, Buf, N);
		if (r < 0)
		{
			if (errno == EINTR || errno == EAGAIN)
				r=0;
			else
				throw SerialInterfaceException (SerialInterfaceException::EXCP_READ);
		}
		anz += r;
		N -= r;
		Buf += r;
	}
	while (N > 0 && TIMER::TimeElapsed()<timeout);
	return std::string(buffer,anz);
}

ssize_t SerialInterface::Write (const void *buf, size_t count) throw (SerialInterfaceException)
{
	if (!isOpen)
		throw SerialInterfaceException(SerialInterfaceException::EXCP_NOTOPEN);
	return write (fd, buf, count);
}

void SerialInterface::Write(const std::string& str) throw(SerialInterfaceException)
{
	if (!isOpen)
		throw SerialInterfaceException(SerialInterfaceException::EXCP_NOTOPEN);
	if (write(fd, str.c_str(), str.size()) < 0)
		throw SerialInterfaceException::EXCP_WRITE;
}

int SerialInterface::Drain (void)
{
	return tcdrain (fd);
}

int SerialInterface::Flush (int queue)
{
	return tcflush (fd, queue);
}

void SerialInterface::SetBaudRate (speed_t BAUT_RATE) throw (SerialInterfaceException)
{
	SetInputBaudRate (BAUT_RATE);
	SetOutputBaudRate (BAUT_RATE);
}

void SerialInterface::SetInputBaudRate (speed_t BAUT_RATE_I) throw (SerialInterfaceException)
{
	cfsetispeed (&Actual_termios, BAUT_RATE_I);
	ApplyChanges();
}

void SerialInterface::SetOutputBaudRate (speed_t BAUT_RATE_O) throw (SerialInterfaceException)
{
	cfsetospeed (&Actual_termios, BAUT_RATE_O);
	ApplyChanges();
}

void SerialInterface::Setc_cflag (tcflag_t c_set) throw (SerialInterfaceException)
{
	Actual_termios.c_cflag |= c_set;
	ApplyChanges();
}

void SerialInterface::Setc_oflag (tcflag_t c_set) throw (SerialInterfaceException)
{
	Actual_termios.c_oflag |= c_set;
	ApplyChanges();
}

void SerialInterface::Setc_iflag (tcflag_t c_set) throw (SerialInterfaceException)
{
	Actual_termios.c_iflag |= c_set;
	ApplyChanges();
}

void SerialInterface::Setc_lflag (tcflag_t c_set) throw (SerialInterfaceException)
{
	Actual_termios.c_lflag |= c_set;
	ApplyChanges();
}

void SerialInterface::Unsetc_cflag (tcflag_t c_unset) throw (SerialInterfaceException)
{
	Actual_termios.c_cflag &= ~c_unset;
	ApplyChanges();
}

void SerialInterface::Unsetc_oflag (tcflag_t c_unset) throw (SerialInterfaceException)
{
	Actual_termios.c_oflag &= ~c_unset;
	ApplyChanges();
}

void SerialInterface::Unsetc_iflag (tcflag_t c_unset) throw (SerialInterfaceException)
{
	Actual_termios.c_iflag &= ~c_unset;
	ApplyChanges();
}

void SerialInterface::Unsetc_lflag (tcflag_t c_unset) throw (SerialInterfaceException)
{
	Actual_termios.c_lflag &= ~c_unset;
	ApplyChanges();
}

void SerialInterface::Setc_cc (cc_t c_cc[NCCS]) throw (SerialInterfaceException)
{
	memcpy (Actual_termios.c_cc, c_cc, sizeof (cc_t) * NCCS);
	ApplyChanges();
}

/*
void SerialInterface::SetTimeout(unsigned int t) throw (SerialInterfaceException)
{
	timer = t;
}*/

void SerialInterface::ApplyChanges() throw (SerialInterfaceException)
{
	if (tcsetattr (fd, TCSAFLUSH, &Actual_termios) < 0)
	{
		throw SerialInterfaceException
		(SerialInterfaceException::EXCP_TERMIOS_SET_FAILED);
	}
}

int SerialInterface::Select (long sec, long usec)
{
	int rv = 0;

	if (sec || usec)
	{

#ifdef __sparc__
		// #warning POLL
		sec *= 1000;
		usec /= 1000;

		struct pollfd fds;
		fds.fd = fd;
		fds.events = POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI;

		TEMP_FAILURE_RETRY ((rv = poll (&fds, 1, usec + sec)));

#else
		// #warning SELECT
		struct timeval timeout;
		fd_set FDs;

		timeout.tv_sec = sec;
		timeout.tv_usec = usec;
		FD_ZERO (&FDs);
		FD_SET (fd, &FDs);
		TEMP_FAILURE_RETRY ((rv =
		                         select (FD_SETSIZE, &FDs, NULL, NULL,
		                                 &timeout)));
#endif
	}

	return rv;
}

int SerialInterface::Select (void)
{
	int rv = 0;

#ifdef __sparc__
	// #warning POLL
	struct pollfd fds;
	fds.fd = fd;
	fds.events = POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI;

	TEMP_FAILURE_RETRY ((rv = poll (&fds, 1, -1)));

#else
	// #warning SELECT
	fd_set FDs;

	FD_ZERO (&FDs);
	FD_SET (fd, &FDs);
	TEMP_FAILURE_RETRY ((rv =
	                         select (FD_SETSIZE, &FDs, NULL, NULL, NULL)));
#endif

	return rv;
}
