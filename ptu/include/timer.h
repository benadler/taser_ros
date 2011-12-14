/***************************************************************************
                                timer.h
                             -------------------
    begin                : Fri Jan 10 2003
    author               : Daniel Westhoff
    email                : westhoff@informatik.uni-hamburg.de
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#ifndef TIMER_H
#define TIMER_H

#include "sys/time.h"

///
/// \brief Methods to measure ellapsed time.
///
/// In this namespace methods are defined to measure the time ellapsed
/// between the methods called.
///
/// In diesem Namensbereich werden Methoden definiert, die es erlauben
/// die Zeit zu messen die zwischen den Aufrufen der Methode vergangen ist.
///

namespace TIMER
{

static struct timeval _tstart;
static struct timezone tz;

///
/// \brief Set the timers starting time.
///
/// This method takes the time when the method is called.
/// TIMER::TimeElapsed returns the elapsed time since the
/// call of this method. To reset the time call this method
/// again.
///
/// Diese Methode nimmt die aktuelle Zeit wenn die Methode
/// aufgerufen wird. TIMER::TimeElapsed liefert die seit dem
/// Aufruf dieser Methode vergangene Zeit. Um die Zeit
/// zurückzusetzen rufen sie diese Methode erneut auf.
///
/// \sa TIMER::TimeElapsed
///

inline void SetStartTime (void)
{
	gettimeofday (&_tstart,
	              &tz);
}

///
/// \brief Compute ellapsed time.
///
/// This method returns the seconds elapsed since the call
/// of TIMER::SetStartTime.
///
/// Diese methode liefert die Sekunden zurück, die seit dem
/// Aufruf von TIMER::SetStartTime vergangen sind.
///
/// \return Seconds since call of TIMER::SetSartTime
///
/// \sa TIMER::SetStartTime
///

inline double TimeElapsed (void)
{
	double t1,t2;

	// take the time now

	struct timeval tnow;

	gettimeofday (&tnow,
	              &tz);

	// compute times in seconds

	t1 = (double) _tstart.tv_sec + (double) _tstart.tv_usec/(1000*1000);

	t2 = (double) tnow.tv_sec + (double) tnow.tv_usec/(1000*1000);

	// retrun ellapsed time

	return t2 - t1;
}

///
/// \brief Compute ellapsed time.
///
/// This method returns the seconds elapsed since the call
/// of the given time.
///
/// Diese methode liefert die Sekunden zurück, die seit der
/// übergebenen Zeit vergangen sind.
///
/// \param time start time
///
/// \return seconds since <i>time</i>
///

inline double TimeElapsed (const struct timeval &time)
{
	double t1,t2;

	// take the time now

	struct timeval tnow;

	gettimeofday (&tnow,
	              &tz);

	// compute times in seconds

	t1 = (double) time.tv_sec + (double) time.tv_usec/(1000*1000);

	t2 = (double) tnow.tv_sec + (double) tnow.tv_usec/(1000*1000);

	// retrun ellapsed time

	return t2 - t1;
}

} // end namespace

#endif // TIMER_H
