/***************************************************************************
                          ptu.h  -  description
                             -------------------
    begin                : Mon Jan 13 2003
    copyright            : (C) 2003 by Bernd Roessler
    email                : roessler@informatik.uni-hamburg.de
 ***************************************************************************/

/*
 * 2011-06-10: Ben Adler: pasted tools.h below to keep things simple
 */
#ifndef TOOLS_H
#define TOOLS_H

#define DELETE(P) delete(P); \
  P=NULL;

#define DELETEF(P) delete [] P; \
  P=NULL;


#ifdef _DEBUG

#define TRACE1(S) std::cout << S<<std::endl;
#define TRACE2(S1, S2) std::cout << S1<<" "<<S2<<std::endl;
#define WATCH(V) std::cout << #V << ":"<<V<<std::endl;
#define EXEC(S) S;

#else

#define TRACE1(S)
#define TRACE2(S1, S2)
#define WATCH(V)
#define EXEC(S)

#endif


///
/// Lexical-Cast-Operator to cast variables of type int or float to string,
/// or similar operations.
///
#include <sstream>

template<typename TargetType, typename SourceType>
TargetType lexical_cast(SourceType in)
{
	std::stringstream interpreter;

	TargetType out;

	if (!(interpreter << in) ||
	        !(interpreter >> out) ||
	        !(interpreter >> std::ws).eof ())
	{
		throw "bad lexical cast";
	}

	return out;
}

#endif

/*
 * end of paste
 */

/**
* @mainpage C++ Programmers Interface for Pan-Tilt-Unit from Directed Perception, Inc.
* \image html ptu_med.gif
* \image latex ptu_med.eps
*	@section auth Author
* <b>Tim Baier</b>
* <b>Bernd Roessler</b>
* <b>Daniel Westhoff</b> <br>
* TAMS <br>
* Faculty of Computer Science  <br>
* University of Hamburg, Germany  <br>
* mail: roessler@informatik.uni-hamburg.de
*	@section date Date
*	(c) 2003
*       (c) 2004
*	@section intro Introduction
*	This library provides a C++ class for easy communication with the Pan-Tilt Unit by <i>Directed Perception, Inc</i>.
* For further information feel free to contact me.
*	@section install Installation
*	<i>./configure</i> followed by <i>make install</i>
*/

#ifndef PTU_H
#define PTU_H

#include <cmath>
#include <string>
#include <iostream>

/** @file ptu.h */

#define PAN_DEG2POS(ANG) (int) (ANG*panResolution) /**<@brief Converts degree to position units for pan axis.*/
#define TILT_DEG2POS(ANG) (int) (ANG*tiltResolution) /**<@brief Converts degree to position units for tilt axis.*/
#define PAN_POS2DEG(POS)  ((double)POS)/panResolution /**<@brief Converts position units to degree for pan axis.*/
#define TILT_POS2DEG(POS) ((double)POS)/tiltResolution /**<@brief Converts position units to degree for tilt axis.*/
#define ANSWER_TO_INT(A) atoi(std::string(A,2,A.size()).c_str());
#define ANSWER_TO_DOUBLE(A) atof(std::string(A,2,A.size()).c_str())

// order of this strings is important (used by checkError)
static const char* PTUException_info[] =
{
	"PTU was not correctly initialized (use PTU::init() to initialize the PTU).",
	"PTU is allready initialized.",
	"There was no answer from the PTU. Check the connection.",
	"Initialization of PTU failed.",
	"Undefined PTU answer error."
};

/**
* @brief Defines the axes for the PTU.
*/
enum PtuAxis
{
	PTU_PAN, /**<@brief The pan axis. */
	PTU_TILT /**<@brief The tilt axis. */
};

/**
* @brief Defines the different units for PTU position commands and queries.
*/
enum PtuUnit
{
	PTU_DEGREE, /**<@brief Degree. */
	PTU_TICKS /**<@brief Ticks as provided by Pan-Tilt Unit. */
};

/**
*@brief Defines the different power modes for motors when not in-transit.
*/
enum HoldPowerMode
{
	HPM_OFF, /**<@brief Hold power mode is off. */
	HPM_LOW, /**<@brief Low hold power mode. */
	HPM_REGULAR /**<@brief Regular hold power mode. */
};

/**
*@brief Defines the different power modes for motors when in-motion.
*/
enum MovePowerMode
{
	MPM_LOW, /**<@brief Low move power mode. */
	MPM_REGULAR, /**<@brief Regular move power mode. */
	MPM_HIGH /**<@brief High move power mode. */
};

/**
* @brief Class for PTU-Exceptions.
*/
class PTUException
{
public:
	/**
	* @brief Defines the different errors.
	*/
	enum error
	{
		EXCP_NOT_INITIALIZED, /**<@brief PTU was not correctly initialized.*/
		EXCP_ALLREADY_INITIALIZED, /**<@brief PTU is allready initialized.*/
		EXCP_READ, /**<@brief No answer from PTU */
		EXCP_INITIALIZATION_FAILED, /**<@brief Initialization of PTU failed. */
		EXCP_UNDEF_ERROR, /**<@brief Undefined PTU error.*/
		EXCP_COMMAND /**<@brief A command to the PTU returned an error. */
	} err; /**<@brief Holds the error.*/



private:
	std::string msg;

public:
	PTUException (const error e)
	{
		msg = PTUException_info[e];
		err = e;
	};

	PTUException (const std::string& str)
	{
		msg = str;
		err = EXCP_COMMAND;
	}

	/**
	* @brief Prints out the error message.
	*/
	const void show ()
	{
		std::cout << msg << std::endl;
	};

	/**
	* @brief Operator for error message output.
	*/
	friend std::ostream& operator<<(std::ostream& os,const PTUException& e)
	{
		os << e.msg << std::endl;
		return os;
	}

	/**
	* @brief Operator for comparing the error.
	*/
	bool operator==(error e)
	{
		return (err == e);
	}

	const std::string text()
	{
		return msg;
	}
};



class SerialInterface;


enum MAXMIN
{
	MAX,
	MIN
};

/**
*	@brief Class for controlling the PTU.
* @author Bernd Roessler
*/

class PTU
{
public:
	/**
	* @brief Contructor
	*
	*@param devName device to which the PTU is connected to (e.g. /dev/ttyS0).
	*/
	PTU(const std::string devName);

	~PTU();

	/**
	* @brief Initializes the communication with the PTU.
	*/
	void init () throw (PTUException);

	/** @name Position-Limits Queries
	* With the following functions one can send position control commands and queries to the PTU.
	*/
	//@{

	/**
	* @brief Queries maximum allowed pan or tilt position.
	*
	*@param axis desired \ref PtuAxis.
	* @param unit desired \ref PtuUnit for position.
	*/
	double getMaxPosition(const PtuAxis axis, const PtuUnit unit = PTU_DEGREE)
	{
		return limitPos(axis,unit,MAX);
	};

	/**
	* @brief Queries minimum allowed pan or tilt position.
	*
	* @param axis desired \ref PtuAxis.
	* @param unit desired \ref PtuUnit for position.
	*/
	double getMinPosition(const PtuAxis axis, const PtuUnit unit = PTU_DEGREE)
	{
		return limitPos(axis,  unit,MIN);
	};

	//@}

	/** @name Absolute Position Control Commands and Queries
	* With the following functions one can send absolute position control commands and queries to the PTU.
	*/
	//@{

	/**
	* @brief Specifies the absolute pan or tilt position.
	*
	* Desired positions can be changed on-the-fly
	* without waiting for previous position commands to complete.
	* @throws PTUException
	* @param axis desired \ref PtuAxis.
	* @param position desired absolute position.
	* @param unit desired \ref PtuUnit for position.
	* @see getPosition
	*/
	void setPosition (const PtuAxis axis, const double position, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		command(axis,position,unit,"P");
	};

	/**
			* @brief Specifies the absolute pan and tilt position.
			*
			* Desired positions can be changed on-the-fly
			* without waiting for previous position commands to complete.
			* @throws PTUException
			* @param pan_position desired absolute pan position.
			* @param tilt_position desired absolute tilt position.
			* @param unit desired \ref PtuUnit for position.
			* @see getPosition
			*/
	void setPosition (const double pan_position, const double tilt_position, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		setPosition(PTU_PAN,pan_position,unit);
		setPosition(PTU_TILT,tilt_position,unit);
	};

	/**
	* @brief Queries the current absolute pan or tilt position.
	*
	* @throws PTUException
	* @param axis desired \ref PtuAxis.
	* @param unit desired \ref PtuUnit for position.
	* @see setPosition
	*/
	double getPosition (const PtuAxis axis, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		return query(axis,unit,"P");
	};

	//@}


	/** @name Relative Position Control Commands and Queries
	* With the following functions one can send relative position control commands and queries to the PTU.
	*/
	//@{

	/**
	* @brief Specifies the relative pan or tilt position.
	* Specify desired axis position as an offset from the current position.
	* Desired offset positions can be changed on-the-fly without waiting for previous position commands to complete.
	*
	* @throws PTUException
	* @param axis desired \ref PtuAxis.
	* @param offset desired relative offset.
	* @param unit desired \ref PtuUnit for position.
	* @see getDesPosition
	*/
	void setRelPosition (const PtuAxis axis, const double offset, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		command(axis,offset,unit,"O");
	};
	/**
			* @brief Specifies the relative pan and tilt position.
			* Specify desired axis position as an offset from the current position.
			* Desired offset positions can be changed on-the-fly without waiting for previous position commands to complete.
			*
			* @throws PTUException
			* @param pan_offset desired relative pan offset.
			* @param tilt_offset desired relative tilt offset.
			* @param unit desired \ref PtuUnit for position.
			* @see getDesPosition
			*/
	void setRelPosition (const double pan_offset, const double tilt_offset, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		setRelPosition(PTU_PAN,pan_offset,unit);
		setRelPosition(PTU_TILT,tilt_offset,unit);
	};

	/**
	* @brief Queries the desired pan or tilt position.
	*
	* @throws PTUException
	* @param axis desired \ref PtuAxis.
	* @param unit desired \ref PtuUnit for position.
	* @see setRelPosition
	*/
	double getDesPosition (const PtuAxis axis, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		return query(axis,unit,"O");
	};

	/**
	* @brief Moves pan-axis of PTU by a special amount to the right.
	*
	* @throws PTUException
	* @param offset desired offset.
	* @param unit desired \ref PtuUnit for offset.
	*/
	void moveRight (const double offset, PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		command(PTU_PAN,fabs(offset),unit,"O-");
	};

	/**
	* @brief Moves pan-axis of PTU by a special amount to the left.
	*
	* @throws PTUException
	* @param offset desired offset.
	* @param unit desired \ref PtuUnit for offset.
	*/
	void moveLeft (const double offset, PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		command(PTU_PAN,fabs(offset),unit,"O");
	};

	/**
	* @brief Moves tilt-axis of PTU by a special amount up.
	*
	* @throws PTUException
	* @param offset desired offset.
	* @param unit desired \ref PtuUnit for offset.
	*/
	void moveUp (const double offset, PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		command(PTU_TILT,fabs(offset),unit,"O");
	};

	/**
	* @brief Moves tilt-axis of PTU by a special amount down.
	*
	* @throws PTUException
	* @param offset desired offset.
	* @param unit desired \ref PtuUnit for offset.
	*/
	void moveDown(const double offset, PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		command(PTU_TILT,fabs(offset),unit,"O-");
	};

	//@}


	/**
	* @name General Position Control Commands
	*/
	//@{

	/**
	* @brief Instructs the PTU to immediately execute positional commands.
	*
	* @see setQueuedMode
	*/
	void setImmediateMode() throw (PTUException)
	{
		send(std::string("I"));
	};

	/**
	* @brief Instructs PTU to queue positional commands.
	*
	* Commands are executed when waitForCompleted() or
	* setImmediateMode() is called.
	* @see setImmediateMode
	* @see waitForCompleted
	*/
	void setQueuedMode() throw (PTUException)
	{
		send(std::string("S"));
	};

	/**
			* @brief Awaits the completion of the last issued pan and tilt axis commands.
			*
			* Used to coordinate axis motions.
			* @see setQueuedMode
			*/
	void waitForCompleted() throw (PTUException)
	{
		send(std::string("A"));
	};

	/**
	* @brief Immediately decelerates and halts all PTU movement.
	*
	*/
	void stop() throw (PTUException)
	{
		send(std::string("H"));
	};

	/**
	* @brief Immediately decelerates and halts PTU pan or tilt movement.
	*
	* @param axis desired \ref PtuAxis.
	* @see stop
	*/
	void stop(PtuAxis axis) throw (PTUException)
	{
		(axis == PTU_PAN) ? send(std::string("HP")) : send(std::string("HT"));
	};

	//@}


	/** @name Speed Control Commands and Queries
			* The Pan-Tilt Unit provides for precise control of axis speed and acceleration.
			* As shown in Figure 1, upper and lower speed limits determine the bounds on nonstationary pan-tilt velocities.
			* The base (start-up) speed specifies the velocity at which the pan-tilt axis can be started from a full stop without losing
			* synchrony, and it is more a function of the motors rather than load characteristics. Due to base
			* speed requirements and the property that motors lose torque as speed increases, acceleration is required to achieve
			* axis speeds above the base rate. The pan-tilt controller uses trapezoidal acceleration and deceleration for speeds above
			* the base rate and less than the maximum allowed speed. Figure 1 shows two acceleration cases. In the first, an axis
			* accelerates up to a desired constant speed (slew rate), then decelerates. The second case shows the case when the
			* unit does not have sufficient time to accelerate up to the desired slew speed before the need to decelerate to the desired
			* position. The pan-tilt controller provides for on-the-fly position and speed changes. If the direction is changed on-the-fly,
			* the controller manages all deceleration, direction reversal, and acceleration to achieve the most recently specified
			* target pan-tilt speed and acceleration rates. Because speed, acceleration, and position are precisely controlled, you can
			* accurately and simply predict the position attained by the pan-tilt unit in time.
			* \image html speed.jpg "Figure 1: Axis Speed, Instantaneous Speeds, Trapezoidal Acceleration, and On-The_Fly Speed and Position Changes"
			* \image latex speed.eps "Figure 1: Axis Speed, Instantaneous Speeds, Trapezoidal Acceleration, and On-The_Fly Speed and Position Changes"
			*/
	//@{

	/**
	* @brief Set desired absolute pan or tilt speed.
	*
	* Desired speed is specified in \ref PtuUnit /second and it can be changed on-the-fly.
	* The speed specifies that rate at which the pan-tilt move to achieve position movement commands.
	* Desired speed commands outside the speed bounds throw an exception and are not executed.
	* @param axis desired \ref PtuAxis.
	* @param speed desired speed in \ref PtuUnit /second.
	* @param unit desired \ref PtuUnit for speed.
	* @see getDesSpeed
	*/
	void setSpeed(const PtuAxis axis, const double speed, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		command(axis,speed,unit,"S");
	};

	/**
	* @brief Set desired absolute pan and tilt speed.
	*
	* Desired speed is specified in \ref PtuUnit /second and it can be changed on-the-fly.
	* The speed specifies that rate at which the pan-tilt move to achieve position movement commands.
	* Desired speed commands outside the speed bounds throw an exception and are not executed.
	* @param pan_speed desired pan speed in \ref PtuUnit /second.
	* @param tilt_speed desired tilt speed in \ref PtuUnit /second.
	* @param unit desired \ref PtuUnit for speed.
	* @see getDesSpeed
	*/
	void setSpeed(const double pan_speed, const double tilt_speed, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		setSpeed(PTU_PAN,pan_speed,unit);
		setSpeed(PTU_TILT,tilt_speed,unit);
	};

	/**
	* @brief Set desired absolute pan and tilt speed to the same value.
	*
	* Desired speed is specified in \ref PtuUnit /second and it can be changed on-the-fly.
	* The speed specifies that rate at which the pan-tilt move to achieve position movement commands.
	* Desired speed commands outside the speed bounds throw an exception and are not executed.
	* @param speed desired pan and tilt speed in \ref PtuUnit /second.
	* @param unit desired \ref PtuUnit for speed.
	* @see getDesSpeed
	*/
	void setSpeed(const double speed, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		setSpeed(PTU_PAN,speed,unit);
		setSpeed(PTU_TILT,speed,unit);
	};

	/**
	* @brief Queries desired pan or tilt speed.
	* @param axis desired \ref PtuAxis.
	* @param unit desired \ref PtuUnit for speed.
	* @see setSpeed
	*/
	double getDesSpeed(const PtuAxis axis, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		return query(axis, unit,"S");
	};

	/**
			 * @brief Set desired relative pan or tilt speed.
			 *
			 * Specifies desired pan or tilt axis speed as an offset from the current speed.
			 * Desired delta (offset) speed is specified in \ref PtuUnit /second and it can be changed on-the-fly.
			 * A desired delta speed command that results in a speed outside the legal speed bounds throws an exception and it is not executed.
			 * @param axis desired \ref PtuAxis.
			 * @param speed desired speed in \ref PtuUnit /second.
			 * @param unit desired \ref PtuUnit for speed.
			 * @see getSpeed
			 */
	void setRelSpeed(const PtuAxis axis, const double speed, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		command(axis,speed,unit,"D");
	};

	/**
	* @brief Set desired relative pan and tilt speed.
	*
	* Specifies desired pan or tilt axis speed as an offset from the current speed.
	* Desired delta (offset) speed is specified in \ref PtuUnit /second and it can be changed on-the-fly.
	* A desired delta speed command that results in a speed outside the legal speed bounds throws an exception and it is not executed.
	* @param pan_speed desired pan speed in \ref PtuUnit /second.
	* @param tilt_speed desired tilt speed in \ref PtuUnit /second.
	* @param unit desired \ref PtuUnit for speed.
	* @see getSpeed
	*/
	void setRelSpeed(const double pan_speed, const double tilt_speed, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		setRelSpeed(PTU_PAN,pan_speed,unit);
		setRelSpeed(PTU_TILT,tilt_speed,unit);
	};

	/**
	* @brief Queries current pan or tilt speed.
	* @param axis desired \ref PtuAxis.
	* @param unit desired \ref PtuUnit for speed.
	* @see setRelSpeed
	*/
	double getSpeed(const PtuAxis axis, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		return query(axis,unit,"D");
	};

	/**
	* @brief Set desired pan or tilt base speed.
	*
	* Specify pan or tilt axis base (start-up) speed. Base speed is specified in \ref PtuUnit /second.
	* @param axis desired \ref PtuAxis.
	* @param speed desired base speed in \ref PtuUnit /second.
	* @param unit desired \ref PtuUnit for speed.
	* @see getBaseSpeed
	*/
	void setBaseSpeed(const PtuAxis axis, const double speed, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		command(axis,speed,unit,"B");
	};

	/**
	* @brief Set desired pan and tilt base speed.
	*
	* Specify pan or tilt axis base (start-up) speed. Base speed is specified in \ref PtuUnit /second.
	* @param pan_speed desired pan base speed in \ref PtuUnit /second.
	* @param tilt_speed desired tilt base speed in \ref PtuUnit /second.
	* @param unit desired \ref PtuUnit for speed.
	* @see getBaseSpeed
	*/
	void setBaseSpeed(const double pan_speed, const double tilt_speed, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		setBaseSpeed(PTU_PAN,pan_speed,unit);
		setBaseSpeed(PTU_TILT,tilt_speed,unit);
	};

	/**
	* @brief Queries pan or tilt base speed.
	* @param axis desired \ref PtuAxis.
	* @param unit desired \ref PtuUnit for speed.
	* @see setBaseSpeed
	*/
	double getBaseSpeed(const PtuAxis axis, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		return query(axis,unit,"B");
	};

	/**
	* @brief Set desired pan or tilt acceleration.
	*
	* Specifies pan or tilt axis acceleration and deceleration for speeds above the base speed. Acceleration is specified in \ref PtuUnit /second^2.
	* Acceleration cannot be changed on the fly.
	* @param axis desired \ref PtuAxis.
	* @param accel desired acceleration in \ref PtuUnit /second^2.
	* @param unit desired \ref PtuUnit for acceleration.
	* @see getAccel
	*/
	void setAccel(const PtuAxis axis, const double accel, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		command(axis,accel,unit,"A");
	};

	/**
	* @brief Set desired pan and tilt acceleration.
	*
	* Specifies pan or tilt axis acceleration and deceleration for speeds above the base speed. Acceleration is specified in \ref PtuUnit /second^2.
	* Acceleration cannot be changed on the fly.
	* @param pan_accel desired pan acceleration in \ref PtuUnit /second^2.
	* @param tilt_accel desired tilt acceleration in \ref PtuUnit /second^2.
	* @param unit desired \ref PtuUnit for acceleration.
	* @see getAccel
	*/
	void setAccel(const double pan_accel, const double tilt_accel, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		setAccel(PTU_PAN,pan_accel,unit);
		setAccel(PTU_TILT,tilt_accel,unit);
	};

	/**
	* @brief Queries desired pan or tilt acceleration.
	* @param axis desired \ref PtuAxis.
	* @param unit desired \ref PtuUnit for acceleration.
	* @see setAccel
	*/
	double getAccel(const PtuAxis axis, const PtuUnit unit = PTU_DEGREE) throw (PTUException)
	{
		return query(axis, unit,"A");
	};

	//@}

	/** @name Speed Control Modes
	* By default, position control commands are independent from the speed control commands.
	* In this independent control mode, the commanded speed is an unsigned magnitude that determines
	* the speed at which independently commanded positions are effected, and the execution of these speed commands
	* do not affect the commanded desired positions themselves. This mode is appropriate for pure position
	* control methods (when pan-tilt control is effected solely by commanding pan-tilt position) and hybrid
	* positionvelocity control methods (when pan-tilt positions and the rate at which they are achieved are both controlled).
	* An alternative pan-tilt control method uses a pure velocity control mode in which all pan-tilt control is effected by
	* signed changes in command axis speed. In this mode, the speed command specifies a signed velocity in which the
	* sign determines the direction of axis movement, and the ordinal value specifies the speed of movement in this
	* direction. In this mode, if the commanded speed is negative, the axis is automatically commanded to the minimum
	* axis position. Conversely, if the speed command is positive, the axis is automatically commanded to the
	* maximum axis position. If the absolute value of the commanded speed is less than the lower speed bound,
	* a speed of zero is applied by halting the axis motion. It is important to note that that in pure velocity control mode,
	* a speed command for a given axis effectively overrides currently executing position commands. As a result, the
	* speed control mode at power up is always set to independent control mode; the speed control mode is not saved as defaults
	* that are preserved when the unit is powered back up. These commands are available in PTU firmware versions 1.09.7 and higher.
	*/
	//@{

	/**
	* @brief Set independent control mode.
	*/
	void setIndependentControl()
	{
		send(std::string("CI"));
	};

	/**
	* @brief Set pure velocity control mode.
	*/
	void setPureVelocityControl()
	{
		send(std::string("CV"));
	};

	//@}

	/** @name Power Control Commands and Queries
	* A key advantage of the constant current motor control drivers used in the Pan-Tilt Controller is that it
	* allows the current consumed by the pan-tilt unit to be controlled via simple unit commands. These capabilities
	* are useful for battery powered operation, reducing unit heat generation,
	* and extending the rated life of the motor driver circuitry.
	*/
	//@{

	/**
	* @brief  Set pan or tilt hold power mode.
	* @param axis desired \ref PtuAxis.
	* @param mode desired \ref HoldPowerMode.
	* @see setMovePowerMode
	*/
	void setHoldPowerMode (const PtuAxis axis, HoldPowerMode mode)
	{
		hpm(axis,mode);
	};

	/**
	* @brief  Set pan or tilt move power mode.
	* @param axis desired \ref PtuAxis.
	* @param mode desired \ref MovePowerMode.
	* @see setHoldPowerMode
	*/
	void setMovePowerMode (const PtuAxis axis, MovePowerMode mode)
	{
		mpm(axis,mode);
	};

	/**
	* @brief  Queries pan or tilt hold power mode.
	* @param axis desired \ref PtuAxis.
	* @see getMovePowerMode
	*/
	HoldPowerMode getHoldPowerMode (const PtuAxis axis);

	/**
	* @brief  Queries pan or tilt move power mode.
	* @param axis desired \ref PtuAxis.
	* @see setHoldPowerMode
	*/
	MovePowerMode getMovePowerMode (const PtuAxis axis);


	//@}

private:
	SerialInterface* com;
	double panResolution,tiltResolution;
	int maxPP, minPP,maxTP,minTP;

	std::string errorCheck() throw (PTUException);
	std::string errorCheck(double timeout) throw (PTUException);
	std::string send(std::string s) throw (PTUException);
	std::string read(double timeout = .5) throw (PTUException);
	void command(const PtuAxis axis, const double param, const PtuUnit unit, const std::string command) throw (PTUException);
	double query(const PtuAxis axis, const PtuUnit unit, const std::string command) throw (PTUException);
	double limitPos(const PtuAxis axis, const PtuUnit unit, const MAXMIN minmax);
	void hpm(const PtuAxis axis, const HoldPowerMode m);
	void mpm(const PtuAxis axis, const MovePowerMode m);
};

#endif
