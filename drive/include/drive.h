#ifndef DRIVECHAIN_H
#define DRIVECHAIN_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

#include <ros/ros.h>
#include <caninterface.h>
#include <cmd_ioboard.h>
#include <cmd_motctrl.h>

// Following are the hexadecimal IDs of the CAN-Devices:
// IOBoard:     0x0101 for sending, 0x0100 for receiving
// Right Motor: 0x0201 for sending, 0x0200 for receiving
// Left  Motor: 0x0301 for sending, 0x0300 for receiving

#define CAN_ID_LEFT_CMD 0x0301		// dec 769
#define CAN_ID_LEFT_REPLY 0x0300	// dec 768
#define CAN_ID_RIGHT_CMD 0x0201		// dec 513
#define CAN_ID_RIGHT_REPLY 0x0200	// dec 512

enum RobotSide
{
        sideLeft,
        sideRight,
        sideFront,
        sideRear,

        sideInvalid
};

/// @class DriveChain
/// @brief The DriveChain-class represents the two motors, their brakes, odometers (=encoders) and temperature sensors.
/// This class is a little more complex, because it does many things. It initializes
/// all the motor-related devices on the CAN and allows them to be read and set. You
/// can get & set the motor's speeds, get the motor's temperatures and enable/disable
/// the brakes.
///
/// The private methods are mostly needed for initialization purposes.
///
/// You *can* use setBrakesEnabled() for using the brakes, but applying and releasing
/// the brakes like that takes along time (have a look the the motor state diagram
/// from Neobotix to see why). Instead, you should use setEmergencyStop(), which will
/// also apply the brakes, but execute much faster.

class DriveChain
{
public:
	DriveChain();
	~DriveChain();
	
	// Do this first!
	void initialize();

	///
	/// This method reads the encoders for the current speeds, converts them to m/s and writes them
	/// into the arguments
	/// @param speedLeft this parameter will be filled with the current left speed in micrometer/s
	/// @param speedRight this parameter will be filled with the current right speed in micrometer/s
	///
	void getMotorSpeeds(int &speedLeft, int &speedRight);

	///
	/// This method sets the motor speeds in m/s. The given parameters are overwritten with the real value that the motors
	/// are sent. This value might be slightly different due to rounding errors. This method does some bounds checking.
	/// @param speedLeft the left speed in micrometer/s
	/// @param speedRight the right speed in micrometer/s
	///
	void setMotorSpeeds(int speedLeft, int speedRight);

	///
	/// This method returns how many meters each wheel has advanced since the last call of this method.
	/// @param left this argument will be set to the left advance in meters
	/// @param right this argument will be set to the right advance in meters
	///
	void getMotorAdvances(float &left, float &right);

	///
	/// This method returns the motor's temperatures
	/// @param left a reference to an int which will be set to the left temperature in degrees C.
	/// @param right a reference to an int which will be set to the right temperature in degrees C.
	///
	void getMotorTemperatures(float &left, float &right);

	///
	/// This method does the complete startup & initialization procedure necessary to get the platform moving.
	/// It loosens the brakes, enables the motors, syncs them, enables the communication and initializes the angles
	/// for both motors. Takes a while to complete...
	///
	void startMotors(void);

	///
	/// This method simply calls disconnectFromControllers(), but its public. Use this to apply the brakes and cleanly shutdown.
	///
	bool setBrakesEnabled(const bool enable);

	///
	/// This method activates/deactivates the motors emergency-stop feature.
	/// Emergency Stop is great for applying brakes when idle, because both applying and loosening the
	/// brakes doesn't require the whole motor init sequence, which takes a lot of time.
	/// @param enable pass true to enable emergency stop, false to disable it.
	///
	void setEmergencyStop(const bool enable);
	
	bool getEmergencyStop() { return _emergencyStopEnabled; };

	///
	/// This method simply calls disconnectFromControllers(), but its public. Use this to apply the brakes and cleanly shutdown.
	///
	void shutdown(void);

private:
	CanInterface mCanInterface;

	double mWheelCircumferenceLeft, mWheelCircumferenceRight;
	
	double debugAngleRight;
	double debugPathLengthRight;

	double _multiplier;		///< The same as the MAGIC define found in mobiled v3. I have no idea why this is needed.
	bool _emergencyStopEnabled;	///< A boolean holding the status of emergency stop. Always up-to-date.
	int _iAngleLeft, _iAngleRight;	///< These hold the latest angle-values from both motor's encoders. Used to calculate the angle deltas

	double meter_2_radiant(double meter, RobotSide side);
	double radiant_2_meter(double radiant, RobotSide side);

	///
	/// This method reads the encoders for the current angles, compares these with the angles
	/// taken last time and writes those values into the given arguments
	/// An older comment said these would be VELOCITIES. I don't know!
	/// @param deltaAngleLeft this argument will be filled with the difference to the last left angle
	/// @param deltaAngleRight this argument will be filled with the difference to the last right angle
	///
	void getAngleDeltas(double &deltaAngleLeft, double &deltaAngleRight);

	///
	/// This method sets the left motor's brake
	/// @param enable whether to enable (true) or disable (false) the brake.
	///
	void setBrakeEnabledLeft(const bool enable);

	///
	/// This method sets the right motor's brake
	/// @param enable whether to enable (true) or disable (false) the brake.
	///
	void setBrakeEnabledRight(const bool enable);

	///
	/// This method sends a packet CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_ENABLEMOTOR, enable, torque)
	/// and enables the motor. Seems to be some needed initialization step.
	/// @param enable whether the motor should be enabled (true) or disabled (false).
	/// @param torque I don't yet know what this is used for. Does it really set the motor's torque/strength/power?
	///
	void setMotorEnabledLeft(const bool enable, const int torque = 0);

	///
	/// This method sends a packet CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_ENABLEMOTOR, enable, torque)
	/// and enables the motor. Seems to be some needed initialization step.
	/// @param enable whether the motor should be enabled (true) or disabled (false).
	/// @param torque I don't yet know what this is used for. Does it really set the motor's torque/strength/power?
	///
	void setMotorEnabledRight(const bool enable, const int torque = 0);

	///
	/// This method synchronizes the left motor to its encoder. This means that the motor will rotate just
	/// enough to make the encoder see at what position it is. It can then be used for e.g. odometry.
	/// @param torque I don't yet know what this is used for. Does it really set the motor's torque/strength/power?
	///
	void syncMotorLeft(const int torque = 0);

	///
	/// This method synchronizes the right motor to its encoder. This means that the motor will rotate just
	/// enough to make the encoder see at what position it is. It can then be used for e.g. odometry.
	/// @param torque I don't yet know what this is used for. Does it really set the motor's torque/strength/power?
	///
	void syncMotorRight(const int torque = 0);

	///
	/// I have no idea what this method does, but its called only from startMotors(), so it seems to be
	/// some kind of necessary initialization. This method simply sends a CAN-packet with ID
	/// CAN_ID_LEFT_CMD and command CMD_MOTCTRL_ENABLECOMM.
	///
	void enableCommutationLeft(void);

	///
	/// I have no idea what this method does, but its called only from startMotors(), so it seems to be
	/// some kind of necessary initialization. This method simply sends a CAN-packet with ID
	/// CAN_ID_RIGHT_CMD and command CMD_MOTCTRL_ENABLECOMM.
	void enableCommutationRight(void);

	///
	/// This method sets the left motor's speed
	/// @param speed the speed in micrometer/s
	///
	void setMotorSpeedLeft(int &speed);

	///
	/// This method sets the right motor's speed
	/// @param speed the speed in micrometer/s
	///
	void setMotorSpeedRight(int &speed);

	///
	/// This method disconnects from the motor's controllers on the CAN. Before it does so,
	/// it ensures the motors are disabled and the brakes are applied.
	///
	void disconnectFromControllers();

	///
	/// This method connects to the motor's controllers on the CAN.
	///
	void connectToControllers();

	///
	/// This method receives the current encoder angle for the left motor and sets the value of the member-variable
	/// _iAngleLeft to that value. This way, the first call to getDeltaAngleLeft() has a value to compare against.
	///
	void initializeAngleLeft(void);

	///
	/// This method receives the current encoder angle for the right motor and sets the value of the member-variable
	/// _iAngleRight to that value. This way, the first call to getDeltaAngleRight() has a value to compare against.
	///
	void initializeAngleRight(void);
};

#endif
