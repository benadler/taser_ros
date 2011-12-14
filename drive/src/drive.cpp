#include "drive.h"

static const int encoderIncrementPerRevolution = 4096;
static const double dVelMeasPeriod = 500.0;
static const double dRevMotorPerRevGear = 1.0;
static const double dGearRatio = 37.0;

static const double dAngGearRadToAngEncIncr = encoderIncrementPerRevolution * dRevMotorPerRevGear * dGearRatio / (2.0 * M_PI);

double deg2rad(double degree)
{
        return (degree * M_PI / 180);
}

double rad2deg(double radiant)
{
        return (radiant * 180 / M_PI);
}

static inline int radPerSecond_2_encoderIncrementPeriod(double radPerSecond)
{
	return (int)(radPerSecond * dAngGearRadToAngEncIncr / dVelMeasPeriod);
}

static inline double encoderAngle_2_rad(int iEncIncr)
{
	return (double)iEncIncr / dAngGearRadToAngEncIncr;
}

static inline double encoderIncrementPeriod_2_radPerSecond(int iEncVelIncrPeriod)
{
	return (double)iEncVelIncrPeriod / dAngGearRadToAngEncIncr * dVelMeasPeriod;
}

double DriveChain::meter_2_radiant(double meter, RobotSide side)
{
	if(side == sideLeft)
		return meter / mWheelCircumferenceLeft * 2 * M_PI;
	else if(side == sideRight)
		return meter / mWheelCircumferenceRight * 2 * M_PI;
	else
	{
		ROS_ERROR("DriveChain::meter_2_radiant(): illegal robotSide!");
		return -1.0;
	}
}

double DriveChain::radiant_2_meter(double radiant, RobotSide side)
{
	if(side == sideLeft)
		return radiant * 2 * M_PI * mWheelCircumferenceLeft;
	else if(side == sideRight)
		return radiant * 2 * M_PI * mWheelCircumferenceRight;
	else
	{
		ROS_ERROR("DriveChain::radiant_2_meter(): illegal robotSide!");
		return -1.0;
	}
}

DriveChain::DriveChain() : mCanInterface(CAN_ID_IO_REPLY | CAN_ID_LEFT_REPLY | CAN_ID_RIGHT_REPLY)
{
	// this was previously called MAGIC and was defined to be either 1 or 0.987
	_multiplier = 0.987;

	debugAngleRight = 0.0;
	debugPathLengthRight = 0.0;
}

DriveChain::~DriveChain()
{
	ROS_INFO("DriveChain::~DriveChain()");

	disconnectFromControllers();
}

void DriveChain::initialize()
{
		// get the wheel circumferences from the ros parameter server
	ros::NodeHandle nh;
	nh.param<double>("/drive/wheel_circumference_left", mWheelCircumferenceLeft, 0.477);
	nh.param<double>("/drive/wheel_circumference_right", mWheelCircumferenceRight, 0.477);
	
	ROS_INFO("DriveChain::initialize(): using wheel circumferences from parameter server: left: %.2f, right: %.2f", mWheelCircumferenceLeft, mWheelCircumferenceRight);
	
	mCanInterface.initialize();
	
	// 	disconnectFromControllers();

	connectToControllers();

	_emergencyStopEnabled = false;
}

void DriveChain::shutdown(void)
{
	ROS_INFO("DriveChain::shutdown()");
	disconnectFromControllers();
}

void DriveChain::disconnectFromControllers(void)
{
	ROS_INFO("DriveChain::disconnectFromControllers()");

	// Before disconnecting, make sure brakes are applied...
	setBrakeEnabledLeft(true);
	setBrakeEnabledRight(true);

	// ... and motors disabled
	setMotorEnabledLeft(false, 0);
	setMotorEnabledRight(false, 0);

	CanFrame frame;
	
	// Do not disconnect the IO-board, remotecontrol and battery might still use it!
// 	frame = CanInterface::createFrame(CAN_ID_IO_CMD, CMD_IOBOARD_DISCONNECT);
// 	mCanInterface.send(frame);
// 	mCanInterface.receive(&frame, 250);
	
	frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_DISCONNECT);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 250, CMD_MOTCTRL_DISCONNECT);

	frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_DISCONNECT);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 250, CMD_MOTCTRL_DISCONNECT);
	
	sleep (1);
}

void DriveChain::connectToControllers(void)
{
	ROS_INFO("DriveChain::connectToControllers()");

	CanFrame frame;
	
// 	can->send(CanMessage(CAN_ID_IO_CMD, CMD_IOBOARD_CONNECT));
// 	CanMessage message1(CAN_ID_IO_REPLY, CMD_IOBOARD_CONNECT);
// 	can->receiveTimeout(&message1);

	frame = CanInterface::createFrame(CAN_ID_IO_CMD, CMD_IOBOARD_CONNECT);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 250, CMD_IOBOARD_CONNECT);
	
// 	can->send(CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_CONNECT));
// 	CanMessage message2(CAN_ID_LEFT_REPLY, CMD_MOTCTRL_CONNECT);
// 	can->receiveTimeout(&message2);
	
	frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_CONNECT);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 500, CMD_MOTCTRL_CONNECT);

// 	can->send(CanMessage(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_CONNECT));
// 	CanMessage message3(CAN_ID_RIGHT_REPLY, CMD_MOTCTRL_CONNECT);
// 	can->receiveTimeout(&message3);

	frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_CONNECT);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 500, CMD_MOTCTRL_CONNECT);
}

void DriveChain::setBrakeEnabledLeft(const bool enable)
{
	ROS_INFO("DriveChain::setBrakeEnabledLeft(enable: %d)", enable);

// 	can->send(CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_DISABLEBRAKE, !enable));
// 	// Absorb the answer from CAN.
// 	can->receiveNextTimeout();
	CanFrame frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_DISABLEBRAKE, !enable);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 500);
}

void DriveChain::setBrakeEnabledRight(const bool enable)
{
	ROS_INFO("DriveChain::setBrakeEnabledRight(enable: %d)", enable);
	
// 	can->send(CanMessage(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_DISABLEBRAKE, !enable));
// 	// Absorb the answer from CAN.
// 	can->receiveNextTimeout();

	CanFrame frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_DISABLEBRAKE, !enable);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 500);
}

void DriveChain::setMotorEnabledLeft(const bool enable, const int torque)
{
	ROS_INFO("DriveChain::setMotorEnabledLeft(enable: %d)", enable);

// 	can->send(CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_ENABLEMOTOR, enable, torque));
// 	sleep(1);
// 	// Absorb the answer from CAN.
// 	can->receiveNextTimeout();
	
	CanFrame frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_ENABLEMOTOR, enable, torque);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 1250, CMD_MOTCTRL_ENABLEMOTOR);
}

void DriveChain::setMotorEnabledRight(const bool enable, const int torque)
{
	ROS_INFO("DriveChain::setMotorEnabledRight(enable: %d)", enable);
	
// 	can->send(CanMessage(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_ENABLEMOTOR, enable, torque));
// 	sleep(1);
// 	// Absorb the answer from CAN.
// 	can->receiveNextTimeout();
	
	CanFrame frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_ENABLEMOTOR, enable, torque);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 1250, CMD_MOTCTRL_ENABLEMOTOR);
}


void DriveChain::syncMotorLeft(const int torque)
{
	ROS_INFO("DriveChain::syncMotorLeft()");

// 	can->send(CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_SYNCHMOTOR, torque));
	// Warning: I'm not sure what ID/CMD the answer packet will have, need to find out!
// 	sleep(1);
// 	CanMessage message(CAN_ID_LEFT_REPLY, CMD_MOTCTRL_SYNCHMOTOR);
// 	can->receiveTimeout(&message, 5000);

	CanFrame frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_SYNCHMOTOR, torque);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 5000, CMD_MOTCTRL_SYNCHMOTOR);
	
	int iEncPosMeasured = (frame.data[0] << 8) | frame.data[1];
	int iEncPosExpected = (frame.data[2] << 8) | frame.data[3];

	ROS_INFO("DriveChain::syncMotorLeft(): iEncPosMeas = %i, iEncPosExp = %i", iEncPosMeasured, iEncPosExpected);
}

void DriveChain::syncMotorRight(const int torque)
{
	ROS_INFO("DriveChain::syncMotorRight()");

// 	can->send(CanMessage(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_SYNCHMOTOR, torque));
// 	// Warning: I'm not sure what ID/CMD the answer packet will have, need to find out!
// 	sleep(1);
// 	CanMessage message(CAN_ID_RIGHT_REPLY, CMD_MOTCTRL_SYNCHMOTOR);
// 	can->receiveTimeout(&message, 5000);

	CanFrame frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_SYNCHMOTOR, torque);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 5000, CMD_MOTCTRL_SYNCHMOTOR);
	
	int iEncPosMeasured = (frame.data[0] << 8) | frame.data[1];
	int iEncPosExpected = (frame.data[2] << 8) | frame.data[3];

	ROS_INFO("DriveChain::syncMotorRight(): iEncPosMeas = %i, iEncPosExp = %i", iEncPosMeasured, iEncPosExpected);
}

void DriveChain::enableCommutationLeft(void)
{
	ROS_INFO("DriveChain::enableCommLeft()");

// 	can->send(CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_ENABLECOMM));
// 	can->receiveNextTimeout();
	
	CanFrame frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_ENABLECOMM);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame);
}

void DriveChain::enableCommutationRight(void)
{
	ROS_INFO("DriveChain::enableCommRight()");

// 	can->send(CanMessage(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_ENABLECOMM));
// 	can->receiveNextTimeout();
	
	CanFrame frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_ENABLECOMM);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame);
}

void DriveChain::startMotors(void)
{
	ROS_INFO("DriveChain::startMotors()");

	const int torque = 70;

	// init left side
	setBrakeEnabledLeft(false);
	usleep(500000);
	setMotorEnabledLeft(true, torque);
	usleep(500000);
	syncMotorLeft(torque);
	enableCommutationLeft();
	initializeAngleLeft();

	// init right side
	setBrakeEnabledRight(false);
	usleep(500000);
	setMotorEnabledRight(true, torque);
	usleep(500000);
	syncMotorRight(torque);
	enableCommutationRight();
	initializeAngleRight();

	ROS_INFO("DriveChain::startMotors(): done sending packets, absorbing replies. Ignore next error.");

/*
	// It seems that after issuing all the commands above, we get some crap back from CAN. Absorb these messages.
	CanFrame frame;
	while(mCanInterface.receive(&frame, 1000))
	{
		unsigned int cmd = frame.data[7] >> 2;
		ROS_INFO("DriveChain::startMotors: bogus reply with cmd 0x%02x, id 0x%04x received.", cmd, frame.can_id);
	}
*/
	ROS_INFO("DriveChain::startMotors(): done.");
}

bool DriveChain::setBrakesEnabled(const bool enable)
{
#warning: have a look at the motor state diagram and startMotors(). we might need to re-int some stuff after disabling the brakes.
	setBrakeEnabledLeft(enable);
	setBrakeEnabledRight(enable);
	return true;
}

void DriveChain::setMotorSpeeds(int speedLeft, int speedRight)
{
	if(speedLeft < -4000000 || speedRight < -4000000 || speedLeft > 4000000 || speedRight > 4000000)
	{
		ROS_ERROR("DriveChain::setMotorSpeeds(): left/right speeds must be between -4 and 4 m/s, they were %d/%d", speedLeft, speedRight);
		exit(1);
	}

	setMotorSpeedLeft(speedLeft);
	setMotorSpeedRight(speedRight);
}

void DriveChain::setMotorSpeedLeft(int &speed)
{
// 	ROS_INFO("DriveChain::setMotorSpeedLeft()");

	if(_emergencyStopEnabled)
	{
		ROS_INFO("DriveChain::setMotorSpeedLeft(): emergency stop enabled, returning.");
		speed = 0;
		return;
	}

	double speedMS = speed / 1000000.0;

// 	ROS_INFO("DriveChain::setMotorSpeedLeft(): setting speed to %.4F m/s.", speedMS);

	// convert the given speed from m/s to rad/s
	speedMS = meter_2_radiant(speedMS, sideLeft);

	int iSpeedEncoderIncrementPeriod = radPerSecond_2_encoderIncrementPeriod(-speedMS / _multiplier);

	if(iSpeedEncoderIncrementPeriod > 32767) iSpeedEncoderIncrementPeriod = -32767;

	if(iSpeedEncoderIncrementPeriod < -32768) iSpeedEncoderIncrementPeriod = -32768;

	speedMS = -encoderIncrementPeriod_2_radPerSecond(iSpeedEncoderIncrementPeriod) * _multiplier;
	speed = (int)(radiant_2_meter(speedMS, sideLeft) * 1000000.0);

	unsigned int hi = iSpeedEncoderIncrementPeriod >> 8;
	unsigned int lo = iSpeedEncoderIncrementPeriod & 0xff;

	CanFrame frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_SETCMDVAL, 0, 0, 0, 0, hi, lo);
	mCanInterface.send(frame);

	// It seems that we'll get two answers for this:
	// - CAN_ID_LEFT_REPLY,  CMD_MOTCTRL_GETSTATUS
	// - CAN_ID_LEFT_REPLY,  CMD_MOTCTRL_GETPOSVEL
	// It also seems like we don't care about them ATM, so we just absorb them
// 	CanMessage message1(CAN_ID_LEFT_REPLY, CMD_MOTCTRL_GETSTATUS);
// 	can->receiveTimeout(&message1);
		mCanInterface.receive(&frame);

// 	CanMessage message2(CAN_ID_LEFT_REPLY, CMD_MOTCTRL_GETPOSVEL);
// 	can->receiveTimeout(&message2);
// 		mCanInterface.receive(&frame);
}

void DriveChain::setMotorSpeedRight(int &speed)
{
// 	ROS_INFO("DriveChain::setMotorSpeedRight()");

	if(_emergencyStopEnabled)
	{
		ROS_INFO("DriveChain::setMotorSpeedRight(): emergency stop enabled, returning.");
		speed = 0;
		return;
	}

	double speedMS = speed / 1000000.0;

// 	ROS_INFO("DriveChain::setMotorSpeedRight(): setting speed to %.4F m/s.", speedMS);

	// WARNING: it seems the right wheel's speed needs to be reversed
	speedMS *= -1.0;

	// convert the given speed from m/s to rad/s
	speedMS = meter_2_radiant(speedMS, sideRight);

	int iSpeedEncoderIncrementPeriod = radPerSecond_2_encoderIncrementPeriod(-speedMS / _multiplier);

	if(iSpeedEncoderIncrementPeriod > 32767) iSpeedEncoderIncrementPeriod = -32767;

	if(iSpeedEncoderIncrementPeriod < -32768) iSpeedEncoderIncrementPeriod = -32768;

	speedMS = -encoderIncrementPeriod_2_radPerSecond(iSpeedEncoderIncrementPeriod) * _multiplier;

	// again, reversing speed
	speed = -(int)(radiant_2_meter(speedMS, sideRight) * 1000000.0);

	unsigned int hi = iSpeedEncoderIncrementPeriod >> 8;
	unsigned int lo = iSpeedEncoderIncrementPeriod & 0xff;

// 	can->send(CanMessage(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_SETCMDVAL, 0, 0, 0, 0, hi, lo));
	CanFrame frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_SETCMDVAL, 0, 0, 0, 0, hi, lo);
	mCanInterface.send(frame);

	// It seems that we'll get two answers for this:
	// - CAN_ID_RIGHT_REPLY,  CMD_MOTCTRL_GETSTATUS
	// - CAN_ID_RIGHT_REPLY,  CMD_MOTCTRL_GETPOSVEL
	// It also seems like we don't care about them ATM, so we just absorb them
// 	CanMessage message1(CAN_ID_RIGHT_REPLY, CMD_MOTCTRL_GETSTATUS);
// 	can->receiveTimeout(&message1);
	mCanInterface.receive(&frame);

// 	CanMessage message2(CAN_ID_RIGHT_REPLY, CMD_MOTCTRL_GETPOSVEL);
// 	can->receiveTimeout(&message2);
// 	mCanInterface.receive(&frame);
}

void DriveChain::initializeAngleLeft(void)
{
	ROS_INFO("DriveChain::initializeAngleLeft()");

// 	can->send(CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_GETPOSVEL));
	CanFrame frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_GETPOSVEL);
	mCanInterface.send(frame);

// 	CanMessage message(CAN_ID_LEFT_REPLY, CMD_MOTCTRL_GETPOSVEL);
// 	can->receiveTimeout(&message);
	mCanInterface.receive(&frame, 250, CMD_MOTCTRL_GETPOSVEL);


	_iAngleLeft = -(
	                  (frame.data[0] << 24)
	                  |
	                  (frame.data[1] << 16)
	                  |
	                  (frame.data[2] << 8)
	                  |
	                  frame.data[3]
	              );

	ROS_INFO("DriveChain::initializeAngleLeft(): angle = %i (0x%08x)", _iAngleLeft, _iAngleLeft);
}

void DriveChain::initializeAngleRight(void)
{
	ROS_INFO("DriveChain::initializeAngleRight()");

// 	can->send(CanMessage(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_GETPOSVEL));
	CanFrame frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_GETPOSVEL);
	mCanInterface.send(frame);

// 	CanMessage message(CAN_ID_RIGHT_REPLY, CMD_MOTCTRL_GETPOSVEL);
// 	can->receiveTimeout(&message);
	mCanInterface.receive(&frame, 250, CMD_MOTCTRL_GETPOSVEL);

	_iAngleRight = -(
	                  (frame.data[0] << 24)
	                  |
	                  (frame.data[1] << 16)
	                  |
	                  (frame.data[2] << 8)
	                  |
	                  frame.data[3]
	              );

	ROS_INFO("DriveChain::initializeAngleRight(): angle = %i (0x%08x)", _iAngleRight, _iAngleRight);
}

void DriveChain::getMotorSpeeds(int &speedLeft, int &speedRight)
{
	// speed in meters per second
	double speedMS;

	CanFrame frame;

// 	can->send(CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_GETPOSVEL));
// 	CanMessage message1(CAN_ID_LEFT_REPLY, CMD_MOTCTRL_GETPOSVEL);
// 	can->receiveTimeout(&message1);
	
	frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_GETPOSVEL);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 250, CMD_MOTCTRL_GETPOSVEL);
	
	short iVelEncIncrPeriodLeft = -((frame.data[4] << 8) | frame.data[5]);
	speedMS = encoderIncrementPeriod_2_radPerSecond(iVelEncIncrPeriodLeft) * _multiplier;
	speedLeft = (int)(radiant_2_meter(speedMS, sideLeft) * 1000000.0);

// 	can->send(CanMessage(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_GETPOSVEL));
// 	CanMessage message2(CAN_ID_RIGHT_REPLY, CMD_MOTCTRL_GETPOSVEL);
// 	can->receiveTimeout(&message2);
	
	frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_GETPOSVEL);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 250, CMD_MOTCTRL_GETPOSVEL);

	short iVelEncIncrPeriodRight = -((frame.data[4] << 8) | frame.data[5]);
	speedMS = encoderIncrementPeriod_2_radPerSecond(iVelEncIncrPeriodRight) * _multiplier;
	speedRight = (int)(radiant_2_meter(speedMS, sideRight) * 1000000.0);

	ROS_INFO("DriveChain::getMotorSpeeds(): left %d micrometer/s, right %d micrometer/s", speedLeft, speedRight);
}

void DriveChain::getAngleDeltas(double &deltaAngleLeft, double &deltaAngleRight)
{
// 	ROS_INFO("DriveChain::getAngleDeltas()");

	// We time this operation. Take the start timestamp
	struct timeval start;
	gettimeofday(&start, 0x0);

// 	CanMessage messagePosVelLeft(CAN_ID_LEFT_CMD, CMD_MOTCTRL_GETPOSVEL);
// 	can->send(messagePosVelLeft);
// 	// the sent message can be used for receiving, we just need to change the ID from CMD to REPLY
// 	messagePosVelLeft.setId(CAN_ID_LEFT_REPLY);
// 	can->receiveTimeout(&messagePosVelLeft);
	
	CanFrame frameLeft = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_GETPOSVEL);
	mCanInterface.send(frameLeft);
	mCanInterface.receive(&frameLeft, 250, CMD_MOTCTRL_GETPOSVEL);

// 	CanMessage messagePosVelRight(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_GETPOSVEL);
// 	can->send(messagePosVelRight);
// 	// the sent message can be used for receiving, we just need to change the ID from CMD to REPLY
// 	messagePosVelRight.setId(CAN_ID_RIGHT_REPLY);
// 	can->receiveTimeout(&messagePosVelRight);

	CanFrame frameRight = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_GETPOSVEL);
	mCanInterface.send(frameRight);
	mCanInterface.receive(&frameRight, 250, CMD_MOTCTRL_GETPOSVEL);

	int iAngleLeft;

	iAngleLeft = (
	                 (frameLeft.data[0] << 24)
	                 |
	                 (frameLeft.data[1] << 16)
	                 |
	                 (frameLeft.data[2] << 8)
	                 |
	                 frameLeft.data[3]
	             );
	iAngleLeft = -iAngleLeft;

	// Shortly over 0x08000000 the encoder jumps to 0xf8000000, resulting in a very large delta in
	// one cycle. Try to detect the jump and modify the previous value so that the delta becomes
	// reasonable again.

	if((_iAngleLeft > 0x07000000) /*(positive)*/ && (iAngleLeft <= 0))
	{
		_iAngleLeft |= 0xf0000000;
		ROS_INFO("DriveChain::getAngleDeltas(): ************* JUMP CAUGHT (positive left) *************");
	}
	else if((_iAngleLeft < (int)0xf9000000) /*(negative)*/ && (iAngleLeft >= 0))
	{
		_iAngleLeft &= ~0xf0000000;
		ROS_INFO("DriveChain::getAngleDeltas(): ************* JUMP CAUGHT (negative left) *************");
	}

	int deltaLeft = iAngleLeft - _iAngleLeft;   // safe of wraps (must be `int')
	_iAngleLeft = iAngleLeft;
	deltaAngleLeft = encoderAngle_2_rad(deltaLeft);

	int iAngleRight;

	iAngleRight = (
	                 (frameRight.data[0] << 24)
	                 |
	                 (frameRight.data[1] << 16)
	                 |
	                 (frameRight.data[2] << 8)
	                 |
	                 frameRight.data[3]
	              );

	if((_iAngleRight > 0x07000000) /*(positive)*/ && (iAngleRight <= 0))
	{
		_iAngleRight |= 0xf0000000;
		ROS_INFO("DriveChain::getAngleDeltas(): ************* JUMP CAUGHT (positive right) *************");
	}
	else if((_iAngleRight < (int)0xf9000000) /*(negative)*/ && (iAngleRight >= 0))
	{
		_iAngleRight &= ~0xf0000000;
		ROS_INFO("DriveChain::getAngleDeltas(): ************* JUMP CAUGHT (negative right) *************");
	}

	int deltaRight = iAngleRight - _iAngleRight;   // safe of wraps (must be `int')
	_iAngleRight = iAngleRight;
	deltaAngleRight = encoderAngle_2_rad(deltaRight);

	double degreesR = rad2deg(deltaAngleRight);
	double degreesL = rad2deg(deltaAngleLeft);

	debugAngleRight += degreesR;

	struct timeval stop;
	gettimeofday (&stop, 0x0);

	double milliseconds =
	    (stop.tv_sec - start.tv_sec) * 1000.0 +
	    (stop.tv_usec - start.tv_usec) / 1000.0;

	ROS_INFO("DriveChain::getAngleDeltas(): took %.3f millisecs, angleDeltas: l=%.4F  r=%.4F", milliseconds, degreesL, degreesR);
	ROS_INFO("DriveChain::getAngleDeltas(): right wheel rotated %.4F degrees in total", debugAngleRight);
	ROS_INFO("DriveChain::getAngleDeltas(): returning rad angle deltas %.4F / %.4F", deltaAngleLeft, deltaAngleRight);
}

void DriveChain::getMotorAdvances(float &advanceLeft, float &advanceRight)
{
// 	ROS_INFO("DriveChain::getMotorAdvances(): asking getAngleDeltas()...");
	double deltaAngleLeft, deltaAngleRight;
	getAngleDeltas(deltaAngleLeft, deltaAngleRight);

	advanceLeft  = (mWheelCircumferenceLeft  * rad2deg(deltaAngleLeft ) / 360.0);
	advanceRight = (mWheelCircumferenceRight * rad2deg(deltaAngleRight) / 360.0);

	debugPathLengthRight += advanceRight;

	ROS_INFO("DriveChain::getMotorAdvances(): we advanced %.6f / %.6f meters since the last call.", advanceLeft, advanceRight);
	ROS_INFO("DriveChain::getMotorAdvances(): right wheel moved %.4Fm in total.", debugPathLengthRight);
}

void DriveChain::getMotorTemperatures(float &left, float &right)
{
	ROS_INFO("DriveChain::getMotorTemperatures()");


	// ask for the left temperature on the CAN
// 	can->send(CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_GETSTATUS));
// 	CanMessage messageLeft(CAN_ID_LEFT_REPLY,  CMD_MOTCTRL_GETSTATUS);
// 	can->receiveTimeout(&messageLeft);

	CanFrame frame;

	frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_GETSTATUS);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 250, CMD_MOTCTRL_GETSTATUS);

	int tempLeft = (frame.data[2] << 8) | frame.data[3];

	// conversion of left temp
	left = ((-1.0 * tempLeft + 500) / 4.0 + 20.0);

	// ask for the right temperature on the CAN
// 	can->send(CanMessage(CAN_ID_RIGHT_CMD,  CMD_MOTCTRL_GETSTATUS));
// 	CanMessage messageRight(CAN_ID_RIGHT_REPLY, CMD_MOTCTRL_GETSTATUS);
// 	can->receiveTimeout(&messageRight);

	frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_GETSTATUS);
	mCanInterface.send(frame);
	mCanInterface.receive(&frame, 250, CMD_MOTCTRL_GETSTATUS);

	int tempRight = (frame.data[2] << 8) | frame.data[3];

	// conversion of right temp
	right = ((-1.0 * tempRight + 500) / 4.0 + 20.0);

	ROS_INFO("DriveChain::getMotorTemperatures(): temps are %.2F, %.2F degrees celsius.", left, right);
}

void DriveChain::setEmergencyStop(const bool enable)
{
	ROS_INFO("DriveChain::setEmergencyStop(enable: %d)", enable);
	
	CanFrame frame;

	if (_emergencyStopEnabled == true && enable == false)
	{
		// DISABLE emergency stop
// 		can->send(CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_RESETEMSTOP));
// 		can->receiveNextTimeout();
		frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_RESETEMSTOP);
		mCanInterface.send(frame);
		mCanInterface.receive(&frame, 250, CMD_MOTCTRL_RESETEMSTOP);

// 		can->send(CanMessage(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_RESETEMSTOP));
// 		can->receiveNextTimeout();
		frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_RESETEMSTOP);
		mCanInterface.send(frame);
		mCanInterface.receive(&frame, 250, CMD_MOTCTRL_RESETEMSTOP);

		_emergencyStopEnabled = false;
	}

	if (_emergencyStopEnabled == false && enable == true)
	{
		// ENABLE emergency stop
// 		can->send(CanMessage(CAN_ID_LEFT_CMD, CMD_MOTCTRL_SETEMSTOP));
// 		can->receiveNextTimeout();
		frame = CanInterface::createFrame(CAN_ID_LEFT_CMD, CMD_MOTCTRL_SETEMSTOP);
		mCanInterface.send(frame);
		mCanInterface.receive(&frame, 250, CMD_MOTCTRL_SETEMSTOP);

// 		can->send(CanMessage(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_SETEMSTOP));
// 		can->receiveNextTimeout();
		frame = CanInterface::createFrame(CAN_ID_RIGHT_CMD, CMD_MOTCTRL_SETEMSTOP);
		mCanInterface.send(frame);
		mCanInterface.receive(&frame, 250, CMD_MOTCTRL_SETEMSTOP);

		_emergencyStopEnabled = true;
	}

}
