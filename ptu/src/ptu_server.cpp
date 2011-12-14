/***************************************************************************
    begin                : 2011-06-09
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"
#include "iowarrior_access.h" // for [get|set]IoWarrior()
#include <ptuwrapper.h>

#include <tf/transform_broadcaster.h>

#include "ptu/GetDynamics.h"
#include "ptu/SetDynamics.h"
#include "ptu/Stop.h"
#include "ptu/WaitForCompletion.h"
#include "ptu/GetPositionAbsolute.h"
#include "ptu/SetPositionAbsolute.h"
#include "ptu/SetPositionRelative.h"
#include "ptu/GetRanges.h"

#include "ptu/position.h"

PtuWrapper ptuWrapper;
tf::TransformBroadcaster* transformBroadcaster;
ros::Timer timerTransformBroadcast;

/*
 * This function does two things:
 *  - it publishes the current transform
 *  - it slows down the timer when the PTU isn't moving, so we don't generate useless traffic
 *    This also means that whenever we initiate a motion, we need to reset the timer to higher speed.
 */
void publishCurrentTransformCallback(const ros::TimerEvent& = ros::TimerEvent())
{
	tf::Transform transform;
	float panCurrent, panDesired, tiltCurrent, tiltDesired;
	
	if(ptuWrapper.getPositionAbsolute(panCurrent, tiltCurrent))
	{
		ROS_INFO("publishCurrentTransformCallback(): will now create transform of pan %.2f, tilt %.2f radians", panCurrent, tiltCurrent);
		transform.setOrigin(tf::Vector3(0.1, 0.2, 0.3));
		transform.setRotation(tf::Quaternion(1.0, 2.0, 3.0));
		transformBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "ptu"));
	}
	else
	{
		ROS_ERROR("publishCurrentTransformCallback(): couldn't get absolute position from ptu.");
	}
	
	if(!ptuWrapper.getPositionDesired(panDesired, tiltDesired))
	{
		ROS_ERROR("publishCurrentTransformCallback(): couldn't get desired PTU position!");
		return;
	}
	
	if(fabs(panDesired - panCurrent) < 0.2 && fabs(tiltDesired - tiltCurrent) < 0.2)
	{
		ROS_INFO("publishCurrentTransformCallback(): reached desired position, setting transform updates to 1Hz");
		timerTransformBroadcast.setPeriod(ros::Duration(1.0));
	}
}

/* we use ros::tf now
void publishPosition()
{
	ptu::position msgPosition;

	if(ptuWrapper.getPositionAbsolute(msgPosition.pan, msgPosition.tilt))
	{
		ROS_DEBUG("publishPosition(): will now publish position of %.2f %.2f", msgPosition.pan, msgPosition.tilt);
		pub_position.publish(msgPosition);
	}
	else
	{
		ROS_ERROR("publishPosition(): couldn't get absolute position from ptu.");
	}
}*/

bool set_position_absolute(ptu::SetPositionAbsolute::Request &req, ptu::SetPositionAbsolute::Response &res)
{
	res.success = ptuWrapper.setPositionAbsolute(req.pan, req.tilt);

	ROS_INFO("request: set_position_absolute: sending back response, success: %d", res.success);

	timerTransformBroadcast.setPeriod(ros::Duration(0.05));

	return res.success;
}

bool set_position_relative(ptu::SetPositionRelative::Request &req, ptu::SetPositionRelative::Response &res)
{
	res.success = ptuWrapper.setPositionRelative(req.pan, req.tilt);

	ROS_INFO("request: set_position_relative: sending back response, success: %d", res.success);

	timerTransformBroadcast.setPeriod(ros::Duration(0.05));

	return res.success;
}

bool get_position_absolute(ptu::GetPositionAbsolute::Request &req, ptu::GetPositionAbsolute::Response &res)
{
	res.success = ptuWrapper.getPositionAbsolute(res.pan, res.tilt);
	
	publishCurrentTransformCallback();

	ROS_INFO("request: get_position_absolute: sending back response, success: %d", res.success);

	return res.success;
}

bool get_ranges(ptu::GetRanges::Request &req, ptu::GetRanges::Response &res)
{
	res.success = ptuWrapper.getPositionRanges(res.panMin, res.panMax, res.tiltMin, res.tiltMax);

	ROS_INFO("request: get_ranges: sending back response, success: %d", res.success);

	return res.success;
}


bool wait_for_completion(ptu::WaitForCompletion::Request &req, ptu::WaitForCompletion::Response &res)
{
	res.success = ptuWrapper.waitForCompletion();
	publishCurrentTransformCallback();

	ROS_INFO("request: wait_for_completion: sending back response, success: %d", res.success);

	return res.success;
}

bool stop(ptu::Stop::Request &req, ptu::Stop::Response &res)
{
	res.success = ptuWrapper.stop();

	publishCurrentTransformCallback();

	ROS_INFO("request: stop: sending back response, success: %d", res.success);

	return res.success;
}

bool get_dynamics(ptu::GetDynamics::Request &req, ptu::GetDynamics::Response &res)
{
	res.success =
	    ptuWrapper.getMaximumSpeed(res.spdPan, res.spdTilt)
	    &&
	    ptuWrapper.getAcceleration(res.accPan, res.accTilt);

	ROS_INFO("request: get_dynamics: sending back response, success: %d", res.success);

	return res.success;
}

bool set_dynamics(ptu::SetDynamics::Request &req, ptu::SetDynamics::Response &res)
{
	res.success =
	    ptuWrapper.setMaximumSpeed(req.spdPan, req.spdTilt)
	    &&
	    ptuWrapper.setAcceleration(req.accPan, req.accTilt);

	ROS_INFO("request: set_dynamics: sending back response, success: %d", res.success);

	return res.success;
}

void set_position_absolute_callback(const ptu::position::ConstPtr& msg)
{
	ptuWrapper.setPositionAbsolute(msg->pan, msg->tilt);
	
	timerTransformBroadcast.setPeriod(ros::Duration(0.05));
	publishCurrentTransformCallback();
	
  ROS_INFO("set_position_absolute_callback(): setting positions %.2f / %.2f according to message on topic", msg->pan, msg->tilt);
}

void set_position_relative_callback(const ptu::position::ConstPtr& msg)
{
	ptuWrapper.setPositionRelative(msg->pan, msg->tilt);
	
	timerTransformBroadcast.setPeriod(ros::Duration(0.05));
	publishCurrentTransformCallback();
	
  ROS_DEBUG("set_position_relative_callback(): setting positions %.2f / %.2f according to message on topic", msg->pan, msg->tilt);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ptu_server");
	ros::NodeHandle n;
	
	// Needs to be created after ros::init and needs to be global, thus we use a pointer and init it here.
	transformBroadcaster = new tf::TransformBroadcaster;
	
	// Switch on the PTU power-supply and wait if not already switched on
	bool ptuPower = false;
	if(!(getIoWarrior("ptu", ptuPower) && ptuPower))
	{
		if(!setIoWarrior("ptu", true))
		{
			ROS_FATAL("Couldn't enable ptu-power, exiting");
			exit(1);
		}
		ROS_INFO("Had to switch on ptu by myself. Waiting 40s for it to come up.");
		usleep(40000000);
	}

	if(!ptuWrapper.initialize())
	{
		ROS_ERROR("Couldn't initialize ptu, exiting.");
		return 1;
	}

  // Previously, we published positions on our own topic, but after learning about ros::tf,
  // it seems there's a better way.
// 	pub_position = n.advertise<ptu::position>("ptu/position_absolute", 1);

	ros::ServiceServer serviceRangesGet = n.advertiseService("ptu/get_ranges", get_ranges);
	ros::ServiceServer serviceWait = n.advertiseService("ptu/wait_for_completion", wait_for_completion);
	ros::ServiceServer serviceStop = n.advertiseService("ptu/stop", stop);

	ros::ServiceServer servicePositionGetAbsolute = n.advertiseService("ptu/get_position_absolute", get_position_absolute);
	ros::ServiceServer servicePositionSetAbsolute = n.advertiseService("ptu/set_position_absolute", set_position_absolute);
	ros::ServiceServer servicePositionSetRelative = n.advertiseService("ptu/set_position_relative", set_position_relative);

	ros::ServiceServer serviceDynamicsGet = n.advertiseService("ptu/get_dynamics", get_dynamics);
	ros::ServiceServer serviceDynamicsSet = n.advertiseService("ptu/set_dynamics", set_dynamics);
	
	// subscribe to topics to set the PTU
	ros::Subscriber sub_position_absolute = n.subscribe("ptu/set_position_absolute", 1, set_position_absolute_callback);
	ros::Subscriber sub_position_relative = n.subscribe("ptu/set_position_relative", 1, set_position_relative_callback);
	
	timerTransformBroadcast = n.createTimer(ros::Duration(0.1), publishCurrentTransformCallback);

	ROS_INFO("Your wish is my command.");

	ros::spin();

	return 0;
}



