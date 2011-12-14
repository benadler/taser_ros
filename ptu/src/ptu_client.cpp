/***************************************************************************
    begin                : 2011-06-09
    copyright            : (C) 2011 Ben Adler
    email                : adler@informatik.uni-hamburg.de
    license              : BSD
 ***************************************************************************/

#include "ros/ros.h"
#include "ptu/GetPositionAbsolute.h"
#include "ptu/SetPositionAbsolute.h"
#include "ptu/SetPositionRelative.h"

#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ptu_client");

	ros::NodeHandle n;

	if(argc == 2 && std::string(argv[1]).compare("get") == 0)
	{
		// get values from ptu_server
		ros::ServiceClient client = n.serviceClient<ptu::GetPositionAbsolute>("ptu/get_position_absolute");
		ptu::GetPositionAbsolute srv;

		if(client.call(srv))
		{
			ROS_INFO("get success %d, ptu values pan / tilt: %.2f / %.2f", srv.response.success, srv.response.pan, srv.response.tilt);
		}
		else
		{
			ROS_ERROR("Failed to call service ptu::get_position_absolute");
			return 1;
		}
	}
	else if(argc == 4)
	{
		// parse pan and tilt values
		float pan, tilt;

		if(EOF == sscanf(argv[2], "%f", &pan))
		{
			ROS_ERROR("couldn't parse pan-value from \"%s\"", argv[2]);
			return 1;
		}

		if(EOF == sscanf(argv[3], "%f", &tilt))
		{
			ROS_ERROR("couldn't parse tilt-value from \"%s\"", argv[2]);
			return 1;
		}

		// execute either abs or rel service of ptu sever
		if(std::string(argv[1]).compare("abs") == 0)
		{
			ros::ServiceClient client = n.serviceClient<ptu::SetPositionAbsolute>("ptu/set_position_absolute");
			ptu::SetPositionAbsolute srv;
			srv.request.pan = pan;
			srv.request.tilt = tilt;

			if(client.call(srv))
			{
				ROS_INFO("success: %d", srv.response.success);
			}
			else
			{
				ROS_ERROR("Failed to call service ptu::set_position_absolute");
				return 1;
			}
		}
		else if(std::string(argv[1]).compare("rel") == 0)
		{
			ros::ServiceClient client = n.serviceClient<ptu::SetPositionRelative>("ptu/set_position_relative");
			ptu::SetPositionRelative srv;
			srv.request.pan = pan;
			srv.request.tilt = tilt;

			if(client.call(srv))
			{
				ROS_INFO("success: %d", srv.response.success);
			}
			else
			{
				ROS_ERROR("Failed to call service ptu::set_position_relative");
				return 1;
			}
		}
		else
		{
			ROS_ERROR("couldn't parse \"abs\" or \"rel\" from \"%s\"", argv[1]);
			return 1;
		}
	}
	else
	{
		ROS_INFO("usage for getting: ptu_client get");
		ROS_INFO("usage for setting: ptu_client rel|abs float_pan_val_in_deg float_tilt_val_in_deg");
		ROS_INFO("             e.g.: ptu_client abs 20 -10");
		return 1;
	}

	return 0;
}