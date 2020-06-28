/*
 * lib_rei_ros_conf.cpp
 *
 *  Created on: Jun 28, 2020
 *      Author: kyberszittya
 */

#include "rei_robot_configurator/rei_robot_configurator_ros.hpp"

namespace rei
{
namespace config
{



RobotMotionParameterProvider_ROS::RobotMotionParameterProvider_ROS(
		std::shared_ptr<ros::NodeHandle> private_nh,
		std::shared_ptr<ros::NodeHandle> nh)
		: private_nh(private_nh), nh(nh)
{

}

bool RobotMotionParameterProvider_ROS::transmitParameters(
		rei_robot_configuration_msgs::ConfigRobotParametersRequest &req,
		rei_robot_configuration_msgs::ConfigRobotParametersResponse &resp)
{
	if (motion_models.find(req.name)!=motion_models.end())
	{
		auto param = motion_models[req.name]->getKinematicParameters();
		resp.type = motion_models[req.name]->getTypeText();
		resp.kinematic_parameters.reserve(param->size());
		resp.kinematic_parameters.insert(
			resp.kinematic_parameters.end(),
			param->begin(),
			param->end()
		);
		return true;
	}
	else
	{
		return false;
	}
}

void RobotMotionParameterProvider_ROS::init()
{
	robot_conf_param_serv = nh->advertiseService(
					"rei_conf/motion_parameters",
					&RobotMotionParameterProvider_ROS::transmitParameters,
					this);
}

void RobotMotionParameterProvider_ROS::insertMotionModel(std::string name, std::unique_ptr<RobotMotionModel>& robot_motion)
{
	ROS_INFO_STREAM("Added motion model " << name);
	motion_models.insert(MapMotionModelElement(name, std::move(robot_motion)));
}


} // END NAMESPACE CONFIG

} // END NAMESPACE REI
