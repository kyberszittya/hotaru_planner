/*
 * rei_robot_configurator_ros.hpp
 *
 *  Created on: Jun 28, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_ROBOT_CONFIGURATOR_REI_ROBOT_CONFIGURATOR_ROS_HPP_
#define INCLUDE_REI_ROBOT_CONFIGURATOR_REI_ROBOT_CONFIGURATOR_ROS_HPP_

#include "rei_robot_configuration.hpp"

#include <rei_robot_configuration_msgs/ConfigRobotParameters.h>
#include <ros/ros.h>

#include <map>

namespace rei
{
namespace config
{

typedef std::pair<std::string, MotionModelUniquePtr> MapMotionModelElement;


class RobotMotionParameterProvider_ROS
{
private:
	std::shared_ptr<ros::NodeHandle> private_nh;
	std::shared_ptr<ros::NodeHandle> nh;
	ros::ServiceServer robot_conf_param_serv;
	std::map<std::string, MotionModelUniquePtr> motion_models;
public:
	RobotMotionParameterProvider_ROS(
			std::shared_ptr<ros::NodeHandle>,
			std::shared_ptr<ros::NodeHandle>);

	bool transmitParameters(rei_robot_configuration_msgs::ConfigRobotParametersRequest &req,
			rei_robot_configuration_msgs::ConfigRobotParametersResponse &resp);

	void init();

	void insertMotionModel(const std::string name, std::unique_ptr<RobotMotionModel>& robot_motion);
};

}
}

#endif /* INCLUDE_REI_ROBOT_CONFIGURATOR_REI_ROBOT_CONFIGURATOR_ROS_HPP_ */
