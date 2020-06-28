/*
 * node_vehicle_conf.cpp
 *
 *  Created on: Jun 28, 2020
 *      Author: kyberszittya
 */


#include "rei_robot_configurator/rei_robot_configurator_ros.hpp"
#include "rei_robot_configurator/parameter_structs.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_conifgurator");
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	std::shared_ptr<ros::NodeHandle> private_nh = std::make_shared<ros::NodeHandle>("~");
	using namespace rei::config;
	RobotMotionParameterProvider_ROS ros_configurator(private_nh, nh);
	std::unique_ptr<RobotMotionModel> motion_model;
	//
	std::string name;
	if (!private_nh->getParam("robot/name", name))
	{
		ROS_ERROR("No robot name specified");
		return -1;
	}
	std::string type_robot;
	if (!private_nh->getParam("robot/type", type_robot))
	{
		ROS_ERROR("No robot type specified");
		return -1;
	}
	if (type_robot=="ackermann")
	{
		ROS_INFO("Ackermann vehicle specified");
		double wheelbase;
		if (!private_nh->getParam("robot/kinematic_parameters/wheelbase", wheelbase))
		{
			ROS_ERROR("No wheelbase specified");
			return -1;
		}
		double front_track;
		if (!private_nh->getParam("robot/kinematic_parameters/front_track", front_track))
		{
			ROS_ERROR("No front track specified");
			return -1;
		}
		double rear_track;
		if (!private_nh->getParam("robot/kinematic_parameters/rear_track", rear_track))
		{
			ROS_ERROR("No rear track sepcified");
			return -1;
		}
		double wheel_radius;
		if (!private_nh->getParam("robot/wheelparameters/wheel_radius", wheel_radius))
		{
			ROS_ERROR("No wheel radius specified");
			return -1;
		}
		rei::config::parameters::VehicleKinematicParameters s(wheelbase, front_track, rear_track, wheel_radius);

		motion_model = std::unique_ptr<VehicleMotionModel>(new VehicleMotionModel(name, s));
		motion_model->initialize();
	}

	//

	ros_configurator.insertMotionModel(name, motion_model);
	ros_configurator.init();
	ros::spin();

	return 0;
}

