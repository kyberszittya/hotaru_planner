/*
 * lib_rei_robot_configurator.cpp
 *
 *  Created on: Jun 28, 2020
 *      Author: kyberszittya
 */

#include "rei_robot_configurator/rei_robot_configuration.hpp"

namespace rei
{

namespace config
{

// SECTION
// RobotMotionModel

RobotMotionModel::RobotMotionModel(const std::string name, const std::string typen): name(name), typen(typen)
{
	kinematic_parameters = std::make_shared<RobotParameters>();
	dynamic_parameters = std::make_shared<RobotParameters>();
}

RobotMotionModel::~RobotMotionModel(){}

const std::string RobotMotionModel::getName() const
{
	return name;
}

void RobotMotionModel::initialize()
{
	setupKinematicParameters();
	setupDynamicParameters();
}

const std::string RobotMotionModel::getTypeText() const
{
	return typen;
}

const SharedRobotParameters RobotMotionModel::getKinematicParameters() const
{
	return kinematic_parameters;
}



// SECTION
// VehicleMotionModel
VehicleMotionModel::VehicleMotionModel(const std::string name, const parameters::VehicleKinematicParameters& kin_param):
		RobotMotionModel(name, "ackermann_vehicle"),
		struct_kin_veh_p(kin_param){}

void VehicleMotionModel::setupKinematicParameters()
{
	// Set kinematic parameters
	kinematic_parameters->push_back(struct_kin_veh_p.wheelbase);
	kinematic_parameters->push_back(struct_kin_veh_p.front_track_width);
	kinematic_parameters->push_back(struct_kin_veh_p.rear_track_width);
	kinematic_parameters->push_back(struct_kin_veh_p.wheel_radius);
}

void VehicleMotionModel::setupDynamicParameters()
{

}


}

}
