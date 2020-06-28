/*
 * motion_parameters.hpp
 *
 *  Created on: Jun 28, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_ROBOT_CONFIGURATOR_MOTION_PARAMETERS_HPP_
#define INCLUDE_REI_ROBOT_CONFIGURATOR_MOTION_PARAMETERS_HPP_


namespace rei
{
namespace config
{
namespace parameters
{

struct VehicleKinematicParameters
{
	// Car kinematic parameters
	const double wheelbase;
	const double front_track_width;
	const double rear_track_width;
	// Wheel radius
	const double wheel_radius;

	VehicleKinematicParameters(
		const double wheelbase,
		const double front_track_width,
		const double rear_track_width,
		const double wheel_radius
		): wheelbase(wheelbase),
		   front_track_width(front_track_width),
		   rear_track_width(rear_track_width),
		   wheel_radius(wheel_radius)
			{}
};

struct MotionDynamicConstraints
{
	double max_linvel;
	double min_linvel;
	double max_angvel;
	double min_angvel;
};

}
}
}

#endif /* INCLUDE_REI_ROBOT_CONFIGURATOR_MOTION_PARAMETERS_HPP_ */
