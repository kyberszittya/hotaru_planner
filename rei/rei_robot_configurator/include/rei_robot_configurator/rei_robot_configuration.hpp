/*
 * rei_robot_configuration.hpp
 *
 *  Created on: Jun 28, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_ROBOT_CONFIGURATOR_REI_ROBOT_CONFIGURATION_HPP_
#define INCLUDE_REI_ROBOT_CONFIGURATOR_REI_ROBOT_CONFIGURATION_HPP_

#include "parameter_structs.hpp"

#include <array>
#include <vector>
#include <memory>

namespace rei
{
namespace config
{

typedef std::vector<double> RobotParameters;
typedef std::shared_ptr<RobotParameters> SharedRobotParameters;

// Interface to all robots
class RobotMotionModel
{
private:

protected:
	const std::string typen;
	SharedRobotParameters kinematic_parameters;
	SharedRobotParameters dynamic_parameters;
	const std::string name;

	virtual void setupKinematicParameters() = 0;
	virtual void setupDynamicParameters() = 0;
public:
	RobotMotionModel(const std::string name, const std::string typen);

	virtual ~RobotMotionModel() = 0;
	void initialize();

	const SharedRobotParameters getKinematicParameters() const;

	const std::string getTypeText() const;
	const std::string getName() const;

};

typedef std::unique_ptr<RobotMotionModel> MotionModelUniquePtr;


class VehicleMotionModel: public RobotMotionModel
{
private:
	parameters::VehicleKinematicParameters struct_kin_veh_p;
protected:
	virtual void setupKinematicParameters() override;
	virtual void setupDynamicParameters() override;
public:
	VehicleMotionModel(const std::string name, const parameters::VehicleKinematicParameters& kin_param);

};

}

}

#endif /* INCLUDE_REI_ROBOT_CONFIGURATOR_REI_ROBOT_CONFIGURATION_HPP_ */
