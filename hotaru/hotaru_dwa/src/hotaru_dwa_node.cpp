/*
 * hotaru_dwa_node.cpp
 *
 *  Created on: Jun 9, 2020
 *      Author: kyberszittya
 */


#include <hotaru_node_elements/hotaru_planner_node_ros1.hpp>
#include <memory>

namespace hotaru
{

class RobotStaticSpecification
{
private:
	const double max_accel;
	const double max_theta_accel;
protected:
public:
	RobotStaticSpecification(const double max_accel,
			const double max_theta_accel):
				max_accel(max_accel),
				max_theta_accel(max_theta_accel){}

};

class RobotDynamicConstraints
{
private:
	double min_speed;
	double max_speed;
	double min_angvel;
	double max_angvel;
public:
	RobotDynamicConstraints(const double min_speed,
			const double max_speed,
			const double min_angvel,
			const double max_angvel):
				min_speed(min_speed),
				max_speed(max_speed),
				min_angvel(min_angvel),
				max_angvel(max_angvel)
				{}

	const double getMinSpeed(){return min_speed;}
	const double getMaxSpeed(){return max_speed;}
};

class VehicleStaticSpecification: public RobotStaticSpecification
{
private:
	const double wheelbase;
	const double trackwidth;
public:
	VehicleStaticSpecification(
			const double max_accel,
			const double max_theta_accel,
			const double wheelbase,
			const double trackwidth):
				RobotStaticSpecification(max_accel, max_theta_accel),
				wheelbase(wheelbase),
				trackwidth(trackwidth)
			{}

	const double getWheelBase()
	{
		return wheelbase;
	}

	const double getTrackwidth()
	{
		return trackwidth;
	}
};

struct Window
{
	const double xmin;
	const double xmax;
	const double ymin;
	const double ymax;

	Window(const double xmin, const double xmax,
			const double ymin, const double ymax):
		xmin(xmin), xmax(xmax),
		ymin(ymin), ymax(ymax){}
};

const Window operator*(const Window& w0, const Window& w1)
{
	Window res(std::max(w0.xmin, w1.xmin),
			std::min(w0.xmax, w1.xmax),
			std::max(w0.ymin, w1.ymin),
			std::max(w0.ymax, w1.ymax));
	return res;
}

struct RobotState2D
{
	double x;
	double y;
	double yaw;
};

class DynamicWindowApproach
{
private:
	double sample_time;
	//std::shared_ptr<VehicleStaticSpecification> static_specification;
	//std::shared_ptr<RobotDynamicConstraints> dynamic_constraints;

	// Use the article by Kong et al for motion model definition
	void motion(std::shared_ptr<RobotState2D> state, const double dt)
	{

	}

	// This window is calculated based on the static specification
	//     [ linvel ]
	// u = [ angvel ]
	Window& calcMotionWindow(const double linvel, const double angvel)
	{
		Window w(linvel - static_specification->max_accel*sample_time,
				 linvel + static_specification->max_accel*sample_time,
				 angvel - static_specification->max_theta_accel*sample_time,
				 angvel + static_specification->max_theta_accel*sample_time);
		return std::move(w);
	}

	Window& calcSpecificationWindow()
	{
		Window w(dynamic_constraints->min_speed, dynamic_constraints->max_speed,
				dynamic_constraints->min_angvel, dynamic_constraints->max_angvel);
		return std::move(w);
	}

	void calcWindow(const double linvel, const double angvel)
	{
		calcMotionWindow(linvel, angvel)*calcSpecificationWindow();
	}

	void predict()
	{

	}
public:
	DynamicWindowApproach(
			std::shared_ptr<VehicleStaticSpecification> static_specification,
			std::shared_ptr<RobotDynamicConstraints> dynamic_constraints,
			double sample_time):
		static_specification(static_specification),
		dynamic_constraints(dynamic_constraints),
		sample_time(sample_time)
	{

	}


};

class HotaruPlannerDwa: public HotaruPlannerNodeRos
{
protected:


public:

};

}


int main(int argc, char** argv)
{
	return 0;
}
