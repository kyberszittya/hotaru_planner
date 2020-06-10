/*
 * gen_dynamicwindow_node.hpp
 *
 *  Created on: Jun 10, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_PERCEPTION_NODES_DYNAMICWINDOW_NODE_HPP_
#define INCLUDE_REI_PERCEPTION_NODES_DYNAMICWINDOW_NODE_HPP_

#include "gen_dynamickinematicwindow.hpp"

#include <grid_map_ros/grid_map_ros.hpp>
#include <rei_common/geometry_utils/window.hpp>

namespace rei
{

class DynamicWindowNode: public InterfaceRos_DynamicKinematicWindow
{
private:
protected:
	grid_map::GridMap map;
	ros::Timer updateTimer;
	// Window definitions
	geometry::Window motion_window;
	geometry::Window specification_window;
public:
	virtual void executeUpdatevehiclestatus(const autoware_msgs::VehicleStatus::ConstPtr& msg) override
	{

	}

	virtual bool initPre() override
	{
		grid_map::GridMap map({"dynamic_window"});
		map.setFrameId("base_link");
		return true;
	}

	virtual bool initPost() override
	{
		updateTimer = nh->createTimer(ros::Duration(0.1), &DynamicWindowNode::cbTimerUpdateWindow, this);
		return true;
	}

	geometry::Window& calcMotionWindow(const double linvel, const double angvel, const double dt)
	{
		geometry::Window w(
			linvel - 2.0*dt,
			linvel + 2.0*dt,
			angvel - 0.5*dt,
			angvel + 0.5*dt
		);
		return std::move(w);
	}

	geometry::Window& calcSpecificationWindow()
	{
		geometry::Window w(
			0, // MIN_SPEED
			25, // MAX SPEED
			-2.5,
			2.5
		);
		return std::move(w);

	}

	void cbTimerUpdateWindow(const ros::TimerEvent& e)
	{
		using namespace grid_map;
		const double dt = 0.1; // TODO: timer based
		geometry::Window w = calcSpecificationWindow()*
			calcMotionWindow(pubsubstate->msg_port_vehicle_status.speed,
				pubsubstate->msg_port_vehicle_status.angle, dt
		);
		map.setGeometry(Length(w.xmax-w.xmin, w.ymax - w.ymin), 0.05);
		GridMapRosConverter::toMessage(map, pubsubstate->msg_port_grid_map);
		publishDynamic_window();
	}
};

}

#endif /* INCLUDE_REI_PERCEPTION_NODES_GEN_DYNAMICWINDOW_NODE_HPP_ */
