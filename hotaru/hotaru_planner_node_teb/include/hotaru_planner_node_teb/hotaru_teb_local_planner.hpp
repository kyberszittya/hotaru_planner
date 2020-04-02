/*
 * hotaru_teb_local_planner.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_LOCAL_PLANNER_HPP_
#define INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_LOCAL_PLANNER_HPP_

#include <hotaru_node_elements/abstract_planner_component.hpp>

#include <teb_local_planner/teb_local_planner_ros.h>

namespace hotaru
{
class HotaruTebLocalPlannerNode: public HotaruPlannerNode
{
private:
protected:
	std::vector<teb_local_planner::ObstaclePtr> obst_vector;
	teb_local_planner::TebConfig tebconfig;
	teb_local_planner::TebVisualizationPtr visual;
	teb_local_planner::PlannerInterfacePtr planner;
	teb_local_planner::RobotFootprintModelPtr robot_model;
	teb_local_planner::ViaPointContainer via_points;
public:
	HotaruTebLocalPlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):
		HotaruPlannerNode(nh, private_nh){}

	virtual void config()
	{
		tebconfig.map_frame = "base_link";
		//
		tebconfig.trajectory.dt_ref = 1.0;
		tebconfig.trajectory.dt_hysteresis = 1.0;
		//

	}

	virtual bool init()
	{
		// Initialize ros publishers
		using namespace teb_local_planner;
		config();
		plannerstate = std::unique_ptr<HotaruPlannerState>(new HotaruPlannerState());
		if (plannerstate == nullptr)
		{
			return false;
		}
		visual = TebVisualizationPtr(new TebVisualization(nh, tebconfig));
		if (visual == nullptr)
		{
			return false;
		}
		robot_model = RobotFootprintModelPtr(new CircularRobotFootprint(1.0));
		planner = PlannerInterfacePtr(new HomotopyClassPlanner(
				tebconfig, &obst_vector, robot_model, visual, &via_points));
		return true;
	}

};

}
#endif /* INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_LOCAL_PLANNER_HPP_ */
