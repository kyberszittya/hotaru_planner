/*
 * impl_hotaru_teb_planner.cpp
 *
 *  Created on: May 6, 2020
 *      Author: kyberszittya
 */

#include <hotaru_planner_node_teb/hotaruteblocalplannernode.hpp>

namespace hotaru
{

void HotaruTebLocalPlannerNode::config()
{
	tebconfig.map_frame = tf_planner_state.getBaseFrame();
	genParamConfig();
}

}



