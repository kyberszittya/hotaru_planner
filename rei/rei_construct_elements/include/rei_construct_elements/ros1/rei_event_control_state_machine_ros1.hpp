/*
 * rei_event_control_state_machine_ros1.hpp
 *
 *  Created on: Sep 3, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_EVENT_CONTROL_STATE_MACHINE_ROS1_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_EVENT_CONTROL_STATE_MACHINE_ROS1_HPP_

#include "rei_event_control_state_machine.hpp"

#include <rei_signal_msgs/ReiRuntimeControl.h>

namespace rei
{

namespace node
{

template<> class EventControlStateMachine<rei_signal_msgs::ReiRuntimeControl, ros::>
{
public:

}

} // namespace node

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_EVENT_CONTROL_STATE_MACHINE_ROS1_HPP_ */
