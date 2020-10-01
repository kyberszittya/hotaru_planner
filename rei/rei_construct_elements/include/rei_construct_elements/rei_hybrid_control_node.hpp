/*
 * rei_hybrid_control_node.hpp
 *
 *  Created on: Sep 2, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_HPP_

#include "hybrid_dynamic_system/rei_hybrid_state_machine.hpp"
#include "rei_control_system.hpp"

#include <memory>

namespace rei
{

namespace node
{

template<class Timestamp, class Clock, class Subscriber, class Publisher> class HybridControlNode:
		public rei::hybridsystem::HybridControl<Timestamp, Clock>
{
protected:
	/// Publisher interface
	/// Notification context
	Publisher pub_controlevent;
	Publisher pub_location_event;
	Publisher pub_transition_event;
	/// Subscriber pipeline
	Subscriber sub_controlsignal_pipeline;
public:
	HybridControlNode<Timestamp, Clock, Subscriber, Publisher>() {}
	virtual ~HybridControlNode() {};

	void startControlReception();

	/**
		 * @fn bool init_control_signal_interface()=0
	 * @brief Initialize control signal interface (receiving state control messages and induce feedback)
	 *
	 * @pre
	 * @post
	 * @return
	 */
	virtual bool init_control_signal_interface() = 0;


};


} // namespace node


} // namespace rei




#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_HPP_ */
