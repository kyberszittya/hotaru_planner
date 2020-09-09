/*
 * rei_control_system.hpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_CONTROL_SYSTEM_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_CONTROL_SYSTEM_HPP_


#include <vector>

#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>
#include <rei_construct_elements/rei_hybrid_control_node.hpp>

namespace rei
{

namespace hybridsystem
{

/*
 * The hybrid dynamic state control block
 * @input: external clock is required
 */
template<class Timestamp, class Clock> class HybridControl
{
protected:
	rei::node::DiscreteEventPipelinePtr<Timestamp, Clock> event_pipeline;
	rei::node::NotificationContextPtr<Timestamp> notification_context;
	rei::node::HybridStateMachinePtr<Timestamp, Clock> hysm;

	std::shared_ptr<Clock> clock;
public:

	void setNotificationContext(rei::node::NotificationContextPtr<Timestamp> context)
	{
		this->notification_context = context;
		hysm->setNotificationContext(notification_context);
	}

	void setHybridSystem(rei::node::HybridStateMachinePtr<Timestamp, Clock> hysm)
	{
		this->hysm = hysm;
	}

	void setDiscreteEventPipeline(rei::node::DiscreteEventPipelinePtr<Timestamp, Clock> pipeline)
	{
		event_pipeline = pipeline;
		event_pipeline->addStateMachine(this->hysm);
	}

	void setClock(std::shared_ptr<Clock> clock)
	{
		this->clock = clock;
		hysm->setClock(clock);
	}

	const std::string getName() const
	{
		return hysm->getName();
	}
};

}

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_CONTROL_SYSTEM_HPP_ */
