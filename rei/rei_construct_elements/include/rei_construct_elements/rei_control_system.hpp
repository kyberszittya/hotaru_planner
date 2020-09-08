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
	std::vector<rei::node::NotificationContextPtr<Timestamp>> notification_context;
	std::vector<rei::node::HybridStateMachine<Timestamp, Clock>> hysm;

	std::shared_ptr<Clock> clock;
public:

	void addNotificationContext(rei::node::NotificationContextPtr context)
	{
		notification_context.push_back(context);
	}

	void addHybridSystem(rei::node::HybridStateMachine hysm)
	{
		notification_context.push_back(hysm);
	}

	void setClock(std::shared_ptr<Clock> clock)
	{
		this->clock = clock;
		for (auto hy: hysm)
		{
			hy->setClock(clock);
		}
	}
};

}

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_CONTROL_SYSTEM_HPP_ */
