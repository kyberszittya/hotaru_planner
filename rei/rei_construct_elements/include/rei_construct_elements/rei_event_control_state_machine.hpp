/*
 * rei_event_control_state_machine.hpp
 *
 *  Created on: Sep 2, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_EVENT_CONTROL_STATE_MACHINE_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_EVENT_CONTROL_STATE_MACHINE_HPP_

#include <map>
#include <memory>

#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>

namespace rei
{

namespace node
{


template<class ControlEventSignal> class EventControlStateMachine: public HybridStateMachine<ControlEventSignal>
{
private:
protected:
public:
	virtual void step(ControlEventSignal s) override
	{
		switch(s.sig_id) // Transit according incoming control signal
		{
		case 0: //
		{

		}
		}
	}
};


} // namespace node

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_EVENT_CONTROL_STATE_MACHINE_HPP_ */
