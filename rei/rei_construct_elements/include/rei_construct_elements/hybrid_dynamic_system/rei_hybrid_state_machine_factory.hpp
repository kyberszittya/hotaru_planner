/*
 * rei_hybrid_state_machine_factory.hpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_HYBRID_DYNAMIC_SYSTEM_REI_HYBRID_STATE_MACHINE_FACTORY_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_HYBRID_DYNAMIC_SYSTEM_REI_HYBRID_STATE_MACHINE_FACTORY_HPP_

#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>

namespace rei
{


namespace node
{

template<class Timestamp, class Clock> class HybridStateMachineFactory
{
private:
	unsigned int cnt_event;
	std::map<std::string, unsigned int> event_mapping;
protected:

	HybridStateMachineFactory(): cnt_event(0)
	{

	}

public:

	HybridStateMachineFactory(HybridStateMachineFactory& ) = delete;
	HybridStateMachineFactory& operator=(HybridStateMachineFactory& ) = delete;

	static HybridStateMachineFactory* getInstance()
	{
		static HybridStateMachineFactory instance;
		return &instance;
	}

	std::map<std::string, unsigned int> getEventMapping()
	{
		return event_mapping;
	}

	/*
	 * @param: sm_delta_time double: delta time between state machines
	 */
	HybridStateMachinePtr<Timestamp, Clock> createHybridStateMachine(
			const std::string name, const double sm_delta_time, std::shared_ptr<Clock> sm_clock)
	{
		HybridStateMachinePtr<Timestamp, Clock> hy =
			std::make_shared<HybridStateMachine<Timestamp, Clock>>(name, sm_delta_time);
		addLocations(*hy, {"PSEUDO_START", "PSEUDO_END"});
		hy->setClock(sm_clock);
		hy->initialize();
		return hy;
	}

	void addLocations(HybridStateMachine<Timestamp, Clock>& sm, const std::initializer_list<std::string>& location_labels)
	{
		for (const auto s: location_labels)
		{
			LocationPtr l = std::make_shared<Location>(s, sm.getNumberOfLocations());
			sm.addState(std::move(l));
		}
	}

	/*
	 *
	 */
	void setTerminalLocations(
			std::shared_ptr<rei::node::HybridStateMachine<Timestamp, Clock>> hy,
			const std::string& start_event,
			const std::string& start_target_location,
			const std::string& end_event,
			const std::string& end_source_location
			)
	{
		addDiscreteTransition(*hy, start_event,
				std::pair<std::string, std::string>("PSEUDO_START", start_target_location));
		addDiscreteTransition(*hy, end_event,
				std::pair<std::string, std::string>(end_source_location, "PSEUDO_END"));
	}

	void addDiscreteTransition(
			HybridStateMachine<Timestamp, Clock>& sm,
			std::string event_label, std::pair<std::string, std::string> transit_tuple)
	{
		if (event_mapping.find(event_label)==event_mapping.end())
		{
			event_mapping.insert(std::pair<std::string, unsigned int>(event_label, cnt_event++));
		}
		sm.addTransition(event_mapping[event_label], transit_tuple.first, transit_tuple.second);
	}

	/*
	 * @brief: Add transitions stored in an initializer list
	 * @param sm: Transitions shall be added to this hybrid state machine
	 * @param transition_list: The list which holds a tuple of transition descriptions, in the following order:
	 * 			<event_label, source, target>
	 */
	void addDiscreteTransitions(
		HybridStateMachine<Timestamp, Clock>& sm,
		std::initializer_list<std::tuple<std::string, std::string, std::string>> transition_list)
	{
		for (const auto& t: transition_list)
		{
			if (event_mapping.find(std::get<0>(t))==event_mapping.end())
			{
				event_mapping.insert(std::pair<std::string, unsigned int>(std::get<0>(t), cnt_event++));
			}
			sm.addTransition(event_mapping[std::get<0>(t)], std::get<1>(t), std::get<2>(t));
		}
	}

	/*
	 * @brief: Assign transition guard function
	 */
	void assignTransitionGuardFunction(HybridStateMachine<Timestamp, Clock>& sm,
			const std::string& source, const std::string& target,
			std::function<bool()> func)
	{
		sm.addEdgeGuardDefinition(source+"->"+target, func);
	}

	/*
	 * @brief: Assign transition guard function
	 */
	void assignTransitionTransitFunction(HybridStateMachine<Timestamp, Clock>& sm,
			const std::string& source, const std::string& target,
			std::function<void()> func)
	{
		sm.addOnTransitFunction(source+"->"+target, func);
	}
};


}  // namespace node

}  // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_HYBRID_DYNAMIC_SYSTEM_REI_HYBRID_STATE_MACHINE_FACTORY_HPP_ */
