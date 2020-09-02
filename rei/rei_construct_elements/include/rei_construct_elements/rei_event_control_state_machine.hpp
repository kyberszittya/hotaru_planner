/*
 * rei_event_control_state_machine.hpp
 *
 *  Created on: Sep 2, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_EVENT_CONTROL_STATE_MACHINE_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_EVENT_CONTROL_STATE_MACHINE_HPP_

#include <map>

namespace rei
{

namespace node
{

template<class GuardDef, class ...Args> class Transition
{
protected:
public:
	// Check transition validity
	virtual bool check(...Args args) = 0;
};

template<class SignalClass> class HybridStateMachine
{
protected:
	unsigned int current_location;
	unsigned int number_of_locations;
	// Location handling
	// map label to location number and vice-versa
	std::map<unsigned int, std::string> location_to_label_map;
	std::map<std::string, unsigned int> label_to_location_map;
	// transition handling
	std::map<unsigned int, unsigned int> transition_map;
public:
	virtual ~HybridStateMachine() = 0;
	virtual void step(SignalClass s) = 0;


	/*
	 * @brief: Add location to state machine
	 */
	void addState(const std::string& loc_name)
	{
		location_to_label_map.insert(std::pair<unsigned int, std::string>(number_of_locations, loc_name));
		label_to_location_map.insert(std::pair<std::string, unsigned int>(loc_name, number_of_locations));
		number_of_locations++;
	}

	/*
	 * @brief: Add transition from state to another (string version)
	 * @param: location from and to
	 */
	void addTransition(const std::string& location)
	{

	}

}; // class HybridStateMachine

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
