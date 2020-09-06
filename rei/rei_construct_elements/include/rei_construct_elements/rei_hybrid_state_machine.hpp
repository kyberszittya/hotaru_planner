/*
 * rei_hybrid_state_machine.hpp
 *
 *  Created on: Sep 3, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_MACHINE_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_MACHINE_HPP_

#include <map>
#include <memory>
#include <list>
#include <vector>
#include <functional>
#include <stack>

namespace rei
{

namespace node
{



template<class Timestamp> class DiscreteEvent
{
private:
	const Timestamp stamp;
	const std::string event_label;
	const unsigned int event_id;
protected:

public:
	DiscreteEvent(const std::string event_label, const unsigned int event_id, const Timestamp stamp):
		stamp(stamp), event_label(event_label), event_id(event_id){}

	/*
	 * @brief: get timestamp of the event
	 */
	const Timestamp getStamp()
	{
		return stamp;
	}

	const unsigned int getEventId()
	{
		return event_id;
	}
};

class Location;
typedef std::shared_ptr<Location> LocationPtr;
typedef const std::shared_ptr<Location> ConstLocationPtr;

class Transition
{
protected:
	// As transition is modeled as a graph edge
	const unsigned int event_id;
	const ConstLocationPtr source_location;   // Define source location
	const ConstLocationPtr target_location;   // Define target location
	// Lambda as guard
	//std::function<bool> guard_def;
public:
	//
	Transition(const unsigned int event_id,
			ConstLocationPtr source_location, ConstLocationPtr target_location):
		event_id(event_id),
		source_location(source_location), target_location(target_location){}
	/*
	 * brief: Retrieve source location
	 */
	const ConstLocationPtr getSource()
	{
		return source_location;
	}

	const ConstLocationPtr getTarget()
	{
		return target_location;
	}

	/*
	 * @brief: Evaluate guard function
	 */
	bool checkDiscreteGuard()
	{
		//return guard_def();
		return true;
	}
};



typedef std::shared_ptr<Transition> TransitionPtr;
typedef const std::shared_ptr<Transition> ConstTransitionPtr;

class Location
{
protected:
	unsigned int location_number;
	const std::string label;
	// Event transition map
	std::map<unsigned int, TransitionPtr> transition_map;
public:
	Location(const std::string& label, unsigned int location_number):
		label(label), location_number(location_number){}

	inline const std::string getLabel() const
	{
		return label;
	}

	inline const unsigned int getLocationNumber() const
	{
		return location_number;
	}

	/*
	 * Add transition to location
	 *
	 * NOTE: multiple transitions on the same event are not accepted
	 */
	void addTransition(const unsigned int event_id, TransitionPtr transition)
	{
		transition_map.insert(std::pair<unsigned int, TransitionPtr>(
				event_id, transition));
	}

	ConstTransitionPtr getNextTransition(const unsigned int event_id)
	{
		return transition_map[event_id];
	}

};



template<class Timestamp, class Clock> class HybridStateMachine
{
protected:
	// Location handling
	unsigned int number_of_locations;
	// State vector
	std::vector<LocationPtr> locations;
	// Current state
	LocationPtr current_location;
	// Map labels to location
	std::map<std::string, ConstLocationPtr> label_to_location;
	// Store transitions as well
	std::vector<TransitionPtr> transitions;
	// Event queue
	std::stack<std::shared_ptr<DiscreteEvent<Timestamp>>> event_queue;
public:
	HybridStateMachine(): number_of_locations(0), current_location(nullptr){}


	/*
	 * @brief: Get number of locations
	 */
	inline const unsigned int getNumberOfLocations() const
	{
		return number_of_locations;
	}

	/*
	 * @brief: get current location
	 */
	inline ConstLocationPtr getCurrentLocation() const
	{
		return current_location;
	}

	/*
	 * @brief: get location based on label
	 */
	inline ConstLocationPtr getLocationByLabel(const std::string& label)
	{
		return label_to_location[label];
	}

	/*
	 * Reset state machine
	 */
	void initialize()
	{
		current_location = locations[0];
	}


	void step()
	{
		// Check if there is any discrete event that occurred
		ConstTransitionPtr tr = current_location->getNextTransition(event_queue.top()->getEventId());
		// If no transition is available, do nothing
		if (tr!=nullptr)
		{
			// Of course, discrete transitions only occur, if the guard property can work
			if (tr->checkDiscreteGuard())
			{
				current_location = tr->getTarget();
			}
			// TODO: handle guard definitions

			// TODO: handle clocks
		}
		event_queue.pop();
	}

	void addEvent(std::shared_ptr<DiscreteEvent<Timestamp>> e)
	{
		event_queue.push(e);
	}

	/*
	 * @brief: Add state explicitly to this state machine
	 */
	void addState(LocationPtr location)
	{
		locations.emplace_back(std::move(location));
		label_to_location.insert(std::pair<std::string, LocationPtr>(
						locations.back()->getLabel(), locations.back()));
		number_of_locations++;
	}

	/*
	 * @brief: Get location labels in a list
	 */
	std::vector<std::string> getLocationLabels()
	{
		std::vector<std::string> labels;
		for (const auto& v: locations)
		{
			labels.emplace_back(v->getLabel());
		}
		return labels;
	}

	/*
	 * @brief: Add transition to the state machine
	 */
	void addTransition(unsigned int event_id,
			const std::string& source, const std::string& target)
	{
		TransitionPtr transition = std::make_shared<Transition>(
			event_id, label_to_location[source], label_to_location[target]
		);
		transitions.push_back(std::move(transition));
		label_to_location[source]->addTransition(event_id, transitions.back());
	}

}; // class HybridStateMachine


template<class Timestamp, class Clock> class DiscreteEventPipeline
{
protected:
	std::vector<std::shared_ptr<HybridStateMachine<Timestamp, Clock>>> state_machines;
public:
	void addStateMachine(std::shared_ptr<HybridStateMachine<Timestamp, Clock>> sm)
	{
		state_machines.emplace_back(sm);
	}

	void propagateEvent(const std::string& event_label, unsigned int event_id, Timestamp stamp)
	{
		// TODO clock implementation
		std::shared_ptr<DiscreteEvent<Timestamp>> new_event =
				std::make_shared<DiscreteEvent<Timestamp>>(event_label, event_id, stamp);
		for (const auto& sm: state_machines)
		{
			sm->addEvent(new_event);
			sm->step();
		}
	}
};

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

	std::shared_ptr<HybridStateMachine<Timestamp, Clock>> createHybridStateMachine()
	{
		std::shared_ptr<HybridStateMachine<Timestamp, Clock>> hy =
			std::make_shared<HybridStateMachine<Timestamp, Clock>>();
		addLocations(*hy, {"PSEUDO_START", "PSEUDO_END"});
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
};


} // namespace node

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_MACHINE_HPP_ */
