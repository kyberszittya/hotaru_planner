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
#include <cstdlib>

#include "rei_hybrid_state_exceptions.hpp"

namespace rei
{

namespace util
{

template<class Timestamp> class ClockInterface
{
public:
	virtual ~ClockInterface() {};

	virtual Timestamp getCurrentTime() = 0;
};

} // namespace util

namespace node
{

template<class Timestamp> class NotificationContext;

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
	const Timestamp getTimeStamp()
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
typedef std::weak_ptr<Location> WeakLocationPtr;
typedef std::shared_ptr<const Location> ConstLocationPtr;

class Transition
{
protected:
	// As transition is modeled as a graph edge
	const unsigned int event_id;
	LocationPtr source_location;   // Define source location
	LocationPtr target_location;   // Define target location
	// Lambda as guard
	//std::function<bool> guard_def;
public:
	//
	Transition(const unsigned int event_id,
			LocationPtr source_location, LocationPtr target_location):
		event_id(event_id),
		source_location(source_location), target_location(target_location){}

	~Transition()
	{
		source_location.reset();
		target_location.reset();
	}
	/*
	 * brief: Retrieve source location
	 */
	const LocationPtr getSource()
	{
		return source_location;
	}

	const LocationPtr getTarget()
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

	/*
	 * @brief: It wouldn't be a state machine if it would not do anything during tranist, right?
	 */
	void onTransit()
	{

	}
};



typedef std::shared_ptr<Transition> TransitionPtr;
typedef std::weak_ptr<Transition> WeakTransitionPtr;

typedef const std::shared_ptr<Transition> ConstTransitionPtr;

class Location
{
protected:
	unsigned int location_number;
	const std::string label;
	// Event transition map
	std::map<unsigned int, WeakTransitionPtr> transition_map;
public:
	Location(const std::string& label, unsigned int location_number):
		label(label), location_number(location_number){}

	~Location()
	{
		transition_map.erase(transition_map.begin(), transition_map.end());
	}

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
	void addTransition(const unsigned int event_id, WeakTransitionPtr transition)
	{
		transition_map.insert(std::pair<unsigned int, TransitionPtr>(
				event_id, transition));
	}

	WeakTransitionPtr getNextTransition(const unsigned int event_id)
	{
		return transition_map[event_id];
	}

};

enum class HybridStateStepResult{ TRANSITED, PROCESS_TIMEOUT, EMPTY_QUEUE, NO_TRANSITION };

template<class Timestamp, class Clock> class HybridStateMachine
{
protected:
	// Name of the hybrid state machine
	std::string name;											///< Hybrid state machine name
	// Delta timestamp
	Timestamp timestamp_delta;									///< Timestamp delta to check if event should be processed anyway
	// Location handling
	unsigned int number_of_locations;							///< Indicate the number of location (e.g. avoid to call size())
	// State vector
	std::vector<LocationPtr> locations;							///< Store locations in a vector of state pointers
	// Current state
	LocationPtr current_location;								///< Current location of the state machine
	// Map labels to location
	std::map<std::string, LocationPtr> label_to_location;
	// Store transitions as well
	std::vector<TransitionPtr> transitions;
	// Event queue
	std::stack<std::shared_ptr<DiscreteEvent<Timestamp>>> event_queue;
	/// Clock
	std::shared_ptr<util::ClockInterface<Timestamp>> sm_clock;
public:
	/*
	 * @param: delta_timestamp double: the allowed delta between incoming timestamps
	 */
	HybridStateMachine(const std::string name, double timestamp_delta):
		name(name),
		timestamp_delta(timestamp_delta),
		number_of_locations(0), current_location(nullptr){}

	~HybridStateMachine()
	{
		sm_clock.reset();
		label_to_location.erase(label_to_location.begin(), label_to_location.end());
		for (auto loc: locations)
		{
			loc.reset();
		}
		for (auto tr: transitions)
		{
			tr.reset();
		}
	}

	/*
	 * @brief: Set a current clock
	 */
	void setClock(std::shared_ptr<util::ClockInterface<Timestamp>> sm_clock)
	{
		if (sm_clock==nullptr)
		{
			this->sm_clock.reset();
		}
		else
		{
			this->sm_clock = sm_clock;
		}
	}

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
	 * @brief: we can expect that start state is always STATE 0
	 */
	inline bool isStarted()
	{
		return current_location->location_number == 0;
	}

	/*
	 * @brief: we can expect that end state is always STATE 1
	 */
	inline bool isEnded()
	{
		return current_location->location_number == 1;
	}

	/*
	 * Reset state machine IFF the state machine has not started operation
	 */
	void initialize()
	{
		current_location = locations[0];
	}


	HybridStateStepResult step()
	{
		// Exception handling first then all the other stuff
		// Check if clock is available
		if (sm_clock==nullptr)
		{
			throw std::runtime_error("Clock unavailable for HYSM: "+name);
		}
		// Ignore empty event queue
		if (event_queue.empty())
		{
			return HybridStateStepResult::EMPTY_QUEUE;
		}
		// Check if there is any discrete event that occurred
		std::shared_ptr<DiscreteEvent<Timestamp>> event = event_queue.top();
		// Evaluate event timestamp with current time
		if (sm_clock->getCurrentTime() < event->getTimeStamp())
		{
			// Should this branch be evaluated anyway?
		}
		else if (abs(static_cast<long>(sm_clock->getCurrentTime() - event->getTimeStamp())) > timestamp_delta)
		{
			return HybridStateStepResult::PROCESS_TIMEOUT;
		}
		TransitionPtr tr = current_location->getNextTransition(event->getEventId()).lock();
		// If no transition is available, do nothing
		HybridStateStepResult res = HybridStateStepResult::TRANSITED;
		if (tr!=nullptr)
		{
			// Of course, discrete transitions only occur, if the guard property can work
			if (tr->checkDiscreteGuard())
			{
				current_location = tr->getTarget();
			}
			// TODO: handle guard definitions
		}
		else
		{
			res = HybridStateStepResult::NO_TRANSITION;
		}
		event_queue.pop();
		return res;
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

template<class Timestamp> class NotificationContext
{
private:
public:
	virtual ~NotificationContext() {}
	virtual void notifyLocation(const LocationPtr) = 0;
	virtual void notifyEvent(const std::shared_ptr<Timestamp> event) = 0;

	virtual void notifyTransition(const TransitionPtr transition) = 0;

};

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

	/*
	 * @param: sm_delta_time double: delta time between state machines
	 */
	std::shared_ptr<HybridStateMachine<Timestamp, Clock>> createHybridStateMachine(
			const std::string name, const double sm_delta_time, std::shared_ptr<Clock> sm_clock)
	{
		std::shared_ptr<HybridStateMachine<Timestamp, Clock>> hy =
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
