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

#include <rei_construct_elements/rei_hybrid_state_exceptions.hpp>
#include <rei_construct_elements/graph_elements/rei_graph.hpp>

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

class Transition: public rei::graph::Edge<unsigned long, double>
{
protected:
	// Lambda as guard
	std::function<bool()> guard_def;
	// Transit functional
	std::function<void()> transit_function;
public:
	//
	Transition(const unsigned long event_id,
			graph::VertexPtr<unsigned long, double> source_location,
			graph::VertexPtr<unsigned long, double> target_location):
			graph::Edge<unsigned long, double>(
					event_id, 0.0, source_location, target_location, EdgeDirectionality::DIRECTED)
			{
				if (source_location!=nullptr && target_location!=nullptr)
				{
					label = source_location->getLabel()+"->"+target_location->getLabel();
				}
				else
				{
					label = "";
				}
				guard_def = []()->bool{return true;};
				transit_function = [](){;};
			}


	/*
	 * brief: Retrieve source location
	 */
	const LocationPtr getSource()
	{
		return std::dynamic_pointer_cast<Location>(source_vertex);
	}

	const LocationPtr getTarget()
	{
		return std::dynamic_pointer_cast<Location>(target_vertex);
	}

	/*
	 * @brief: Evaluate guard function
	 */
	bool checkDiscreteGuard()
	{
		return guard_def();
	}

	/*
	 * @brief: Execute associated function
	 */
	void onTransit()
	{
		transit_function();
	}


	/*
	 * @brief: get label
	 */
	const std::string getLabel() const
	{
		return label;
	}

	/*
	 * @brief: add guard definition
	 */
	void setGuardDefinition(std::function<bool()> func)
	{
		guard_def = func;
	}

	/*
	 * @brief: add transit function
	 */
	void setOnTransitFunction(std::function<void()> func)
	{
		transit_function = func;
	}
};



typedef std::shared_ptr<Transition> TransitionPtr;
typedef std::weak_ptr<Transition> WeakTransitionPtr;

typedef const std::shared_ptr<Transition> ConstTransitionPtr;

class Location: public rei::graph::Vertex<unsigned long, double>
{
private:
	// Function to execute (equiv. 'do' semantic)
	std::function<void()> func_do;
protected:
public:
	Location(const std::string& label, unsigned int location_number):
		rei::graph::Vertex<unsigned long, double>(label, location_number)
	{
		func_do = [](){;};
	}

	void setDoFunction(std::function<void()> func)
	{
		func_do = func;
	}
};

enum class HybridStateStepResult{ TRANSITED, PROCESS_TIMEOUT, SHIFTED_TIMESTAMP, EMPTY_QUEUE, NO_TRANSITION };

template<class Timestamp> class NotificationEvent
{
private:
	const std::string event_name;
	const Timestamp stamp;
public:
	NotificationEvent(std::string name, const Timestamp stamp): event_name(name),
		stamp(stamp)
	{}

	const std::string getEventName() const
	{
		return event_name;
	}

	const Timestamp getTimestamp() const
	{
		return stamp;
	}
};

template<class Timestamp> using NotificationEventPtr = std::shared_ptr<NotificationEvent<Timestamp>>;

template<class Timestamp> class NotificationContext
{
private:
	std::stack<NotificationEventPtr<Timestamp>> stack_event;
	std::stack<NotificationEventPtr<Timestamp>> stack_location;
	std::stack<NotificationEventPtr<Timestamp>> stack_transition;
public:
	virtual ~NotificationContext()
	{}
	virtual void notifyLocation(const LocationPtr location, Timestamp stamp)
	{
		std::shared_ptr<NotificationEvent<Timestamp>> event =
			std::make_shared<NotificationEvent<Timestamp>>(location->getLabel(), stamp);
		stack_location.push(std::move(event));

	}

	virtual void notifyTransition(const TransitionPtr transition, Timestamp stamp)
	{
		std::shared_ptr<NotificationEvent<Timestamp>> event =
			std::make_shared<NotificationEvent<Timestamp>>(transition->getLabel(), stamp);
		stack_transition.push(std::move(event));
	}

	NotificationEventPtr<Timestamp> popEvent()
	{
		if (!stack_event.empty())
		{
			NotificationEventPtr<Timestamp> event = stack_event.top();
			stack_event.pop();
			return event;
		}
		return nullptr;
	}

	NotificationEventPtr<Timestamp> popTransitionEvent()
	{
		if (!stack_transition.empty())
		{
			NotificationEventPtr<Timestamp> event = stack_transition.top();
			stack_transition.pop();
			return event;
		}
		return nullptr;
	}

	NotificationEventPtr<Timestamp> popLocationEvent()
	{
		if (!stack_location.empty())
		{
			NotificationEventPtr<Timestamp> event = stack_location.top();
			stack_location.pop();
			return event;
		}
		return nullptr;
	}

};

template<class Timestamp> using NotificationContextPtr = std::shared_ptr<NotificationContext<Timestamp>>;

template<class Timestamp, class Clock> class HybridStateMachine: public rei::graph::Graph<unsigned long, double>
{
protected:
	// Delta timestamp
	Timestamp timestamp_delta;									///< Timestamp delta to check if event should be processed anyway
	// Notification context
	NotificationContextPtr<Timestamp> notification_context;
	// Location handling
	// Current state
	LocationPtr current_location;								///< Current location of the state machine
	// Event queue
	std::stack<std::shared_ptr<DiscreteEvent<Timestamp>>> event_queue;
	/// Clock
	std::shared_ptr<util::ClockInterface<Timestamp>> sm_clock;
public:
	/*
	 * @param: delta_timestamp double: the allowed delta between incoming timestamps
	 */
	HybridStateMachine(const std::string name, double timestamp_delta):
		rei::graph::Graph<unsigned long, double>(name),
		timestamp_delta(timestamp_delta),
		current_location(nullptr){}

	~HybridStateMachine()
	{
		sm_clock.reset();
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
	 * @brief: Delay of state variable in current location.
	 */
	void delayStateVar()
	{
		// TODO: implement as well with continuous state definition
	}

	/*
	 *
	 */
	void setNotificationContext(NotificationContextPtr<Timestamp> notification_context)
	{
		this->notification_context = notification_context;
	}

	/*
	 * @brief: Get number of locations
	 */
	inline const unsigned int getNumberOfLocations() const
	{
		return number_of_nodes;
	}

	/*
	 * @brief: get current location
	 */
	inline ConstLocationPtr getCurrentLocation() const
	{
		return std::dynamic_pointer_cast<Location>(current_location);
	}

	/*
	 * @brief: get location based on label
	 */
	inline ConstLocationPtr getLocationByLabel(const std::string& label)
	{
		return std::dynamic_pointer_cast<Location>(label_to_node[label]);
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
		current_location = std::dynamic_pointer_cast<Location>(vertices[0]);
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
		if (sm_clock->getCurrentTime() + timestamp_delta < event->getTimeStamp())
		{
			return HybridStateStepResult::SHIFTED_TIMESTAMP;
		}
		else if (abs(static_cast<long>(sm_clock->getCurrentTime() - event->getTimeStamp())) > timestamp_delta)
		{
			return HybridStateStepResult::PROCESS_TIMEOUT;
		}
		// If everything is OK, try to transit!
		HybridStateStepResult res = HybridStateStepResult::TRANSITED;
		try
		{
			TransitionPtr tr = std::dynamic_pointer_cast<Transition>(
					current_location->getOutgoingEdge(event->getEventId()).lock());
			// If no transition is available, do nothing
			if (tr->checkDiscreteGuard())
			{
				current_location = tr->getTarget();
				if (notification_context!=nullptr)
				{
					notification_context->notifyTransition(tr, sm_clock->getCurrentTime());
					notification_context->notifyLocation(current_location, sm_clock->getCurrentTime());
				}
			}
			else
			{
				res = HybridStateStepResult::NO_TRANSITION;
			}
			// TODO: handle guard definitions

		}
		catch(graph::exceptions::NonexistentEdge& e)
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
		addVertex(std::move(location));
	}



	/*
	 * @brief: Add transition to the state machine
	 */
	void addTransition(unsigned int event_id,
			const std::string& source, const std::string& target)
	{
		TransitionPtr edge = std::make_shared<Transition>(
			event_id, label_to_node[source], label_to_node[target]
		);
		addEdge(edge, event_id);
	}

	/*
	 * @brief: add a guard definition to edge
	 */
	void addEdgeGuardDefinition(std::string label, std::function<bool()> function)
	{
		TransitionPtr tr = std::dynamic_pointer_cast<Transition>(label_to_edge[label]);
		if (tr!=nullptr) tr->setGuardDefinition(function);
	}

	void addOnTransitFunction(std::string label, std::function<void()> function)
	{
		TransitionPtr tr = std::dynamic_pointer_cast<Transition>(label_to_edge[label]);
		if (tr!=nullptr) tr->setOnTransitFunction(function);
	}

}; // class HybridStateMachine

template<class Timestamp, class Clock> using HybridStateMachinePtr = std::shared_ptr<HybridStateMachine<Timestamp, Clock> >;


template<class Timestamp, class Clock> class DiscreteEventPipeline
{
protected:
	std::vector<std::shared_ptr<HybridStateMachine<Timestamp, Clock>>> state_machines;
	std::map<std::string, unsigned int> event_mapping;
public:
	void addStateMachine(std::shared_ptr<HybridStateMachine<Timestamp, Clock>> sm)
	{
		state_machines.emplace_back(sm);
	}

	void propagateEvent(const std::string& event_label, unsigned int event_id, Timestamp stamp)
	{
		std::shared_ptr<DiscreteEvent<Timestamp>> new_event =
				std::make_shared<DiscreteEvent<Timestamp>>(event_label, event_id, stamp);
		for (const auto& sm: state_machines)
		{
			sm->addEvent(new_event);
		}
	}

	/*
	 * @brief: Set event mapping from event label to event id
	 */
	void setEventMapping(std::map<std::string, unsigned int> event_map)
	{
		event_mapping = event_map;
	}

	/*
	 * @brief: Proapgate event based on internal signal map
	 */
	void propagateEvent(const std::string& event_label, Timestamp stamp)
	{
		auto it = event_mapping.find(event_label);
		if (it!=event_mapping.end()) propagateEvent(event_label, it->second, stamp);
		else throw rei::exceptions::UnknownEvent();
	}


};

template<class Timestamp, class Clock> using DiscreteEventPipelinePtr = std::shared_ptr<DiscreteEventPipeline<Timestamp, Clock>>;


} // namespace node

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_MACHINE_HPP_ */
