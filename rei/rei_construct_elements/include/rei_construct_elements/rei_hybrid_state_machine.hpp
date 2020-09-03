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
#include <vector>

namespace rei
{

namespace node
{


class Location
{
protected:
	unsigned int location_number;
	std::string label;
public:

	const std::string getLabel() const
	{
		return label;
	}


};

class Transition
{
protected:
	// As transition is modeled as a graph edge
	const Location* source_location;   // Define source location
	const Location* target_location;   // Define target location
	// Lambda as guard
	std::function<bool> guard_def;
public:
	virtual ~Transition() = 0;
	//

	/*
	 * brief: Retrieve source location
	 */
	const Location* getSource()
	{
		return source_location;
	}

	/*
	 * @brief: Evaluate guard function
	 */
	bool checkDiscreteGuard()
	{
		return guard_def();
	}
};

template<class SignalClass> class HybridStateMachine
{
protected:
	// Location handling
	unsigned int number_of_locations;
	// State vector
	std::vector<Location> locations;
	// Map labels to location
	std::map<std::string, Location*> label_to_location;
	// Store transitions as well
	std::vector<Transition> transitions;
	std::map<Location*, Transition*> transition_map;
public:
	virtual ~HybridStateMachine() = 0;
	virtual void step(SignalClass s) = 0;

	/*
	 * @brief: Add state explicitly to this state machine
	 */
	void addState(Location&& location)
	{
		locations.push_back(location);
		label_to_location.insert(std::pair<std::string, Location*>(
						location.label, &location));
	}

	/*
	 * @brief: Add transition to the state machine
	 */
	void addTransition(Transition&& transition)
	{
		transitions.push_back(transition);
		transition_map.insert(std::pair<Location*, Transition*>(
				transition.source_location, &transition));
	}

	/*
	 * @brief: Add state through append
	 */
	HybridStateMachine<SignalClass> operator+=(Location& location)
	{
		addState(std::move(location));
	}

	/*
	 * @brief: Add transition from state to another (string version)
	 * @param: location from and to
	 */
	HybridStateMachine<SignalClass> operator+=(Transition& transition)
	{
		addTransition(std::move(transition));
	}

}; // class HybridStateMachine

template<class SignalClass> class HybridStateMachineFactory
{
private:
protected:
	static HybridStateMachineFactory* instance;

	HybridStateMachineFactory()
	{

	}

public:
	~HybridStateMachineFactory()
	{
		if (instance != nullptr) delete instance;
	}

	HybridStateMachineFactory(HybridStateMachineFactory& ) = delete;
	HybridStateMachineFactory& operator=(HybridStateMachineFactory& ) = delete;

	static HybridStateMachineFactory* getInstance()
	{
		if (instance == nullptr) instance = new HybridStateMachineFactory();
		return instance;
	}

	Location* createLocation()
	{

	}
};


} // namespace node

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_MACHINE_HPP_ */
