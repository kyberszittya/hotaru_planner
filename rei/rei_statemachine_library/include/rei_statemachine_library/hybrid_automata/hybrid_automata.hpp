/*
 * hybrid_automata.hpp
 *
 *  Created on: Feb 26, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_STATEMACHINE_LIBRARY_HYBRID_AUTOMATA_HYBRID_AUTOMATA_HPP_
#define INCLUDE_REI_STATEMACHINE_LIBRARY_HYBRID_AUTOMATA_HYBRID_AUTOMATA_HPP_


#include <rei_statemachine_library/abstract_statemachine_definitions.hpp>

namespace rei
{

template<class StateDef, class GuardDef> class AbstractSingularHybridStateMachine
{
private:
	double var;

public:
	void stepTime(unsigned long long timestamp)
	{

	}
};

}


#endif /* INCLUDE_REI_STATEMACHINE_LIBRARY_HYBRID_AUTOMATA_HYBRID_AUTOMATA_HPP_ */
