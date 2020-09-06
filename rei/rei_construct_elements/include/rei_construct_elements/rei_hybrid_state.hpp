/*
 * rei_hybrid_state.hpp
 *
 *  Created on: Sep 5, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_HPP_


#include <vector>
#include <map>

template<class StateVar> class ContinousVariable
{
private:
	const std::string var_id;
	StateVar val;
public:
	ContinousVariable(const std::string& var_id, const StateVar initial_value):
		var_id(var_id), val(initial_value){}
};

template<class StateVar> class HybridState
{
private:
	std::map<std::string, ContinousVariable<StateVar>> variables;
public:

};


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_HPP_ */
