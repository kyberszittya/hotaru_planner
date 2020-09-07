/*
 * rei_event_exceptions.hpp
 *
 *  Created on: Sep 7, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_EXCEPTIONS_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_EXCEPTIONS_HPP_

#include <string>
#include <exception>

namespace rei
{

namespace exceptions
{

class ClockUnavailableException: std::exception
{
private:
	//std::string hymsm_name;
public:
	//ClockUnavailableException(const std::string hysm_name): hymsm_name(hymsm_name){}

	virtual const char* what() const noexcept
	{
		//std::string s("Clock is unavailable for Hybrid state machine: ");
		//s += hymsm_name;
		//return s.c_str();
		return "WAT";
	}
};

}

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_EXCEPTIONS_HPP_ */
