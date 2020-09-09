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

struct UnknownEvent: public std::exception
{

	const char* what() const throw()
	{
		return "Unknown event received on pipeline";
	}
};

}

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_STATE_EXCEPTIONS_HPP_ */
