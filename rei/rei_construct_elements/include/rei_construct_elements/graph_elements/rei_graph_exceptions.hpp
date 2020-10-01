/*
 * rei_graph_exceptions.hpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_GRAPH_ELEMENTS_REI_GRAPH_EXCEPTIONS_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_GRAPH_ELEMENTS_REI_GRAPH_EXCEPTIONS_HPP_


#include <exception>

namespace rei
{

namespace graph
{

namespace exceptions
{

struct NonexistentEdge: public std::exception
{
	const char* what() const throw()
	{
		return "Edge is nonexistent in graph!";
	}
};

} // namespace exceptions

} // namespace node


} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_GRAPH_ELEMENTS_REI_GRAPH_EXCEPTIONS_HPP_ */
