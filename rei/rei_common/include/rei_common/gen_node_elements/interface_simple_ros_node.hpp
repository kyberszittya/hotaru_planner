/*
 * interface_simple_ros_node.hpp
 *
 *  Created on: Apr 10, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_COMMON_GEN_NODE_ELEMENTS_INTERFACE_SIMPLE_ROS_NODE_HPP_
#define INCLUDE_REI_COMMON_GEN_NODE_ELEMENTS_INTERFACE_SIMPLE_ROS_NODE_HPP_

#include <exception>

namespace rei
{

class ExceptionNodePreInitialization: public std::exception
{
public:
	virtual const char* what() const throw()
	{
		return "Failed to initialize pre-node";
	}
};

class ExceptionNodePostInitialization: public std::exception
{
public:
	virtual const char* what() const throw()
	{
		return "Failed to initialize post-node";
	}
};


class ExceptionNodeAssignSyncGuards: public std::exception
{
public:
	virtual const char* what() const throw()
	{
		return "Failed to assign synchronization state machine guards";
	}
};


class ExceptionNodeMiddleWare: public std::exception
{
public:
	virtual const char* what() const throw()
	{
		return "Failed to initialize middleware section of the node";
	}
};


class ExceptionTimeoutStateMachineInitialization: public std::exception
{
public:
	virtual const char* what() const throw()
	{
		return "Failed to initialize timeout checking state machine";
	}
};

/**
 * This simple ROS node should be platform-independent, i.e. the same functionalities should be implemented in ROS1 and ROS2
 */
class Interface_SimpleRosNode
{
public:
	virtual ~Interface_SimpleRosNode() = 0;

	/*
	 * @brief: A function that calls every function required to initialize a node generated
	 *     STEPS:
	 *           1. Initialize descendant specific functionalities before middleware functionalities
	 *           2. Initialize timeout state machine
	 *           3. Assign guard related to timeout functions
	 *           4. Initialize middleware functionalities
	 *           5. Initialize descendant node-specific functionalities
	 */
	void initialize(const bool debug)
	{
		// Initialize node before initialization
		if (!initPre())
		{
			throw ExceptionNodePreInitialization();
		}
		// Initialize timeout machine
		if (!initTimeoutStateMachine())
		{
			throw ExceptionTimeoutStateMachineInitialization();
		}
		// Initialize middleware exception
		if (!assignSyncGuards())
		{
			throw ExceptionTimeoutStateMachineInitialization();
		}
		// Initialize middleware
		if (!initMiddleware(debug))
		{
			throw ExceptionNodeMiddleWare();
		}
		// Initialize post middleware
		if (!initPost())
		{
			throw ExceptionNodePostInitialization();
		}
	}
	virtual bool initPre() = 0;   						///< Initialize pre middleware parts of the node
	virtual bool initTimeoutStateMachine() = 0;			///< Initialize timeout statemachine part of the node
	virtual bool assignSyncGuards() = 0;				///< Assign sync guards to state machine tranisitions
	virtual bool initMiddleware(const bool debug) = 0;	///< Initialize middleware
	virtual bool initPost() = 0;						///< Initialize post middleware part of the node
};

}


#endif /* INCLUDE_REI_COMMON_GEN_NODE_ELEMENTS_INTERFACE_SIMPLE_ROS_NODE_HPP_ */
