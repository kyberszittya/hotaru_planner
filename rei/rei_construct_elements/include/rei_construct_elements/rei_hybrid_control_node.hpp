/*
 * rei_hybrid_control_node.hpp
 *
 *  Created on: Sep 2, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_HPP_


namespace rei
{

namespace node
{

template<class Subscriber, class Publisher> class HybridControlNode
{
protected:
	Publisher pub_controlevent;
	Subscriber sub_controlsignal;
public:
	virtual ~HybridControlNode() = 0;

	void startControlReception();

	/**
	 * @brief: Initialize control signal interface (receiving state control messages
	 * and induce feedback).
	 */
	virtual bool init_control_signal_interface() = 0;
};


} // namespace node


} // namespace rei




#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_HPP_ */
