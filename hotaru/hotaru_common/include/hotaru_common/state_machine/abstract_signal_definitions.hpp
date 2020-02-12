/*
 * abstract_signal_definitions.hpp
 *
 *  Created on: Jan 30, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_COMMON_STATE_MACHINE_ABSTRACT_SIGNAL_DEFINITIONS_HPP_
#define INCLUDE_HOTARU_COMMON_STATE_MACHINE_ABSTRACT_SIGNAL_DEFINITIONS_HPP_


namespace hotaru
{


class AbstractSignalInterface
{
public:
	virtual const unsigned long getId() = 0;
	virtual const unsigned long long getTimestamp() = 0;
};

template<int SIG_ID> class AbstractSignal: public AbstractSignalInterface
{
private:
	const unsigned long long timestamp;
public:
	AbstractSignal(const unsigned long long timestamp):
		timestamp(timestamp){}
	static const unsigned long signal_id = SIG_ID;
	virtual const unsigned long getId() override
	{
		return signal_id;
	}
	virtual const unsigned long long getTimestamp() override
	{
		return timestamp;
	}
};
}

#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_ABSTRACT_SIGNAL_DEFINITIONS_HPP_ */
