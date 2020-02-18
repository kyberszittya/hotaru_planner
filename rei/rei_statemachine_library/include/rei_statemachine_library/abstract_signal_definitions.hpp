/*
 * abstract_signal_definitions.hpp
 *
 *  Created on: Jan 30, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_COMMON_STATE_MACHINE_ABSTRACT_SIGNAL_DEFINITIONS_HPP_
#define INCLUDE_REI_COMMON_STATE_MACHINE_ABSTRACT_SIGNAL_DEFINITIONS_HPP_


namespace rei
{


class AbstractSignalInterface
{
public:
	virtual ~AbstractSignalInterface() = 0;
	virtual const unsigned long getId() = 0;
	virtual const unsigned long long getTimestamp() = 0;
	virtual void effect() = 0;
	virtual bool isEffective() const = 0;
};

template<int SIG_ID> class AbstractSignal: public AbstractSignalInterface
{
private:
	bool effective;
	const unsigned long long timestamp;
public:
	AbstractSignal(const unsigned long long timestamp):
		effective(false),
		timestamp(timestamp){}
	//virtual ~AbstractSignal(){}
	static const unsigned long signal_id = SIG_ID;
	virtual const unsigned long getId() override
	{
		return signal_id;
	}
	virtual const unsigned long long getTimestamp() override
	{
		return timestamp;
	}

	virtual void effect() override
	{
		effective = true;
	}

	virtual bool isEffective() const
	{
		return effective;
	}
};
}

#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_ABSTRACT_SIGNAL_DEFINITIONS_HPP_ */
