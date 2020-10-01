/*
 * rei_internal_signaling_structure.hpp
 *
 *  Created on: Sep 14, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_HYBRID_DYNAMIC_SYSTEM_REI_INTERNAL_SIGNALING_STRUCTURE_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_HYBRID_DYNAMIC_SYSTEM_REI_INTERNAL_SIGNALING_STRUCTURE_HPP_

#include <memory>

namespace rei
{

namespace node
{

enum class HybridStateStepResult{ TRANSITED, PROCESS_TIMEOUT, SHIFTED_TIMESTAMP, EMPTY_QUEUE, NO_TRANSITION };

template<class Timestamp> class NotificationEvent
{
private:
	const std::string event_name;
	const Timestamp stamp;
public:
	NotificationEvent(std::string name, const Timestamp stamp): event_name(name),
		stamp(stamp)
	{}

	const std::string getEventName() const
	{
		return event_name;
	}

	const Timestamp getTimestamp() const
	{
		return stamp;
	}
};


template<class Timestamp> using NotificationEventPtr = std::shared_ptr<NotificationEvent<Timestamp>>;

template<class Timestamp> class TransitionNotificationEvent: public NotificationEvent<Timestamp>
{
private:
	const std::string source_location;
	const std::string target_location;
public:
	TransitionNotificationEvent(std::string name, const Timestamp stamp, const std::string source_location, const std::string target_location):
		NotificationEvent<Timestamp>(name, stamp),
		source_location(source_location),
		target_location(target_location)
	{}

	const std::string getSourceLocation() const
	{
		return source_location;
	}

	const std::string getTargetLocation() const
	{
		return target_location;
	}
};

template<class Timestamp> using TransitionNotificationEventPtr = std::shared_ptr<TransitionNotificationEvent<Timestamp>>;

template<class Timestamp> class LocationNotificationEvent: public NotificationEvent<Timestamp>
{
private:
	const std::string location;
public:
	LocationNotificationEvent(std::string name, const Timestamp stamp, const std::string location):
		NotificationEvent<Timestamp>(name, stamp),
		location(location)
		{}

	const std::string getLocation() const
	{
		return location;
	}
};

template<class Timestamp> using LocationNotificationEventPtr = std::shared_ptr<LocationNotificationEvent<Timestamp>>;

} // namespace node

} // namespace rei

#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_HYBRID_DYNAMIC_SYSTEM_REI_INTERNAL_SIGNALING_STRUCTURE_HPP_ */
