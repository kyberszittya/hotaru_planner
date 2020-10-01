/*
 * read_hybrid_system_spec.hpp
 *
 *  Created on: Sep 8, 2020
 *      Author: kyberszittya
 */

/**
 * @def read_hybrid_stem_spec.hpp
 * @brief Read hybrid dynamical system from specification file
 * @author Hajdu Csaba (kyberszittya)
 * 		CONTACT: kyberszittya@protonmail.ch
 */


#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_SPECREAD_READ_HYBRID_SYSTEM_SPEC_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_SPECREAD_READ_HYBRID_SYSTEM_SPEC_HPP_

#include <tinyxml2.h>
#include <exception>
#include <fstream>
#include <iostream>


#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>
#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine_factory.hpp>

namespace rei
{

namespace specification
{

namespace exceptions
{

struct SpecElementNotFound: public std::exception
{
	const char* what() const throw()
	{
		return "Specification element not found";
	}
};

struct SpecificationFileNotFound: public std::exception
{
	const char* what() const throw()
	{
		return "Specification file not found";
	}
};

} // namespace exceptions

template<class Clock> class ReadSpecXml
{
public:
	virtual ~ReadSpecXml() {}
	virtual tinyxml2::XMLError readXmlDescription(const std::string file_name) = 0;
};



/**
 * @class ReadSpecXmlHybridStateMachine
 * @brief XML reader class to instantiate a Hybrid automata
 *
 */
template<class Timestamp, class Clock> class ReadSpecXmlHybridStateMachine: public ReadSpecXml<Clock>
{
private:
	node::HybridStateMachinePtr<Timestamp, Clock> hybrid_state_machine;
	node::HybridStateMachineFactory<Timestamp, Clock>* factory;
public:

	ReadSpecXmlHybridStateMachine<Timestamp, Clock>(): factory(nullptr){}

	node::HybridStateMachinePtr<Timestamp, Clock> getCurrentHybridStateMachine()
	{
		return std::move(hybrid_state_machine);
	}

	std::map<std::string, unsigned int> getEventMapping()
	{
		return factory->getEventMapping();
	}

	virtual tinyxml2::XMLError readXmlDescription(const std::string file_name) override
	{
		if (factory==nullptr)
		{
			factory = node::HybridStateMachineFactory<Timestamp, Clock>::getInstance();
		}
		/// Step 0. Read xml file
		std::ifstream f(file_name);
		if (!f.good())
		{
			std::cerr << "Specification file not found: " << file_name << '\n';
			throw exceptions::SpecificationFileNotFound();
		}
		using namespace tinyxml2;
		XMLDocument doc;
		XMLError error = doc.LoadFile(file_name.c_str());
		XMLNode* root = doc.FirstChild();
		if (root == nullptr)
			throw exceptions::SpecElementNotFound();
		XMLElement* el_hy_sys = doc.FirstChildElement("hybridsystem");
		if (el_hy_sys == nullptr)
			throw exceptions::SpecElementNotFound();
		/// Step 1. Read statemachines
		XMLElement* statemachine = el_hy_sys->FirstChildElement("statemachine");
		if (statemachine==nullptr)
			throw exceptions::SpecElementNotFound();
		/// Step 2.1
		XMLElement* el_name = statemachine->FirstChildElement("name");
		if (el_name==nullptr)
			throw exceptions::SpecElementNotFound();
		std::string hysm_name = el_name->GetText();
		XMLElement* el_delta_timeout = statemachine->FirstChildElement("delta_timeout");
		if (el_delta_timeout==nullptr)
			throw exceptions::SpecElementNotFound();
		double delta_timeout = std::atof(el_delta_timeout->GetText());
		hybrid_state_machine = factory->createHybridStateMachine(hysm_name, delta_timeout);
		/// Step 2. Read locations
		XMLElement* el_locations = statemachine->FirstChildElement("locations");
		if (el_locations==nullptr)
			throw exceptions::SpecElementNotFound();
		std::vector<std::string> location_labels;
		for (XMLElement* el = el_locations->FirstChildElement("location"); el != nullptr; el=el->NextSiblingElement("location"))
		{
			std::string location_name = el->FirstChildElement("name")->GetText();
			location_labels.push_back(location_name);
		}
		factory->addLocations(*hybrid_state_machine, location_labels.begin(), location_labels.end());
		/// Step 3. Set terminal locations
		XMLElement* el_start_location = statemachine->FirstChildElement("start_location");
		XMLElement* el_end_location = statemachine->FirstChildElement("end_location");
		/// Start element initialization
		std::string start_location = el_start_location->FirstChildElement("location")->GetText();
		std::string start_event = el_start_location->FirstChildElement("event")->GetText();
		// End element initialization
		std::string end_location = el_end_location->FirstChildElement("location")->GetText();
		std::string end_event = el_end_location->FirstChildElement("event")->GetText();
		/// Set terminal locations
		factory->setTerminalLocations(hybrid_state_machine, start_event, start_location, end_event, end_location);
		/// Step 4. Add transitions
		XMLElement* el_transitions = statemachine->FirstChildElement("transitions");
		std::vector<node::TransitionTuple> transitions;
		for (XMLElement* el = el_transitions->FirstChildElement("transition"); el != nullptr; el=el->NextSiblingElement("transition"))
		{
			std::string source_location = el->FirstChildElement("source")->GetText();
			std::string target_location = el->FirstChildElement("target")->GetText();
			std::string discrete_event = el->FirstChildElement("discreteevent")->GetText();
			transitions.push_back(std::make_tuple(discrete_event, source_location, target_location));
		}
		factory->addDiscreteTransitions(*hybrid_state_machine, transitions.begin(), transitions.end());
		/// Return serialization result
		return error;
	}
};

}

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_SPECREAD_READ_HYBRID_SYSTEM_SPEC_HPP_ */
