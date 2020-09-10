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

#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>
#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine_factory.hpp>

namespace rei
{

namespace node
{


class ReadSpecXml
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
template<class Timestamp, class Clock> class ReadSpecXmlHybridStateMachine: public ReadSpecXml
{
private:
	HybridStateMachinePtr<Timestamp, Clock> hybrid_state_machine;
	HybridStateMachineFactory<Timestamp, Clock>* factory;
public:

	virtual tinyxml2::XMLError readXmlDescription(const std::string file_name) override
	{
		if (factory==nullptr)
		{
			factory = HybridStateMachineFactory<Timestamp, Clock>::getInstance();
		}
		hybrid_state_machine = factory->createHybridStateMachine();
		/// Step 0. Read xml file
		using namespace tinyxml2;
		XMLDocument doc;
		XMLError error = doc.LoadFile(file_name.c_str());
		XMLNode* root = doc.FirstChild();
		if (root == nullptr) return XML_ERROR_FILE_READ_ERROR;
		/// Step 1. Read statemachines
		XMLElement* statemachine = root->FirstChildElement("statemachine");
		/// Step 2. Read locations
		X
		return error;
	}
};

}

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_SPECREAD_READ_HYBRID_SYSTEM_SPEC_HPP_ */
