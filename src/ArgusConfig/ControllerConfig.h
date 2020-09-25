#ifndef ARGUSCONTROLLERCONFIG_H
#define ARGUSCONTROLLERCONFIG_H

#include "pugixml.hpp"

class ControllerConfig {
public:
	ControllerConfig();
	ControllerConfig(pugi::xml_node &data);
	~ControllerConfig();
	void saveConfig(pugi::xml_node& node);
protected:
	uint64_t guid;
};

#endif // ARGUSCONTROLLERCONFIG_H