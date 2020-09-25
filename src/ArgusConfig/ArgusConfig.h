#ifndef ARGUSCONFIG_H
#define ARGUSCONFIG_H

#include "pugixml.hpp"
#include "ControllerConfig.h"
#include <vector>
#include <memory>

class ArgusConfig {
public:
	static ArgusConfig* newConfig();
	static ArgusConfig* loadConfig(std::string file);
	static ArgusConfig* loadConfig();
	void saveConfig();
protected:
	ArgusConfig();
	ArgusConfig(std::string file);
	~ArgusConfig();
	static ArgusConfig* inst;
	const static char* defaultConfigLocation;
private:
	pugi::xml_document doc;
	pugi::xml_node knownDevicesNode;
	std::vector<std::shared_ptr<ControllerConfig>> controllerConfigs;
	std::string configLocation = defaultConfigLocation;
};

#endif // ARGUSCONFIG_H