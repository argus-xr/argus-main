#ifndef ARGUSCONFIG_H
#define ARGUSCONFIG_H

#include "pugixml.hpp"
#include "ControllerConfig.h"
#include <unordered_map>
#include <memory>

class ArgusConfig {
public:
	static ArgusConfig* newConfig();
	static ArgusConfig* loadConfig(std::string file);
	static ArgusConfig* loadConfig();
	static std::shared_ptr<ControllerConfig> getControllerConfig(uint64_t guid);
	static void setControllerConfig(std::shared_ptr<ControllerConfig> cntcfg);
	static ArgusConfig* getInst();
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
	std::unordered_map<uint64_t, std::shared_ptr<ControllerConfig>> controllerConfigs;
	std::string configLocation = defaultConfigLocation;
};

#endif // ARGUSCONFIG_H