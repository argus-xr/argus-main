#include "ArgusConfig.h"

const char* ArgusConfig::defaultConfigLocation = "config.xml";
ArgusConfig* ArgusConfig::inst = nullptr;

ArgusConfig* ArgusConfig::newConfig() {
	if (!inst) {
		delete inst;
	}
	inst = new ArgusConfig();
	return inst;
}

ArgusConfig* ArgusConfig::loadConfig() {
	return loadConfig(defaultConfigLocation);
}

ArgusConfig* ArgusConfig::loadConfig(std::string file) {
	if (inst) {
		delete inst;
	}
	inst = new ArgusConfig(file);
	return inst;
}
void ArgusConfig::saveConfig() {
	doc.save_file(configLocation.c_str());
}

ArgusConfig* ArgusConfig::getInst() {
	return inst;
}

std::shared_ptr<ControllerConfig> ArgusConfig::getControllerConfig(uint64_t guid) {
	auto it = inst->controllerConfigs.find(guid);
	if (it != inst->controllerConfigs.end()) {
		return it->second;
	}
	else {
		return std::shared_ptr<ControllerConfig>(); // basically null
	}
}

void ArgusConfig::setControllerConfig(std::shared_ptr<ControllerConfig> cntcfg) {
	inst->controllerConfigs[cntcfg->guid] = cntcfg;
}

ArgusConfig::ArgusConfig() {
	knownDevicesNode = doc.append_child("KnownDevices");
	saveConfig();
}

ArgusConfig::ArgusConfig(std::string file) {
	configLocation = file;
	bool changed = false;
	doc.load_file(file.c_str());
	knownDevicesNode = doc.child("KnownDevices");
	if (!knownDevicesNode) {
		knownDevicesNode = doc.append_child("KnownDevices");
		{
			ControllerConfig* testConfig = new ControllerConfig();
			controllerConfigs[testConfig->guid] = std::shared_ptr<ControllerConfig>(testConfig);
			pugi::xml_node dev = knownDevicesNode.append_child("device");
			testConfig->saveConfig(dev);
		}
		changed = true;
	}
	else {
		for (pugi::xml_node dev : knownDevicesNode.children()) {
			auto devc = std::shared_ptr<ControllerConfig>(new ControllerConfig(dev));
			controllerConfigs[devc->guid] = devc;
		}
	}
	if (changed) {
		saveConfig();
	}
}

ArgusConfig::~ArgusConfig() {
	
}