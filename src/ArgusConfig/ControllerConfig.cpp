#include "ControllerConfig.h"

ControllerConfig::ControllerConfig() {
	guid = 0;
}

ControllerConfig::ControllerConfig(pugi::xml_node& data) {
	guid = data.attribute("guid").as_ullong();
}

ControllerConfig::~ControllerConfig() {
	
}

void ControllerConfig::saveConfig(pugi::xml_node& node) {
	if (!node) {
		return;
	}
	node.remove_children();
	node.remove_attributes();
	pugi::xml_attribute att = node.append_attribute("guid");
	att.set_value(guid);
}