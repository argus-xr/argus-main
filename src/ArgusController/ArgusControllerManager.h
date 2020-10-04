#ifndef ARGUSCONTROLLERMANAGER_H
#define ARGUSCONTROLLERMANAGER_H

#include <memory>
#include "ArgusController.h"
#include <unordered_map>

class ArgusControllerManager {
protected:
	static std::unordered_map<uint64_t, std::shared_ptr<ArgusController>> controllers;

public:
	static std::shared_ptr<ArgusController> getController(uint64_t guid) {
		auto it = controllers.find(guid);
		if (it != controllers.end()) {
			return it->second;
		}
		else {
			return std::shared_ptr<ArgusController>(); // basically null
		}
	}

	static void setController(uint64_t guid, std::shared_ptr<ArgusController> controller) {
		controllers[guid] = controller;
	}
};

#endif // ARGUSCONTROLLERMANAGER_H