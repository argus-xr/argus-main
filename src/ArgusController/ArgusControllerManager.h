#ifndef ARGUSCONTROLLERMANAGER_H
#define ARGUSCONTROLLERMANAGER_H

#include <memory>
#include <vector>
#include "ArgusController.h"

class ArgusControllerManager {
protected:
	std::vector<std::shared_ptr<ArgusController>> controllers;
};

#endif // ARGUSCONTROLLERMANAGER_H