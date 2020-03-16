#include "ArgusDevice.h"

ArgusDevice::ArgusDevice() {

}

ArgusDevice::~ArgusDevice() {
	if (viewer != nullptr) {
		delete viewer;
		viewer = nullptr;
	}
	if (controller != nullptr) {
		delete controller;
		controller = nullptr;
	}
}

bool ArgusDevice::isViewer() {
	return viewer != nullptr;
}

bool ArgusDevice::isController() {
	return controller != nullptr;
}

ArgusViewer* ArgusDevice::getViewer() {
	return viewer;
}

ArgusController* ArgusDevice::getController() {
	return controller;
}