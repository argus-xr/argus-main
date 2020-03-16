#include "ArgusDeviceManager.h"


std::shared_ptr<ArgusDevice> ArgusDeviceManager::getDevice(std::size_t num) {
	return deviceList.at(num);
}

std::size_t ArgusDeviceManager::getDeviceCount() {
	return deviceList.size();
}

void ArgusDeviceManager::registerDevice(std::shared_ptr<ArgusDevice> device) {
	deviceList.push_back(device);
}