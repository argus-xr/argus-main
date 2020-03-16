#ifndef ARGUSDEVICEMANAGER_H
#define ARGUSDEVICEMANAGER_H

#include <vector>
#include <ArgusDevice.h>
#include <memory>

class ArgusDeviceManager {
public:
	std::shared_ptr<ArgusDevice> getDevice(std::size_t num);
	std::size_t getDeviceCount();

	void registerDevice(std::shared_ptr<ArgusDevice> device);
protected:
	std::vector<std::shared_ptr<ArgusDevice>> deviceList;
};

#endif // ARGUSDEVICEMANAGER_H