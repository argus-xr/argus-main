#ifndef ARGUSDEVICE_H
#define ARGUSDEVICE_H

// Forward declarations to avoid #including them in the header.
class ArgusViewer;
class ArgusController;

class ArgusDevice {
public:
	ArgusDevice();
	~ArgusDevice();
	bool isViewer();
	bool isController();
	ArgusViewer* getViewer();
	ArgusController* getController();
protected:
	ArgusViewer* viewer = nullptr;
	ArgusController* controller = nullptr;
};

#endif // ARGUSDEVICE_H