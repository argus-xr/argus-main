#ifndef ARGUSCONTROLLER_H
#define ARGUSCONTROLLER_H

#include <cstddef>

#include "VideoFrame.h"

class ArgusController {
protected:
	bool active = false; // is this controller currently connected and moving?
	// include current position, orientation, and linear/angular velocity
	VideoFrame* lastFrame = nullptr;
};

#endif // ARGUSCONTROLLER_H
