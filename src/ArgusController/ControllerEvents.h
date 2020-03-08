#ifndef CONTROLLEREVENTS_H
#define CONTROLLEREVENTS_H

#include "VideoFrame.h"
#include <memory>

struct NewVideoFrameEvent {
	std::shared_ptr<VideoFrame> pointer;
};

#endif // CONTROLLEREVENTS_H
