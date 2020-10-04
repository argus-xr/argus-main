#include "ArgusController.h"

ArgusController::ArgusController(std::shared_ptr<ControllerConfig> config) {
	this->config = config;
}

void ArgusController::setVideoFrame(std::shared_ptr<VideoFrame> newFrame, uint64_t timestamp) {
	lastFrame = newFrame;
	lastFrameTimestamp = timestamp;
}

void ArgusController::addIMUData(IMUData data) {
	IMUDataQueue.push(data);
}