#include "ArgusController.h"

ArgusController::ArgusController(std::shared_ptr<ControllerConfig> config) {
	this->config = config;
	kf.init();
}

void ArgusController::setVideoFrame(std::shared_ptr<VideoFrame> newFrame, uint64_t timestamp) {
	lastFrame = newFrame;
	lastFrameTimestamp = timestamp;
}

void ArgusController::addIMUData(IMUData data) {
	kf.feedIMU(data);
	//IMUDataQueue.push(data);
	position = kf.getPosition();
	orientation = kf.getOrientation();
}

void ArgusController::printTracking() {
	printf("Controller tracking: p %7.2f, %7.2f, %7.2f - q %6.2f, %6.2f, %6.2f, %6.2f\n", position.x(), position.y(), position.z(), orientation.w(), orientation.x(), orientation.y(), orientation.z());
}