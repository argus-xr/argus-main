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
	uint64_t roundedStep = (data.timestamp_us + timestepLength / 2) / timestepLength; // round to 10 ms timesteps
	if (roundedStep - lastIMUStep > 1000 || lastIMUStep <= 0) { // lost IMU for a full 10 seconds (1000 steps), or only just started.
		kf.init();
		kf.loop();
		lastIMUStep = roundedStep + 1;
	}
	else {
		uint64_t steps = roundedStep - lastIMUStep;

		for (int i = 0; i < steps; ++i) {
			kf.loop();
		}

		lastIMUStep = roundedStep;
	}
	kf.feedIMU(data);

	position = kf.getPosition();
	orientation = kf.getOrientation();
}

void ArgusController::printTracking() {
	printf("Controller tracking: p %7.2f, %7.2f, %7.2f - q %6.2f, %6.2f, %6.2f, %6.2f\n", position.x(), position.y(), position.z(), orientation.w(), orientation.x(), orientation.y(), orientation.z());
}