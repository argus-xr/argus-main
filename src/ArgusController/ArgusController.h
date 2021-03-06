#ifndef ARGUSCONTROLLER_H
#define ARGUSCONTROLLER_H

#include <cstddef>

#include "VideoFrame.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <kalman/LinearizedSystemModel.hpp>// somehow, Quaternion.h breaks without this.
#include <Eigen/src/Geometry/Quaternion.h>

#include <stack>

#include "KalmanFilter.hpp"
#include "ControllerConfig.h"
#include "IMUData.h"

class ArgusController {
protected:
	bool active = false; // is this controller currently connected and moving?
	// include current position, orientation, and linear/angular velocity
	std::shared_ptr<VideoFrame> lastFrame;
	uint64_t lastFrameTimestamp = 0;

	std::stack<IMUData> IMUDataQueue;
	uint64_t lastIMUStep = 0;
	uint64_t timestepLength = 10000; // 10 ms

	Eigen::Quaternion<double> orientation;
	Eigen::Vector3d position;

	KalmanFilter kf;
	std::shared_ptr<ControllerConfig> config;

public:
	ArgusController(std::shared_ptr<ControllerConfig> config);
	void setVideoFrame(std::shared_ptr<VideoFrame> newFrame, uint64_t timestamp);
	void addIMUData(IMUData data);
	void printTracking();
};

#endif // ARGUSCONTROLLER_H
