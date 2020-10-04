#ifndef ARGUSVIEWER_H
#define ARGUSVIEWER_H

#define _USE_MATH_DEFINES
#include <cmath>

#include "SystemModel.hpp"
#include <kalman/UnscentedKalmanFilter.hpp>
#include "AccelerationMeasurementModel.hpp"
#include "OrientationMeasurementModel.hpp"

namespace ArgusKalman {

	class KalmanFilter {

	protected:
		State state;
		Control control;
		SystemModel sys;
		Kalman::UnscentedKalmanFilter<State> ukf;
		AccelerationMeasurementModel amm;
		OrientationMeasurementModel omm;
	public:
		void init();
		void tick();
		void test();
	};

}

#endif // ARGUSVIEWER_H