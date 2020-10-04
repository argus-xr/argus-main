#include "KalmanFilter.h"

namespace ArgusKalman {

	void KalmanFilter::init() {
		state.setZero();
		ukf.init(state);
	}

	void KalmanFilter::tick() {
		sys.f(state, control);
		ukf.predict(sys, control);

		{
			OrientationMeasurement om = omm.h(state);

			// om.ox() = // set measurement.

			ukf.update(omm, om);
		}

		{
			AccelerationMeasurement am = amm.h(state);

			ukf.update(amm, am);
		}
	}

	void KalmanFilter::test() {
		init();

		int zAngle = 0;

		for (int i = 0; i < 2000; ++i) {
			sys.f(state, control);
			auto ukf_state = ukf.predict(sys, control);

			{
				OrientationMeasurement om = omm.h(state);



				ukf_state = ukf.update(omm, om);
			}

			{
				AccelerationMeasurement am = amm.h(state);

				ukf_state = ukf.update(amm, am);
			}
			if (i % 100 == 0) {
				printf("Kalman test: px %f.2 py %f.2 pz %f.2", ukf_state.px(), ukf_state.py(), ukf_state.pz());
			}
		}
	}

}