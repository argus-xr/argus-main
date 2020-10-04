
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>


#include "SystemModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "AccelerationMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>
#include <Eigen/src/Geometry/Quaternion.h>


using namespace KalmanExamples;

typedef float T;

// Some type shortcuts
typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;

typedef Robot1::PositionMeasurement<T> PositionMeasurement;
typedef Robot1::OrientationMeasurement<T> OrientationMeasurement;
typedef Robot1::AccelerationMeasurement<T> AccelerationMeasurement;
typedef Robot1::PositionMeasurementModel<T> PositionModel;
typedef Robot1::OrientationMeasurementModel<T> OrientationModel;
typedef Robot1::AccelerationMeasurementModel<T> AccelerationModel;

class KalmanFilter {
public:
    void test()
    {
        // Simulated (true) system state
        State x;
        x.initialize();

        // Control input
        Control u;
        // System
        SystemModel sys;

        // Measurement models
        // Set position landmarks at (-10, -10) and (30, 75)
        PositionModel pm(-10, -10, 30, 75);
        OrientationModel om;
        AccelerationModel am;

        // Random number generation (for noise simulation)
        std::default_random_engine generator;
        generator.seed((unsigned int) std::chrono::system_clock::now().time_since_epoch().count());
        std::normal_distribution<T> noise(0, 1);

        // Unscented Kalman Filter
        Kalman::UnscentedKalmanFilter<State> ukf(1);

        // Init filters with true system state
        ukf.init(x);

        // Standard-Deviation of noise added to all state vector components during state transition
        T systemNoise = 0.1f;
        // Standard-Deviation of noise added to all measurement vector components in orientation measurements
        T orientationNoise = 0.025f;
        // Standard-Deviation of noise added to all measurement vector components in distance measurements
        T distanceNoise = 0.25f;

        // Simulate for 100 steps
        const size_t N = 100;
        for (size_t i = 1; i <= N; i++)
        {
            // Generate some control input
            u.v() = 1.f + std::sin(T(2) * T(M_PI) / T(N));
            u.dtheta() = std::sin(T(2) * T(M_PI) / T(N)) * (1 - 2 * (i > 50));

            // Simulate system
            x = sys.f(x, u);

            // Add noise: Our robot move is affected by noise (due to actuator failures)
            x.px() += systemNoise * noise(generator);
            x.py() += systemNoise * noise(generator);
            x.pz() += systemNoise * noise(generator);

            // Predict state for current time-step using the filters
            auto x_ukf = ukf.predict(sys, u);

            // Orientation measurement
            {
                // We can measure the orientation every 5th step
                OrientationMeasurement orientation = om.h(x);

                // Measurement is affected by noise as well
                //orientation.theta() += orientationNoise * noise(generator);
                Eigen::Quaternion<float> quat = x.getQuat();
                Eigen::Quaternion<float> randomQuat = Eigen::Quaternion<float>::UnitRandom();
                quat = quat.slerp(orientationNoise, randomQuat).normalized();

                orientation.ox() = quat.x();
                orientation.oy() = quat.y();
                orientation.oz() = quat.z();
                orientation.ow() = quat.w();

                // Update UKF
                x_ukf = ukf.update(om, orientation);
            }

            // Position measurement
            {
                // We can measure the position every 10th step
                PositionMeasurement position = pm.h(x);

                // Measurement is affected by noise as well
                position.d1() += distanceNoise * noise(generator);
                position.d2() += distanceNoise * noise(generator);

                // Update UKF
                //x_ukf = ukf.update(pm, position);
            }

            // Acceleration measurement
            {
                // We can measure the position every 10th step
                AccelerationMeasurement acceleration = am.h(x);



                // Update UKF
                x_ukf = ukf.update(am, acceleration);
            }

            if (i % 1 == 0) {
                // Print to stdout as csv format
                printf("Kalman test: p %6.2f, %6.2f, %6.2f - a %6.2f, %6.2f, %6.2f - v %6.2f, %6.2f, %6.2f - q %6.2f, %6.2f, %6.2f, %6.2f\n", x.px(), x.py(), x.pz(),
                        x.vx(), x.vy(), x.vz(), x.ax(), x.ay(), x.az(), x.ox(), x.oy(), x.oz(), x.ow());
                printf("Estimate:    p %6.2f, %6.2f, %6.2f - a %6.2f, %6.2f, %6.2f - v %6.2f, %6.2f, %6.2f - q %6.2f, %6.2f, %6.2f, %6.2f\n", x_ukf.px(), x_ukf.py(), x_ukf.pz(),
                        x_ukf.vx(), x_ukf.vy(), x_ukf.vz(), x_ukf.ax(), x_ukf.ay(), x_ukf.az(), x_ukf.ox(), x_ukf.oy(), x_ukf.oz(), x_ukf.ow());
            }
        }
    }
};
