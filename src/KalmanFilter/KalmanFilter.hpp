#ifndef ARGUSKALMANFILTER_H
#define ARGUSKALMANFILTER_H

// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>


#include "SystemModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "AccelerationMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"
#include "RotationMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>
#include <Eigen/src/Geometry/Quaternion.h>

#include "IMUData.h"


using namespace KalmanExamples;

typedef float T;

// Some type shortcuts
typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;

typedef Robot1::PositionMeasurement<T> PositionMeasurement;
typedef Robot1::OrientationMeasurement<T> OrientationMeasurement;
typedef Robot1::AccelerationMeasurement<T> AccelerationMeasurement;
typedef Robot1::RotationMeasurement<T> RotationMeasurement;
typedef Robot1::PositionMeasurementModel<T> PositionModel;
typedef Robot1::OrientationMeasurementModel<T> OrientationModel;
typedef Robot1::AccelerationMeasurementModel<T> AccelerationModel;
typedef Robot1::RotationMeasurementModel<T> RotationModel;

class KalmanFilter {
protected:
    State x;
    Control u;
    SystemModel sys;

    // Measurement models
    OrientationModel om;
    AccelerationModel am;
    RotationModel rm;

    // Unscented Kalman Filter
    Kalman::UnscentedKalmanFilter<State> ukf;
public:
    void init() {
        x.initialize();

        // Init filters with true system state
        ukf = Kalman::UnscentedKalmanFilter<State>(1);
        ukf.init(x);
    }

    void loop() {
        sys.f(x, u);
        x = ukf.predict(sys, u);
    }

    void feedIMU(IMUData id) {
        // Acceleration measurement
        {
            // We can measure the position every 10th step
            AccelerationMeasurement acceleration = am.h(x);

            acceleration.ax() = id.aX;
            acceleration.ay() = id.aY;
            acceleration.az() = id.aZ;

            // Update UKF
            x = ukf.update(am, acceleration);
        }
    }

    void test()
    {
        init();
        State est = x;

        // Random number generation (for noise simulation)
        std::default_random_engine generator;
        generator.seed((unsigned int) std::chrono::system_clock::now().time_since_epoch().count());
        std::normal_distribution<T> noise(0, 1);

        // Init filters with true system state
        ukf.init(x);

        // Standard-Deviation of noise added to all state vector components during state transition
        T systemNoise = 1.f;
        // Standard-Deviation of noise added to all measurement vector components in orientation measurements
        T orientationNoise = 0.025f;
        // Standard-Deviation of noise added to all measurement vector components in distance measurements
        T distanceNoise = 0.25f;
        T accelerometerNoise = 0.25f;
        T gyroDriftNoise = 0.25f;

        Eigen::Vector3f gyroDrift;
        gyroDrift.setZero();

        // Simulate for 100 steps
        const size_t N = 500;
        for (size_t i = 1; i <= N; i++)
        {
            // Generate some control input
            u.v() = 1.f + std::sin(T(2) * T(3.14f) / T(N));
            u.dtheta() = std::sin(T(2) * T(3.14f) / T(N)) * (1 - 2 * (i > N / 2));

            // Simulate system
            x = sys.f(x, u);
            est = sys.f(est, u);

            {
                // Add noise: Our robot move is affected by noise (due to actuator failures)
                x.px() += systemNoise * noise(generator) * x.timestep;
                x.py() += systemNoise * noise(generator) * x.timestep;
                x.pz() += systemNoise * noise(generator) * x.timestep;
    
                Eigen::Quaternion<float> quat = State::getQuat(x);
                Eigen::Quaternion<float> randomQuat = Eigen::Quaternion<float>::UnitRandom().normalized();
                quat = quat.slerp(orientationNoise, randomQuat).normalized();

                State::setQuat(x, quat);
            }

            // Predict state for current time-step using the filters
            est = ukf.predict(sys, u);

            // Orientation measurement
            {
                // We can measure the orientation every 5th step
                OrientationMeasurement orientation = om.h(x);

                // Measurement is affected by noise as well
                //orientation.theta() += orientationNoise * noise(generator);
                Eigen::Quaternion<float> quat = State::getQuat(x);
                Eigen::Quaternion<float> randomQuat = Eigen::Quaternion<float>::UnitRandom().normalized();
                quat = quat.slerp(orientationNoise, randomQuat).normalized();

                OrientationMeasurement::setQuat(orientation, quat);

                // Update UKF
                est = ukf.update(om, orientation);
            }

            // Acceleration measurement
            {
                // We can measure the position every 10th step
                AccelerationMeasurement acceleration = am.h(x);

                acceleration.ax() += accelerometerNoise * noise(generator);
                acceleration.ay() += accelerometerNoise * noise(generator);
                acceleration.az() += accelerometerNoise * noise(generator);

                // Update UKF
                est = ukf.update(am, acceleration);
            }

            // Rotation measurement
            {
                // We can measure the position every 10th step
                /*RotationMeasurement rotation = rm.h(x);

                gyroDrift.x() += gyroDriftNoise * noise(generator);
                gyroDrift.y() += gyroDriftNoise * noise(generator);
                gyroDrift.z() += gyroDriftNoise * noise(generator);

                rotation.rx() += gyroDrift.x();
                rotation.ry() += gyroDrift.y();
                rotation.rz() += gyroDrift.z();

                // Update UKF
                est = ukf.update(rm, rotation);*/
            }

            if (i % 10 == 0) {
                auto xQuat = State::getQuat(x);
                // Print to stdout as csv format
                printf("Kalman test: p %6.2f, %6.2f, %6.2f - v %6.2f, %6.2f, %6.2f - a %6.2f, %6.2f, %6.2f - q %6.2f, %6.2f, %6.2f, %6.2f\n", x.px(), x.py(), x.pz(),
                    x.vx(), x.vy(), x.vz(), x.ax(), x.ay(), x.az(), xQuat.w(), xQuat.x(), xQuat.y(), xQuat.z());
                auto estQuat = State::getQuat(est);
                printf("Estimate:    p %6.2f, %6.2f, %6.2f - v %6.2f, %6.2f, %6.2f - a %6.2f, %6.2f, %6.2f - q %6.2f, %6.2f, %6.2f, %6.2f\n\n", est.px(), est.py(), est.pz(),
                    est.vx(), est.vy(), est.vz(), est.ax(), est.ay(), est.az(), estQuat.w(), estQuat.x(), estQuat.y(), estQuat.z());
            }
        }
    }
};
#endif // ARGUSKALMANFILTER_H