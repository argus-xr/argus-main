#ifndef ARGUSACCELERATIONMEASUREMENTMODEL_HPP_
#define ARGUSACCELERATIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"

namespace ArgusKalman
{

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
class AccelerationMeasurement : public Kalman::Vector<float, 3>
{
public:
    KALMAN_VECTOR(AccelerationMeasurement, float, 3)

    //! Acceleration
    static constexpr size_t aX = 0;
    static constexpr size_t aY = 1;
    static constexpr size_t aZ = 2;

    float ax()  const { return (*this)[aX]; }
    float ay()  const { return (*this)[aY]; }
    float az()  const { return (*this)[aZ]; }

    float& ax() { return (*this)[aX]; }
    float& ay() { return (*this)[aY]; }
    float& az() { return (*this)[aZ]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are known.
 * The robot can measure the direct distance to both the landmarks, for instance
 * through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
class AccelerationMeasurementModel : public Kalman::MeasurementModel<State, AccelerationMeasurement, Kalman::StandardBase>
{
public:
    /**
     * @brief Constructor
     *
     */
    AccelerationMeasurementModel()
    {
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    AccelerationMeasurement h(const State& x) const
    {
        AccelerationMeasurement measurement;

        // Acceleartion is given by the actual robot acceleration
        measurement.ax() = x.ax();
        measurement.ay() = x.ay();
        measurement.az() = x.az();
        
        return measurement;
    }
};

} // namespace ArgusKalman

#endif