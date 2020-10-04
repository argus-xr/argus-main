#ifndef ARGUSORIENTATIONMEASUREMENTMODEL_HPP_
#define ARGUSORIENTATIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"

namespace ArgusKalman
{

/**
 * @brief Measurement vector measuring an orientation (i.e. by using a compass)
 *
 * @param T Numeric scalar type
 */
class OrientationMeasurement : public Kalman::Vector<float, 4>
{
public:
    KALMAN_VECTOR(OrientationMeasurement, float, 4)
    
    //! Orientation
    static constexpr size_t oX = 0;
    static constexpr size_t oY = 1;
    static constexpr size_t oZ = 2;
    static constexpr size_t oW = 3;
    
    float ox()  const { return (*this)[oX]; }
    float oy()  const { return (*this)[oY]; }
    float oz()  const { return (*this)[oZ]; }
    float ow()  const { return (*this)[oW]; }

    float& ox() { return (*this)[oX]; }
    float& oy() { return (*this)[oY]; }
    float& oz() { return (*this)[oZ]; }
    float& ow() { return (*this)[oW]; }
};

/**
 * @brief Measurement model for measuring orientation of a 3DOF robot
 *
 * This is the measurement model for measuring the orientation of our
 * planar robot. This could be realized by a compass / magnetometer-sensor.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
class OrientationMeasurementModel : public Kalman::MeasurementModel<State, OrientationMeasurement, Kalman::StandardBase>
{
public:    
    OrientationMeasurementModel()
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
    OrientationMeasurement h(const State& x) const
    {
        OrientationMeasurement measurement;
        
        // Measurement is given by the actual robot orientation
        measurement.ox() = x.ox();
        measurement.oy() = x.oy();
        measurement.oz() = x.oz();
        measurement.ow() = x.ow();
        
        return measurement;
    }
};

} // namespace ArgusKalman

#endif