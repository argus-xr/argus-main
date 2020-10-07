#ifndef KALMAN_EXAMPLES_ROBOT1_ROTATIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_ROTATIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include <Eigen/src/Geometry/Quaternion.h>

namespace KalmanExamples
{
namespace Robot1
{

/**
 * @brief Measurement vector measuring an orientation (i.e. by using a compass)
 *
 * @param T Numeric scalar type
 */
template<typename T>
class RotationMeasurement : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(RotationMeasurement, T, 3)

    // orientation 3x3 matrix
    static constexpr size_t rX = 0;
    static constexpr size_t rY = 1;
    static constexpr size_t rZ = 2;

    T rx()       const { return (*this)[rX]; }
    T ry()       const { return (*this)[rY]; }
    T rz()       const { return (*this)[rZ]; }

    T& rx() { return (*this)[rX]; }
    T& ry() { return (*this)[rY]; }
    T& rz() { return (*this)[rZ]; }
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
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class RotationMeasurementModel : public Kalman::MeasurementModel<State<T>, RotationMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef KalmanExamples::Robot1::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExamples::Robot1::RotationMeasurement<T> M;
    
    RotationMeasurementModel()
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
    M h(const S& x) const
    {
        M measurement;

        measurement.rx() = x.rx();
        measurement.ry() = x.ry();
        measurement.rz() = x.rz();
        
        return measurement;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif