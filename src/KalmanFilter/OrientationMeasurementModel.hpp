#ifndef KALMAN_EXAMPLES_ROBOT1_ORIENTATIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_ORIENTATIONMEASUREMENTMODEL_HPP_

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
class OrientationMeasurement : public Kalman::Vector<T, 4>
{
public:
    KALMAN_VECTOR(OrientationMeasurement, T, 4)
    
    //! Orientation
    static constexpr size_t oX = 0;
    static constexpr size_t oY = 0;
    static constexpr size_t oZ = 0;
    static constexpr size_t oW = 0;

    T ox()  const { return (*this)[oX]; }
    T oy()  const { return (*this)[oY]; }
    T oz()  const { return (*this)[oZ]; }
    T ow()  const { return (*this)[oW]; }

    T& ox() { return (*this)[oX]; }
    T& oy() { return (*this)[oY]; }
    T& oz() { return (*this)[oZ]; }
    T& ow() { return (*this)[oW]; }
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
class OrientationMeasurementModel : public Kalman::MeasurementModel<State<T>, OrientationMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef KalmanExamples::Robot1::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExamples::Robot1::OrientationMeasurement<T> M;
    
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
    M h(const S& x) const
    {
        M measurement;
        
        // Measurement is given by the actual robot orientation
        const float ow = x.ow();
        const float ox = x.ox();
        const float oy = x.oy();
        const float oz = x.oz();
        auto quat = Eigen::Quaternion<float>(ow, ox, oy, oz);
        quat.normalize();

        measurement.ox() = quat.x();
        measurement.oy() = quat.y();
        measurement.oz() = quat.z();
        measurement.ow() = quat.w();
        
        return measurement;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif