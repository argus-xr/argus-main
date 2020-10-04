#ifndef ARGUSACCELERATIONMEASUREMENTMODEL_HPP_
#define ARGUSACCELERATIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"

namespace KalmanExamples
{
namespace Robot1
{

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template<typename T>
class AccelerationMeasurement : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(AccelerationMeasurement, T, 3)

    //! Acceleration
    static constexpr size_t aX = 0;
    static constexpr size_t aY = 1;
    static constexpr size_t aZ = 2;

    T ax()  const { return (*this)[aX]; }
    T ay()  const { return (*this)[aY]; }
    T az()  const { return (*this)[aZ]; }

    T& ax() { return (*this)[aX]; }
    T& ay() { return (*this)[aY]; }
    T& az() { return (*this)[aZ]; }
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
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class AccelerationMeasurementModel : public Kalman::MeasurementModel<State<T>, AccelerationMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef KalmanExamples::Robot1::State<T> S;

    //! Measurement type shortcut definition
    typedef  KalmanExamples::Robot1::AccelerationMeasurement<T> M;
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
    M h(const S& x) const
    {
        M measurement;

        // Acceleartion is given by the actual robot acceleration
        measurement.ax() = x.ax();
        measurement.ay() = x.ay();
        measurement.az() = x.az();
        
        return measurement;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif