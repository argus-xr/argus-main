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
class OrientationMeasurement : public Kalman::Vector<T, 9>
{
public:
    KALMAN_VECTOR(OrientationMeasurement, T, 9)

    // orientation 3x3 matrix
    static constexpr size_t O11 = 0;
    static constexpr size_t O12 = 1;
    static constexpr size_t O13 = 2;
    static constexpr size_t O21 = 3;
    static constexpr size_t O22 = 4;
    static constexpr size_t O23 = 5;
    static constexpr size_t O31 = 6;
    static constexpr size_t O32 = 7;
    static constexpr size_t O33 = 8;

protected:
    T o11()       const { return (*this)[O11]; }
    T o12()       const { return (*this)[O12]; }
    T o13()       const { return (*this)[O13]; }
    T o21()       const { return (*this)[O21]; }
    T o22()       const { return (*this)[O22]; }
    T o23()       const { return (*this)[O23]; }
    T o31()       const { return (*this)[O31]; }
    T o32()       const { return (*this)[O32]; }
    T o33()       const { return (*this)[O33]; }

    T& o11() { return (*this)[O11]; }
    T& o12() { return (*this)[O12]; }
    T& o13() { return (*this)[O13]; }
    T& o21() { return (*this)[O21]; }
    T& o22() { return (*this)[O22]; }
    T& o23() { return (*this)[O23]; }
    T& o31() { return (*this)[O31]; }
    T& o32() { return (*this)[O32]; }
    T& o33() { return (*this)[O33]; }

public:
    static const Eigen::Matrix3f getMatrix(const OrientationMeasurement<T> om) {
        Eigen::Matrix3f o;
        o << om.o11() / matMult, om.o12() / matMult, om.o13() / matMult, om.o21() / matMult, om.o22() / matMult, om.o23() / matMult, om.o31() / matMult, om.o32() / matMult, om.o33() / matMult;
        return o;
    }

    static const Eigen::Quaternion<float> getQuat(const OrientationMeasurement<T> om) {
        return Eigen::Quaternion<float>(getMatrix(om));
    }

    static void setMatrix(OrientationMeasurement<T> &om, Eigen::Matrix3f m) {
        om.o11() = (float)m(0, 0) * matMult;
        om.o12() = (float)m(0, 1) * matMult;
        om.o13() = (float)m(0, 2) * matMult;
        om.o21() = (float)m(1, 0) * matMult;
        om.o22() = (float)m(1, 1) * matMult;
        om.o23() = (float)m(1, 2) * matMult;
        om.o31() = (float)m(2, 0) * matMult;
        om.o32() = (float)m(2, 1) * matMult;
        om.o33() = (float)m(2, 2) * matMult;
    }


    static void setQuat(OrientationMeasurement<T> &om, Eigen::Quaternion<float> q) {
        setMatrix(om, Eigen::Matrix3f(q));
    }

    static const inline float matMult = 2.0f;
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
        Eigen::Matrix3f m = S::getMatrix(x).normalized();

        M::setMatrix(measurement, m);
        
        return measurement;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif