#ifndef KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>
#include <Eigen/src/Geometry/Quaternion.h>

namespace KalmanExamples
{
namespace Robot1
{

/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 18>
{
public:
    KALMAN_VECTOR(State, T, 18)
    


    // position
    static constexpr size_t pX = 0;
    static constexpr size_t pY = 1;
    static constexpr size_t pZ = 2;

    // velocity
    static constexpr size_t vX = 3;
    static constexpr size_t vY = 4;
    static constexpr size_t vZ = 5;

    // linear acceleration
    static constexpr size_t aX = 6;
    static constexpr size_t aY = 7;
    static constexpr size_t aZ = 8;

    // orientation 3x3 matrix
    static constexpr size_t O11 = 9;
    static constexpr size_t O12 = 10;
    static constexpr size_t O13 = 11;
    static constexpr size_t O21 = 12;
    static constexpr size_t O22 = 13;
    static constexpr size_t O23 = 14;
    static constexpr size_t O31 = 15;
    static constexpr size_t O32 = 16;
    static constexpr size_t O33 = 17;

    T px()       const { return (*this)[pX]; }
    T py()       const { return (*this)[pY]; }
    T pz()       const { return (*this)[pZ]; }

    T vx()       const { return (*this)[vX]; }
    T vy()       const { return (*this)[vY]; }
    T vz()       const { return (*this)[vZ]; }

    T ax()       const { return (*this)[aX]; }
    T ay()       const { return (*this)[aY]; }
    T az()       const { return (*this)[aZ]; }

    T o11()       const { return (*this)[O11]; }
    T o12()       const { return (*this)[O12]; }
    T o13()       const { return (*this)[O13]; }
    T o21()       const { return (*this)[O21]; }
    T o22()       const { return (*this)[O22]; }
    T o23()       const { return (*this)[O23]; }
    T o31()       const { return (*this)[O31]; }
    T o32()       const { return (*this)[O32]; }
    T o33()       const { return (*this)[O33]; }

    T& px() { return (*this)[pX]; }
    T& py() { return (*this)[pY]; }
    T& pz() { return (*this)[pZ]; }

    T& vx() { return (*this)[vX]; }
    T& vy() { return (*this)[vY]; }
    T& vz() { return (*this)[vZ]; }

    T& ax() { return (*this)[aX]; }
    T& ay() { return (*this)[aY]; }
    T& az() { return (*this)[aZ]; }

    T& o11() { return (*this)[O11]; }
    T& o12() { return (*this)[O12]; }
    T& o13() { return (*this)[O13]; }
    T& o21() { return (*this)[O21]; }
    T& o22() { return (*this)[O22]; }
    T& o23() { return (*this)[O23]; }
    T& o31() { return (*this)[O31]; }
    T& o32() { return (*this)[O32]; }
    T& o33() { return (*this)[O33]; }

    float timestep = 0.01f;

    void initialize() {
        setZero();

        // quaternion identity is 0, 0, 0, 1
        //ow() = 1;
        setQuat(*this, Eigen::Quaternion<float>::Identity());
    }

    static const Eigen::Matrix3f getMatrix(const State<T> state) {
        Eigen::Matrix3f o;
        o << state.o11(), state.o12(), state.o13(), state.o21(), state.o22(), state.o23(), state.o31(), state.o32(), state.o33();
        return o;
    }

    static const Eigen::Quaternion<float> getQuat(const State<T> state) {
        return Eigen::Quaternion<float>(getMatrix(state));
    }

    static void setMatrix(State<T> state, Eigen::Matrix3f m) {
        state.o11() = (float)m(0, 0);
        state.o12() = (float)m(0, 1);
        state.o13() = (float)m(0, 2);
        state.o21() = (float)m(1, 0);
        state.o22() = (float)m(1, 1);
        state.o23() = (float)m(1, 2);
        state.o31() = (float)m(2, 0);
        state.o32() = (float)m(2, 1);
        state.o33() = (float)m(2, 2);
    }


    static void setQuat(State<T> state, Eigen::Quaternion<float> q) {
        setMatrix(state, Eigen::Matrix3f(q));
    }
};

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * This is the system control-input of a very simple planar robot that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(Control, T, 2)
    
    //! Velocity
    static constexpr size_t V = 0;
    //! Angular Rate (Orientation-change)
    static constexpr size_t DTHETA = 1;
    
    T v()       const { return (*this)[ V ]; }
    T dtheta()  const { return (*this)[ DTHETA ]; }
    
    T& v()      { return (*this)[ V ]; }
    T& dtheta() { return (*this)[ DTHETA ]; }
};

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::SystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExamples::Robot1::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExamples::Robot1::Control<T> C;
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;

        auto quat = S::getQuat(x);
        auto vec = Eigen::Vector3f(0, 0, 1);
        auto angleAxis = Eigen::AngleAxis<float>(u.dtheta(), vec);
        auto rotatingQuat = Eigen::Quaternion<float>::Quaternion(angleAxis);
        auto rotatedQuat = (rotatingQuat * quat).normalized();
        
        /*// New orientation given by old orientation plus orientation change
        auto newOrientation = x.theta() + u.dtheta();
        // Re-scale orientation to [-pi/2 to +pi/2]
        
        x_.theta() = newOrientation;*/

        S::setQuat(x, rotatedQuat);
        
        // New x-position given by old x-position plus change in x-direction
        // Change in x-direction is given by the cosine of the (new) orientation
        // times the velocity

        auto forward = Eigen::Vector3f(0, 1, 0);
        auto forwardRotated = quat.normalized() * forward;

        x_.px() = x.px() + x.vx() * x.timestep;
        x_.py() = x.py() + x.vy() * x.timestep;
        x_.pz() = x.pz() + x.vz() * x.timestep;

        x_.vx() = x.vx() + x.ax() * x.timestep;
        x_.vy() = x.vy() + x.ay() * x.timestep;
        x_.vz() = x.vz() + x.az() * x.timestep;

        x_.ax() = x.ax() + (forwardRotated.x() * u.v() - x.vx()) * x.timestep * 100.0f;
        x_.ay() = x.ay() + (forwardRotated.y() * u.v() - x.vy()) * x.timestep * 100.0f;
        x_.az() = x.az() + (forwardRotated.z() * u.v() - x.vz()) * x.timestep * 100.0f;
        
        // Return transitioned state vector
        return x_;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif