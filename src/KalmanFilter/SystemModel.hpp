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
class State : public Kalman::Vector<T, 13>
{
public:
    KALMAN_VECTOR(State, T, 13)
    


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

    // orientation quaternion
    static constexpr size_t oX = 9;
    static constexpr size_t oY = 10;
    static constexpr size_t oZ = 11;
    static constexpr size_t oW = 12;

    T px()       const { return (*this)[pX]; }
    T py()       const { return (*this)[pY]; }
    T pz()       const { return (*this)[pZ]; }

    T vx()       const { return (*this)[vX]; }
    T vy()       const { return (*this)[vY]; }
    T vz()       const { return (*this)[vZ]; }

    T ax()       const { return (*this)[aX]; }
    T ay()       const { return (*this)[aY]; }
    T az()       const { return (*this)[aZ]; }

    T ox()       const { return (*this)[oX]; }
    T oy()       const { return (*this)[oY]; }
    T oz()       const { return (*this)[oZ]; }
    T ow()       const { return (*this)[oW]; }

    T& px() { return (*this)[pX]; }
    T& py() { return (*this)[pY]; }
    T& pz() { return (*this)[pZ]; }

    T& vx() { return (*this)[vX]; }
    T& vy() { return (*this)[vY]; }
    T& vz() { return (*this)[vZ]; }

    T& ax() { return (*this)[aX]; }
    T& ay() { return (*this)[aY]; }
    T& az() { return (*this)[aZ]; }

    T& ox() { return (*this)[oX]; }
    T& oy() { return (*this)[oY]; }
    T& oz() { return (*this)[oZ]; }
    T& ow() { return (*this)[oW]; }

    void initialize() {
        setZero();

        // quaternion identity is 0, 0, 0, 1
        ow() = 1;
    }

    Eigen::Quaternion<float> const getQuat() {
        const float w = ow();
        const float x = ox();
        const float y = oy();
        const float z = oz();
        return Eigen::Quaternion<float>(w, x, y, z);
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

        const float ox = x.ox();
        const float oy = x.oy();
        const float oz = x.oz();
        const float ow = x.ow();
        auto quat = Eigen::Quaternion<float>(ow, ox, oy, oz);
        auto vec = Eigen::Vector3f(0, 0, 1);
        auto angleAxis = Eigen::AngleAxis<float>(u.dtheta(), vec);
        auto rotatingQuat = Eigen::Quaternion<float>::Quaternion(angleAxis);
        auto rotatedQuat = (rotatingQuat * quat).normalized();
        
        /*// New orientation given by old orientation plus orientation change
        auto newOrientation = x.theta() + u.dtheta();
        // Re-scale orientation to [-pi/2 to +pi/2]
        
        x_.theta() = newOrientation;*/

        x_.ox() = rotatedQuat.x();
        x_.oy() = rotatedQuat.y();
        x_.oz() = rotatedQuat.z();
        x_.ow() = rotatedQuat.w();
        
        // New x-position given by old x-position plus change in x-direction
        // Change in x-direction is given by the cosine of the (new) orientation
        // times the velocity

        auto forward = Eigen::Vector3f(0, 1, 0);
        auto forwardRotated = quat.normalized() * forward;

        x_.px() = x.px() + x.vx();
        x_.py() = x.py() + x.vy();
        x_.pz() = x.pz() + x.vz();

        x_.vx() = x.vx() + x.ax();
        x_.vy() = x.vy() + x.ay();
        x_.vz() = x.vz() + x.az();

        x_.ax() = x.ax() * 0.5f + forwardRotated.x() * u.v() / 100.0f;
        x_.ay() = x.ay() * 0.5f + forwardRotated.y() * u.v() / 100.0f;
        x_.az() = x.az() * 0.5f + forwardRotated.z() * u.v() / 100.0f;
        
        // Return transitioned state vector
        return x_;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif