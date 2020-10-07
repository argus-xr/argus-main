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
class State : public Kalman::Vector<T, 16>
{
public:
    KALMAN_VECTOR(State, T, 16)
    


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

    // angular velocity
    static constexpr size_t rX = 9;
    static constexpr size_t rY = 10;
    static constexpr size_t rZ = 11;

    // orientation 3x3 matrix
    /*static constexpr size_t O11 = 12;
    static constexpr size_t O12 = 13;
    static constexpr size_t O13 = 14;
    static constexpr size_t O21 = 15;
    static constexpr size_t O22 = 16;
    static constexpr size_t O23 = 17;
    static constexpr size_t O31 = 18;
    static constexpr size_t O32 = 19;
    static constexpr size_t O33 = 20;*/

    // orientation quaternion
    static constexpr size_t oW = 12;
    static constexpr size_t oX = 13;
    static constexpr size_t oY = 14;
    static constexpr size_t oZ = 15;

    T px()       const { return (*this)[pX]; }
    T py()       const { return (*this)[pY]; }
    T pz()       const { return (*this)[pZ]; }

    T vx()       const { return (*this)[vX]; }
    T vy()       const { return (*this)[vY]; }
    T vz()       const { return (*this)[vZ]; }

    T ax()       const { return (*this)[aX]; }
    T ay()       const { return (*this)[aY]; }
    T az()       const { return (*this)[aZ]; }

    T rx()       const { return (*this)[rX]; }
    T ry()       const { return (*this)[rY]; }
    T rz()       const { return (*this)[rZ]; }
    
    T& px() { return (*this)[pX]; }
    T& py() { return (*this)[pY]; }
    T& pz() { return (*this)[pZ]; }

    T& vx() { return (*this)[vX]; }
    T& vy() { return (*this)[vY]; }
    T& vz() { return (*this)[vZ]; }

    T& ax() { return (*this)[aX]; }
    T& ay() { return (*this)[aY]; }
    T& az() { return (*this)[aZ]; }

    T& rx() { return (*this)[rX]; }
    T& ry() { return (*this)[rY]; }
    T& rz() { return (*this)[rZ]; }

protected:
    /*T o11()       const { return (*this)[O11]; }
    T o12()       const { return (*this)[O12]; }
    T o13()       const { return (*this)[O13]; }
    T o21()       const { return (*this)[O21]; }
    T o22()       const { return (*this)[O22]; }
    T o23()       const { return (*this)[O23]; }
    T o31()       const { return (*this)[O31]; }
    T o32()       const { return (*this)[O32]; }
    T o33()       const { return (*this)[O33]; }*/

    T ow()       const { return (*this)[oW]; }
    T ox()       const { return (*this)[oX]; }
    T oy()       const { return (*this)[oY]; }
    T oz()       const { return (*this)[oZ]; }

    /*T& o11() { return (*this)[O11]; }
    T& o12() { return (*this)[O12]; }
    T& o13() { return (*this)[O13]; }
    T& o21() { return (*this)[O21]; }
    T& o22() { return (*this)[O22]; }
    T& o23() { return (*this)[O23]; }
    T& o31() { return (*this)[O31]; }
    T& o32() { return (*this)[O32]; }
    T& o33() { return (*this)[O33]; }*/

    T& ow() { return (*this)[oW]; }
    T& ox() { return (*this)[oX]; }
    T& oy() { return (*this)[oY]; }
    T& oz() { return (*this)[oZ]; }

public:
    T timestep = T(0.01);

    void initialize() {
        setZero();

        // quaternion identity is 0, 0, 0, 1
        //ow() = 1;
        setQuat(*this, Eigen::Quaternion<T>::Identity());
    }

    /*static const Eigen::Matrix3f getMatrix(const State<T> state) {
        Eigen::Matrix3f o;
        o << state.o11(), state.o12(), state.o13(), state.o21(), state.o22(), state.o23(), state.o31(), state.o32(), state.o33();
        return o;
    }

    static const Eigen::Quaternion<float> getQuat(const State<T> state) {
        return Eigen::Quaternion<float>(getMatrix(state));
    }

    static void setMatrix(State<T> &state, Eigen::Matrix3f m) {
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


    static void setQuat(State<T> &state, Eigen::Quaternion<float> q) {
        setMatrix(state, Eigen::Matrix3f(q));
    }*/

    static const Eigen::Matrix3d getMatrix(const State<T> state) {
        return Eigen::Matrix3d(getQuat(state));
    }

    static const Eigen::Quaternion<T> getQuat(const State<T> state) {
        return Eigen::Quaternion<T>(state.ow() / quatMult, state.ox() / quatMult, state.oy() / quatMult, state.oz() / quatMult);
    }

    static void setMatrix(State<T> &state, Eigen::Matrix3d m) {
        setQuat(state, Eigen::Quaternion<T>(m));
    }

    static void setQuat(State<T> &state, Eigen::Quaternion<T> q) {
        state.ow() = q.w() * quatMult;
        state.ox() = q.x() * quatMult;
        state.oy() = q.y() * quatMult;
        state.oz() = q.z() * quatMult;
    }

    static const inline T quatMult = T(10.0);
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
class Control : public Kalman::Vector<T, 0>
{
public:
    KALMAN_VECTOR(Control, T, 0)
    
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
        
        /*// New orientation given by old orientation plus orientation change
        auto newOrientation = x.theta() + u.dtheta();
        // Re-scale orientation to [-pi/2 to +pi/2]
        
        x_.theta() = newOrientation;*/

        S::setQuat(x_, quat.normalized());
        
        // New x-position given by old x-position plus change in x-direction
        // Change in x-direction is given by the cosine of the (new) orientation
        // times the velocity

        x_.px() = x.px() + (x.vx()/* + x.ax() * 0.5f*/) * x.timestep;
        x_.py() = x.py() + (x.vy()/* + x.ay() * 0.5f*/) * x.timestep;
        x_.pz() = x.pz() + (x.vz()/* + x.az() * 0.5f*/) * x.timestep;

        x_.vx() = x.vx() + x.ax() * x.timestep;
        x_.vy() = x.vy() + x.ay() * x.timestep;
        x_.vz() = x.vz() + x.az() * x.timestep;

        x_.ax() = x.ax();
        x_.ay() = x.ay();
        x_.az() = x.az();
        
        x_.rx() = x.rx();
        x_.ry() = x.ry();
        x_.rz() = x.rz();

        // Return transitioned state vector
        return x_;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif