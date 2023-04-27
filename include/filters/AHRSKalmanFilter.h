#ifndef AHRS_KALMAN_FILTER_H_
#define AHRS_KALMAN_FILTER_H_

#include "quaternions/quaternionMath.h"
#include "types/filterTypes.h"

namespace attitude {

namespace filter {

namespace ahrs {

static constexpr double GRAVITY = 9.81; /// m/s^2

/**
* Initializes the Kalman Filter state estimates using accelerometer and magnetometer measurements forming an E-compass. Referenced
* from Mathworks: https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: accelerometer - The accelerometer measurement in the body frame referenced from NED frame
* Input: magnetometer - The magnetometer measurement in the body frame referenced from NED frame
* Input: omega - The bodys measured angular rates
* Output: Boolean if it passed or failed
**/
template <typename Scalar>
inline bool AHRSKalmanInitialize(const AttitudeMeasurement<Scalar>& accelerometer,
                                 const AttitudeMeasurement<Scalar>& magnetometer,
                                 const BodyRate<Scalar>& omega,
                                 AHRSData<Scalar>& data)
{
    const Eigen::Vector<Scalar, 3> cross1 = accelerometer.cross(magnetometer);
    const Eigen::Vector<Scalar, 3> cross2 = cross1.cross(accelerometer);

    RotationMatrix<Scalar> rotation{cross2, cross1, accelerometer}; // Not sure if this is populating the matrix as I intend it to
    rotation.col(0) /= rotation.col(0).norm();
    rotation.col(1) /= rotation.col(1).norm();
    rotation.col(2) /= rotation.col(2).norm();

    OptionalQuaternion<Scalar> quat = rotationToQuaternion(rotation);

    if (!quat) {
        return false;
    }

    data.omega = omega;
    data.quaternion = *quat;
    return true;
}

/**
* Propagates the state estimates of the Kalman Filter to 'current time' using a discrete time formulation for the quaternion
* and process noise. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: params - AHRS params
* Input: data - AHRS data structure
* Output: Boolean if it passsed or failed
**/
template <typename Scalar>
inline void AHRSKalmanPropagate(const AHRSParams<Scalar>& params, AHRSData<Scalar>& data)
{
    // First propagate the angular velocity
    data.omega = data.omegaMeas - data.omegaBias;

    const Scalar dt = params.dt;
    const Eigen::Matrix<Scalar, 3, 3> identity = Eigen::Matrix<Scalar, 3, 3>::Identity();
    const Scalar w = data.omega.norm();

    // Discrete time quaternion propagation
    Eigen::Matrix<Scalar, 4, 4> omega = Eigen::Matrix<Scalar, 4, 4>::Identity();

    if (w != static_cast<Scalar>(0.0)){
        const Scalar scalarTerm = static_cast<Scalar>(std::cos(0.5 * w * dt));
        const BodyRate<Scalar> psi = (static_cast<Scalar>(std::sin(0.5 * w * dt)) / w) * data.omega;
        const Eigen::Matrix<Scalar, 3, 3> phiCross{{0.0, -psi(2), psi(1)},
                                                {psi(2), 0.0, -psi(0)},
                                                {-psi(1), psi(0), 0.0}};
        
        omega.block(0, 0, 3, 3) = scalarTerm * identity - phiCross;
        omega.block(0, 3, 3, 1) = psi;
        omega.block(3, 0, 1, 3) = -psi.transpose();
        omega(3, 3) = scalarTerm;
    }

    data.quaternion = omega * data.quaternion;
}

/**
* Update the estimated states for the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm using
* accelerometer measurements -- variant of MEKF. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: params - AHRS params
* Input: data - AHRS data structure
* Output: Boolean if it passsed or failed
**/
template <typename Scalar>
inline bool accelerometerUpdate(const AHRSParams<Scalar>& params, AHRSData<Scalar>& data) {
    const OptionalRotationMatrix rotation = quaternionRotationMatrix(data.quaternion);
    if (!rotation){
        return false;
    }

    data.gravityEstimated = GRAVITY * (*rotation).col(2);

    return true;
}

/**
* Update the estimated states for the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm using
* magnetometer measurements -- variant of MEKF. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: params - AHRS params
* Input: data - AHRS data structure
* Output: Boolean if it passsed or failed
**/
template <typename Scalar>
inline bool magnetometerUpdate(const AHRSParams<Scalar>& params, AHRSData<Scalar>& data) {
    const OptionalRotationMatrix rotation = quaternionRotationMatrix(data.quaternion);
    if (!rotation){
        return false;
    }

    data.gravityEstimated = (*rotation) * data.magneticEstimated;

    return true;
}


/**
* Update the estimated states for the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm -- variant
* of MEKF. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input:
* Output:
**/
template <typename Scalar>
inline void AHRSKalmanUpdate()
{

}

/**
* Runs the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm -- variant of MEKF. Referenced
* from Mathworks: https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input:
* Output:
**/
template <typename Scalar>
inline void AHRSKalmanFilter()
{

}

} // namespace ahrs

} // namespace filter

} // namespace attitude

#endif // AHRS_KALMAN_FILTER_H_
