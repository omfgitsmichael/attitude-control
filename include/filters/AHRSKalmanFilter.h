#ifndef AHRS_KALMAN_FILTER_H_
#define AHRS_KALMAN_FILTER_H_

#include "quaternions/quaternionMath.h"
#include "types/filterTypes.h"

namespace attitude {

namespace filter {

namespace ahrs {

/**
* Initializes the Kalman Filter state estimates using accelerometer and magnetometer measurements forming an E-compass. Referenced
* from Mathworks: https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: accelerometer - The accelerometer measurement in the body frame referenced from NED frame
* Input: magnetometer - The magnetometer measurement in the body frame referenced from NED frame
* Output:
**/
template <typename Scalar>
bool AHRSKalmanInitialize(const AttitudeMeasurement<Scalar>& accelerometer,
                          const AttitudeMeasurement<Scalar>& magnetometer,
                          AHRSData<Scalar>& data)
{
    const Eigen::Vector<Scalar, 3> cross1 = accelerometer.cross(magnetometer);
    const Eigen::Vector<Scalar, 3> cross2 = cross1.cross(accelerometer);

    RotationMatrix<Scalar> rotation{cross2, cross1, accelerometer};
    rotation.col(0) /= rotation.col(0).norm();
    rotation.col(1) /= rotation.col(1).norm();
    rotation.col(2) /= rotation.col(2).norm();

    OptionalQuaternion<Scalar> quat = rotationToQuaternion(rotation);

    if (!quat) {
        return false;
    }

    data.quaternion = *quat;
    return true;
}

/**
* Propagates the state estimates of the Kalman Filter to 'current time' using a discrete time formulation for the quaternion
* and process noise. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input:
* Output:
**/
template <typename Scalar>
void AHRSKalmanPropagate()
{

}

/**
* Update the estimated states for the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm -- variant
* of MEKF. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input:
* Output:
**/
template <typename Scalar>
void AHRSKalmanUpdate()
{

}

/**
* Runs the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm -- variant of MEKF. Referenced
* from Mathworks: https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input:
* Output:
**/
template <typename Scalar>
void AHRSKalmanFilter()
{

}

} // namespace ahrs

} // namespace filter

} // namespace attitude

#endif // AHRS_KALMAN_FILTER_H_
