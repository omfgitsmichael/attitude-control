#ifndef AHRS_KALMAN_FILTER_H_
#define AHRS_KALMAN_FILTER_H_

#include "quaternions/quaternionMath.h"
#include "types/filterTypes.h"

namespace attitude {

namespace filter {

namespace ahrs {

// Can have these as params which can be calculateed via calibration
static constexpr double GRAVITY = 9.81; /// m/s^2
static constexpr double GEOMAGNETIC_FIELD_STRENGTH = 0.5; /// gauss

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
    const Eigen::Vector<Scalar, 3> cross1 = accelerometer.attitudeMeasVector.cross(magnetometer.attitudeMeasVector);
    const Eigen::Vector<Scalar, 3> cross2 = cross1.cross(accelerometer.attitudeMeasVector);

    RotationMatrix<Scalar> rotation{cross2, cross1, accelerometer.attitudeMeasVector}; // Not sure if this is populating the matrix as I intend it to
    rotation.col(0) /= rotation.col(0).norm();
    rotation.col(1) /= rotation.col(1).norm();
    rotation.col(2) /= rotation.col(2).norm();

    OptionalQuaternion<Scalar> quat = rotationToQuaternion(rotation);

    if (!quat) {
        return false;
    }

    data.omega = omega;
    data.quaternion = *quat;
    data.magneticVector = rotation.transpose() * magnetometer.attitudeMeasVector;
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

    // Propagate the covariance matrix

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
inline void  accelerometerUpdate(const AHRSParams<Scalar>& params,
                                 const RotationMatrix<Scalar>& rotation,
                                 AHRSData<Scalar>& data)
{
    const Eigen::Matrix<Scalar, 3, 3> identity = Eigen::Matrix<Scalar, 3, 3>::Identity();

    // Measurement and error model
    const AttitudeVector<Scalar> estMeas = GRAVITY * rotation.col(2);
    const AttitudeVector<Scalar> meas = data.accelerometerMeas.attitudeMeasVector + data.linearAccelForces;
    const AttitudeVector<Scalar> residual = meas - estMeas;

    // Calculate the measurement sensitivity matrix
    const Eigen::Matrix<Scalar, 3, 3> measCross{{0.0, -estMeas(2), estMeas(1)},
                                                {estMeas(2), 0.0, -estMeas(0)},
                                                {-estMeas(1), estMeas(0), 0.0}};
    Eigen::Matrix<Scalar, 3, 12> H = Eigen::Matrix<Scalar, 3, 12>::Zero();
    H.block(0, 0, 3, 3) = -measCross;
    H.block(0, 3, 3, 3) = params.dt * measCross;
    H.block(0, 6, 3, 3) = identity;

    // Calculate the Kalman gain
    const Scalar measurementNoise = data.accelerometerMeas.sigma * data.accelerometerMeas.sigma +
        params.linearAccelNoise * params.linearAccelNoise + params.dt * params.dt * (params.biasProcessNoise * params.biasProcessNoise +
        params.omegaProcessNoise * params.omegaProcessNoise);
    const Eigen::Matrix<Scalar, 3, 3> R = measurementNoise * identity;
    Eigen::Matrix<Scalar, 3, 3> S = H * data.P * H.transpose() + R;
    Eigen::Matrix<Scalar, 12, 3> K = data.P * H.transpose() * S.inverse();

    // Update states and costates
    data.deltaX += K * (residual - H * data.deltaX);
    data.P = (Eigen::Matrix<Scalar, 12, 12>::Identity() - K * H) * data.P;
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
inline bool magnetometerUpdate(const AHRSParams<Scalar>& params,
                               const RotationMatrix<Scalar>& rotation,
                               AHRSData<Scalar>& data)
{
    const OptionalRotationMatrix rotation = quaternionRotationMatrix(data.quaternion);
    if (!rotation){
        return false;
    }

    const Eigen::Matrix<Scalar, 3, 3> identity = Eigen::Matrix<Scalar, 3, 3>::Identity();

    // Measurement and error model
    const AttitudeVector<Scalar> estMeas = (*rotation) * data.magneticVector;
    const AttitudeVector<Scalar> meas = data.magnetometerMeas.attitudeMeasVector - data.magneticDisturbances;
    const AttitudeVector<Scalar> residual = meas - estMeas;

    // Calculate the measurement sensitivity matrix
    const Eigen::Matrix<Scalar, 3, 3> measCross{{0.0, -estMeas(2), estMeas(1)},
                                                {estMeas(2), 0.0, -estMeas(0)},
                                                {-estMeas(1), estMeas(0), 0.0}};
    Eigen::Matrix<Scalar, 3, 12> H = Eigen::Matrix<Scalar, 3, 12>::Zero();
    H.block(0, 0, 3, 3) = -measCross;
    H.block(0, 3, 3, 3) = params.dt * measCross;
    H.block(0, 9, 3, 3) = -identity;

    // Calculate the Kalman gain
    const Scalar measurementNoise = data.magnetometerMeas.sigma * data.magnetometerMeas.sigma +
        params.magDisturbanceNoise * params.magDisturbanceNoise + params.dt * params.dt * (params.biasProcessNoise * params.biasProcessNoise +
        params.omegaProcessNoise * params.omegaProcessNoise);
    const Eigen::Matrix<Scalar, 3, 3> R = measurementNoise * identity;
    Eigen::Matrix<Scalar, 3, 3> S = H * data.P * H.transpose() + R;
    Eigen::Matrix<Scalar, 12, 3> K = data.P * H.transpose() * S.inverse();

    // Update states and costates
    data.deltaX += K * (residual - H * data.deltaX);
    data.P = (Eigen::Matrix<Scalar, 12, 12>::Identity() - K * H) * data.P;

    return true;
}


/**
* Update the estimated states for the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm -- variant
* of MEKF. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: params - AHRS params
* Input: data - AHRS data structure
* Output: Boolean if it passsed or failed
**/
template <typename Scalar>
inline bool AHRSKalmanUpdate(const AHRSParams<Scalar>& params, AHRSData<Scalar>& data)
{
    const OptionalRotationMatrix rotation = quaternionRotationMatrix(data.quaternion);
    if (!rotation){
        return false;
    }

    data.deltaX = DeltaStates<Scalar, 12>::Zero();

    // Update the states
    if (data.accelerometerMeas.valid) {
        accelerometerUpdate(params, *rotation, data);
    }

    if (data.magnetometerMeas.valid) {
        magnetometerUpdate(params, *rotation, data);
    }

    // Update the primary states
    data.omegaBias += data.deltaX.segment(3, 3);
    data.linearAccelForces += data.deltaX.segment(6, 3);
    data.magneticDisturbances += data.deltaX.tail(3);
    data.omega = data.omegaMeas - data.omegaBias;

    const Eigen::Matrix<Scalar, 4, 3> E{{data.quaternion(3), -data.quaternion(2), data.quaternion(1)},
                                        {data.quaternion(2), data.quaternion(3), -data.quaternion(0)},
                                        {-data.quaternion(1), data.quaternion(0), data.quaternion(3)},
                                        {-data.quaternion(0), -data.quaternion(1), -data.quaternion(2)}};
    data.quaternion += static_cast<Scalar>(0.5) * E * data.deltaX.head(3);
    data.quaternion /= data.quaternion.norm();

    // Update the magnetic vector if the magnetometer measurement was valid
    if (data.magnetometerMeas.valid) {
        const AttitudeVector<Scalar> disturbanceNED = (*rotation).transpose() * data.magneticDisturbances;
        const AttitudeVector<Scalar> magneticNED = data.magneticVector - disturbanceNED;
        const Scalar inclination = static_cast<Scalar>(std::atan2(magneticNED(2), magneticNED(0)));
        data.magneticVector = GEOMAGNETIC_FIELD_STRENGTH *  AttitudeVector<Scalar>{std::cos(inclination), 0.0, std::sin(inclination)};
    }

    return true;
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
