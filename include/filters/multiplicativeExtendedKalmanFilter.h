#ifndef ATTITUDE_KALMAN_FILTER_H_
#define ATTITUDE_KALMAN_FILTER_H_

#include "quaternions/quaternionMath.h"
#include "types/filterTypes.h"

namespace attitude {

namespace filter {

namespace mekf {

/**
* Initializes the Kalman Filter state estimates. Referenced from "Optimal Estimation of Dynamic Systems" by John L. Crassidis and 
* John L. Junkins, 2nd edition.
* Input:
* Output:
**/
template <typename Scalar>
void multiplicativeExtendedKalmanInitialize()
{

}

/**
* Propagates the state estimates of the Kalman Filter to 'current time' using a discrete time formulation for the quaternion
* and process noise. Referenced from "Optimal Estimation of Dynamic Systems" by John L. Crassidis and John L. Junkins, 2nd edition.
* Input: params - MEKF params
* Input: data - MEKF data structure
**/
template <typename Scalar>
void multiplicativeExtendedKalmanPropagate(const MEKFParams<Scalar>& params, MEKFData<Scalar>& data)
{
    const Scalar dt = params.dt;
    const Eigen::Matrix<Scalar, 3, 3> identity = Eigen::Matrix<Scalar, 3, 3>::Identity();
    const Scalar w = data.omega.norm();
    const Scalar w2 = w * w;
    const Scalar w3 = w * w * w;

    // Propagate the angular velocity
    data.omega = data.omegaMeas - data.omegaBias;

    // Discrete time quaternion propagation
    const Scalar scalarTerm = static_cast<Scalar>(std::cos(0.5 * w * dt));
    const BodyRate<Scalar> psi = (static_cast<Scalar>(std::sin(0.5 * w * dt)) / w) * data.omega;
    const Eigen::Matrix<Scalar, 3, 3> phiCross{{0.0, -psi(2), psi(1)},
                                               {psi(2), 0.0, -psi(0)},
                                               {-psi(1), psi(0), 0.0}};

    Eigen::Matrix<Scalar, 4, 4> omega = Eigen::Matrix<Scalar, 4, 4>::Zero();
    omega.block(0, 0, 3, 3) = scalarTerm * identity - phiCross;
    omega.block(0, 3, 3, 1) = psi;
    omega.block(3, 0, 1, 3) = -psi.transpose();
    omega.block(3, 3, 1, 1) = scalarTerm;

    data.quaternion = omega * data.quaternion;

    // Discrete time covariance propagation
    const Scalar sigmaV2 = params.omegaProcessNoise * params.omegaProcessNoise;
    const Scalar sigmaU2 = params.biasProcessNoise * params.biasProcessNoise;
    const Eigen::Matrix<Scalar, 3, 3> omegaCross{{0.0, -data.omega(2), data.omega(1)},
                                                 {data.omega(2), 0.0, -data.omega(0)},
                                                 {-data.omega(1), data.omega(0), 0.0}};
    const Eigen::Matrix<Scalar, 3, 3> omegaCross2 = omegaCross * omegaCross;

    Eigen::Matrix<Scalar, 6, 6> phi = Eigen::Matrix<Scalar, 6, 6>::Zero();
    phi.block(0, 0, 3, 3) = identity - omegaCross * (static_cast<Scalar>(std::sin(w * dt)) / w)
        + omegaCross2 * ((static_cast<Scalar>(1.0 - std::cos(w * dt))) / w2);
    phi.block(0, 3, 3, 3) = omegaCross * ((static_cast<Scalar>(1.0 - std::cos(w * dt))) / w2)
        - identity * dt - omegaCross2 * ((w * dt - static_cast<Scalar>(std::sin(w * dt))) / w3);
    phi.block(3, 3, 3, 3) = identity;

    Eigen::Matrix<Scalar, 6, 6> gamma = Eigen::Matrix<Scalar, 6, 6>::Zero();
    gamma.block(0, 0, 3, 3) = -identity;
    gamma.block(3, 3, 3, 3) = identity;

    Eigen::Matrix<Scalar, 6, 6> process = Eigen::Matrix<Scalar, 6, 6>::Zero();
    const Eigen::Matrix<Scalar, 3, 3> crossTerms = (static_cast<Scalar>(0.5) * sigmaU2 * dt * dt) * identity;
    process.block(0, 0, 3, 3) = (sigmaV2 * dt + static_cast<Scalar>(1.0 / 3.0) * sigmaU2 * dt * dt * dt) * identity;
    process.block(0, 3, 3, 3) = crossTerms;
    process.block(3, 0, 3, 3) = crossTerms;
    process.block(3, 3, 3, 3) = (sigmaU2 * dt) * identity;

    data.P = phi * data.P * phi.tranpose() + gamma * process * gamma.tranpose();
}

/**
* Update the estimated states using 'N' Attitude measurements. Murrell's version of the multiplicative extended Kalman Filter
* (MEKF) update phase is used for improved computational performance. Referenced from "Optimal Estimation of Dynamic Systems"
* by John L. Crassidis and John L. Junkins, 2nd edition.
* Input: data - MEKF data structure
* Output: Boolean if the Kalman filter passed or failed
**/
template <typename Scalar>
bool multiplicativeExtendedKalmanUpdate(MEKFData<Scalar>& data)
{
    OptionalRotationMatrix<Scalar> rotation = quaternionRotationMatrix(data.quaternion);
    if (!rotation) {
        return false;
    }

    data.deltaX = DeltaStates<Scalar, 6>::Zero();

    // Loop through each of the measurements one at a time for improved computational performance (Murrell's version)
    for (const auto meas : data.attitudeMeasurements) {
        const AttitudeMeasurement<Scalar> estMeas = (*rotation) * meas.attitudeRefVector;
        const Eigen::Matrix<Scalar, 3, 3> measCross{{0.0, -estMeas(2), estMeas(1)}, {estMeas(2), 0.0, -estMeas(0)}, {-estMeas(1), estMeas(0), 0.0}};

        // Calculate the measurement sensitivity matrix
        Eigen::Matrix<Scalar, 3, 6> H = Eigen::Matrix<Scalar, 3, 6>::Zero();
        H.block(0, 0, 3, 3) = measCross;

        // Calculate the Kalman gain
        Eigen::Matrix<Scalar, 3, 3> S = H * data.P * H.transpose() + meas.sigma * meas.sigma * Eigen::Matrix<Scalar, 3, 3>::Identity();
        Eigen::Matrix<Scalar, 6, 3> K = data.P * H.transpose() * S.inverse();

        // Update states and costates
        const AttitudeMeasurement<Scalar> residual = meas.attitudeMeasVector - estMeas;
        data.deltaX += K * (residual - H * data.deltaX);
        data.P = (Eigen::Matrix<Scalar, 6, 6>::Identity() - K * H) * data.P;
    }

    // Update the primary states
    data.omegaBias += data.deltaX.tail(3);

    const Eigen::Matrix<Scalar, 4, 3> E{{data.quaternion(3), -data.quaternion(2), data.quaternion(1)},
                                        {data.quaternion(2), data.quaternion(3), -data.quaternion(0)},
                                        {-data.quaternion(1), data.quaternion(0), data.quaternion(3)},
                                        {-data.quaternion(0), -data.quaternion(1), -data.quaternion(2)}};
    data.quaternion += static_cast<Scalar>(0.5) * E * data.deltaX.head(3);
    data.quaternion /= data.quaternion.norm();

    return true;
}

/**
* Runs the attitude estimation multiplicative extended Kalman Filter (MEKF) algorithm. Referenced from "Optimal Estimation of Dynamic
* Systems" by John L. Crassidis and John L. Junkins, 2nd edition.
* Input: params - MEKF params
* Input: data - MEKF data structure
* Output: Boolean if the Kalman filter passed or failed
**/
template <typename Scalar>
bool multiplicativeExtendedKalmanFilter(const MEKFParams<Scalar>& params, MEKFData<Scalar>& data)
{
    // Propagate the states to 'current time'
    multiplicativeExtendedKalmanPropagate(params, data);

    // Update the states based off the current measurements
    bool result = multiplicativeExtendedKalmanUpdate(data);

    return result;
}

} // namespace mekf

} // namespace filter

} // namespace attitude

#endif // ATTITUDE_KALMAN_FILTER_H_