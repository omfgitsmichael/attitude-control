#ifndef FILTER_TYPES_H_
#define FILTER_TYPES_H_

#include <vector>

#include "types/typenames.h"

namespace attitude {

namespace filter {
/**
 * Attitude vector typename
**/
template <typename Scalar>
using AttitudeVector = Eigen::Vector<Scalar, 3>;

/**
 * The delta X update typename
**/
template <typename Scalar, int N>
using DeltaStates = Eigen::Vector<Scalar, N>;

/**
 * The delta X update typename
**/
template <typename Scalar, int N>
using Covariance = Eigen::Matrix<Scalar, N, N>;

/**
 * Kalman Filter attitude measurement structure
**/
template <typename Scalar>
struct AttitudeMeasurement
{
    Scalar sigma = 0.0; /// Attitude measurement measurement noise standard deviation
    AttitudeVector<Scalar> attitudeRefVector = AttitudeVector<Scalar>::Zero();
    AttitudeVector<Scalar> attitudeMeasVector = AttitudeVector<Scalar>::Zero();
};

namespace mekf {

/**
 * Kalman Filter propagation parameters
**/
template <typename Scalar>
struct MEKFParams
{
    Scalar dt = 0.0;
    Scalar omegaProcessNoise = 0.0; /// Angular rate process noise standard deviation
    Scalar biasProcessNoise = 0.0;  /// Angular rate bias process noise standard deviation
};

/**
 * Attitude estimation Kalman Filter data type structure
**/
template <typename Scalar>
struct MEKFData
{
    // Measurements
    std::vector<AttitudeMeasurement<Scalar>> attitudeMeasurements; /// Size is the number of incoming measurement vectors
    BodyRate<Scalar> omegaMeas = BodyRate<Scalar>::Zero();

    // Kalman Filter estimated states
    Quaternion<Scalar> quaternion = Quaternion<Scalar>::Zero();
    BodyRate<Scalar> omega = BodyRate<Scalar>::Zero();
    BodyRate<Scalar> omegaBias = BodyRate<Scalar>::Zero();
    DeltaStates<Scalar, 6> deltaX = DeltaStates<Scalar, 6>::Zero();

    // Covariance matrix
    Covariance<Scalar, 6> P = Covariance<Scalar, 6>::Zero();
};

} // namespace mekf

namespace ahrs {

/**
 * Kalman Filter propagation parameters
**/
template <typename Scalar>
struct AHRSParams
{
    Scalar dt = 0.0;
    Scalar omegaProcessNoise = 0.0; /// Angular rate process noise standard deviation
    Scalar biasProcessNoise = 0.0;  /// Angular rate bias process noise standard deviation
};

/**
 * Attitude estimation Kalman Filter data type structure. TODO::Michael::Make it variable if magnetometer measurements are 'jammed'
**/
template <typename Scalar>
struct AHRSData
{
    // Measurements
    AttitudeMeasurement<Scalar> accelerometerMeas;
    AttitudeMeasurement<Scalar> magnetometerMeas;
    BodyRate<Scalar> omegaMeas = BodyRate<Scalar>::Zero();

    // Kalman Filter estimated states
    Quaternion<Scalar> quaternion = Quaternion<Scalar>::Zero();
    BodyRate<Scalar> omega = BodyRate<Scalar>::Zero();
    BodyRate<Scalar> omegaBias = BodyRate<Scalar>::Zero();
    DeltaStates<Scalar, 12> deltaX = DeltaStates<Scalar, 6>::Zero();

    // Covariance matrix
    Covariance<Scalar, 12> P = Covariance<Scalar, 12>::Zero();
};

} // namespace ahrs

} // namespace filter

} // namespace attitude

#endif // FILTER_TYPES_H_