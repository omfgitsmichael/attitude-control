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
template <typename Scalar>
using DeltaStates = Eigen::Vector<Scalar, 6>;

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

/**
 * Kalman Filter propagation parameters
**/
template <typename Scalar>
struct KalmanFilterParams
{
    Scalar dt = 0.0;
    Scalar omegaProcessNoise = 0.0; /// Angular rate process noise standard deviation
    Scalar biasProcessNoise = 0.0;  /// Angular rate bias process noise standard deviation
};

/**
 * Attitude estimation Kalman Filter data type structure
**/
template <typename Scalar>
struct KalmanFilterData
{
    // Measurements
    std::vector<AttitudeMeasurement<Scalar>> attitudeMeasurements; /// Size is the number of incoming measurement vectors
    BodyRate<Scalar> omegaMeas = BodyRate<Scalar>::Zero();

    // Kalman Filter estimated states
    Quaternion<Scalar> quaternion = Quaternion<Scalar>::Zero();
    BodyRate<Scalar> bias = BodyRate<Scalar>::Zero();
    DeltaStates<Scalar> deltaX = DeltaStates<Scalar>::Zero();
};

} // namespace filter

} // namespace attitude

#endif // FILTER_TYPES_H_
