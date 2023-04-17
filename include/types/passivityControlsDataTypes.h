#ifndef PASSIVITY_CONTROLS_DATA_TYPES_H_
#define PASSIVITY_CONTROLS_DATA_TYPES_H_

#include <memory>

#include "types/typenames.h"
#include "utils/controlUtils.h"

namespace attitude {

namespace control {
/**
 * System parameter estimate typename
**/
template <typename Scalar>
using Theta = Eigen::Vector<Scalar, 3>;

/**
* The passivity-based adaptive control paramters
**/
template <typename Scalar>
struct PassivityParams
{
    Scalar dt = 0;
    Eigen::Matrix<Scalar, 3, 3> lambda = Eigen::Matrix<Scalar, 3, 3>::Zero();   /// Diaginal matrix
    Eigen::Matrix<Scalar, 3, 3> k = Eigen::Matrix<Scalar, 3, 3>::Zero();        /// Diaginal matrix
    Eigen::Matrix<Scalar, 3, 3> gammaInv = Eigen::Matrix<Scalar, 3, 3>::Zero(); /// Diaginal matrix
    DeadzoneParams<Scalar> deadzoneParams;                                      /// Deadzone parameters
    ProjectionParams<Scalar, 1> projectionParams;                               /// Projection parameters
};

} // namespace control

/**
 * Passivity-based adpative control data type structure
**/
template <typename Scalar>
struct PassivityControlData {
    // State data
    Quaternion<Scalar> quat = Quaternion<Scalar>::Zero();
    Quaternion<Scalar> quatDes = Quaternion<Scalar>::Zero();
    BodyRate<Scalar> omega = BodyRate<Scalar>::Zero();
    BodyRate<Scalar> omegaDesired = BodyRate<Scalar>::Zero();
    BodyRate<Scalar> omegaDotDesired = BodyRate<Scalar>::Zero();

    // Controller data
    control::Theta<Scalar> theta = control::Theta<Scalar>::Zero();
    Control<Scalar> u = Control<Scalar>::Zero();
};

/**
 * Shared pointer to the passivity-based adaptive controls data structure
**/
template <typename Scalar>
using sPassivityControlData = std::shared_ptr<PassivityControlData<Scalar>>;

} // namespace attitude

#endif // PASSIVITY_CONTROLS_DATA_TYPES_H_
