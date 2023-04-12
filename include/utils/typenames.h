#ifndef TYPENAME_H_
#define TYPENAME_H_

#include <Eigen/Dense>

namespace attitude {

/**
 * Quaternion typename
**/
template <typename Scalar>
using Quaternion = Eigen::Vector<Scalar, 4>;

/**
 * Euler angle typename
**/
template <typename Scalar>
using EulerAngle = Eigen::Vector<Scalar, 3>;

/**
 * Body rate typename
**/
template <typename Scalar>
using BodyRate = Eigen::Vector<Scalar, 3>;

/**
 * Rotation matrix typename
**/
template <typename Scalar>
using RotationMatrix = Eigen::Matrix<Scalar, 3, 3>;

} // namespace attitude

#endif // TYPENAME_H_
