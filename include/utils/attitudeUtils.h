#ifndef ATTITUDE_UTILS_H_
#define ATTITUDE_UTILS_H_

#include <string>
#include <cmath>

#include <iostream>

#include "utils/typenames.h"

namespace attitude {

/**
 * Converts euler angles into the desired rotation matrix. Referenced from Analytics of Space Systems by Hanspeter Schaub and
 * John L. Junkins, Pages 759 - 760
 * Input: sequence - String which determines the rotation sequence
 * Input: theta - The Euler angles
 * output: Rotation matrix
**/
template <typename Scalar>
inline RotationMatrix<Scalar> eulerRotationMatrix(const std::string& sequence, const EulerAngle<Scalar>& theta)
{
    const Scalar c1 = static_cast<Scalar>(std::cos(theta(0)));
    const Scalar c2 = static_cast<Scalar>(std::cos(theta(1)));
    const Scalar c3 = static_cast<Scalar>(std::cos(theta(2)));
    const Scalar s1 = static_cast<Scalar>(std::sin(theta(0)));
    const Scalar s2 = static_cast<Scalar>(std::sin(theta(1)));
    const Scalar s3 = static_cast<Scalar>(std::sin(theta(2)));

    RotationMatrix<Scalar> rotation = RotationMatrix<Scalar>::Zero();

    if (sequence.compare("121") == 0) {
        rotation(0, 0) = c2;
        rotation(0, 1) = s2 * s1;
        rotation(0, 2) = -s2 * c1;

        rotation(1, 0) = s3 * s2;
        rotation(1, 1) = -s3 * c2 * s1 + c3 * c1;
        rotation(1, 2) = s3 * c2 * c1 + c3 * s1;

        rotation(2, 0) = c3 * s2;
        rotation(2, 1) = -(c3 * c2 * s1 + s3 * c1);
        rotation(2, 2) = c3 * c2 * c1 - s3 * s1;
    } else if (sequence.compare("123") == 0) {
        rotation(0, 0) = c3 * c2;
        rotation(0, 1) = c3 * s2 * s1 + s3 * c1;
        rotation(0, 2) = -c3 * s2 * c1 + s3 * s1;

        rotation(1, 0) = -s3 * c2;
        rotation(1, 1) = -s3 * s2 * s1 + c3 * c1;
        rotation(1, 2) = s3 * s2 * c1 + c3 * s1;

        rotation(2, 0) = s2;
        rotation(2, 1) = -c2 * s1;
        rotation(2, 2) = c2 * c1;
    } else if (sequence.compare("131") == 0) {
        rotation(0, 0) = c2;
        rotation(0, 1) = s2 * c1;
        rotation(0, 2) = s2 * s1;

        rotation(1, 0) = -c3 * s2;
        rotation(1, 1) = c3 * c2 * c1 - s3 * s1;
        rotation(1, 2) = c3 * c2 * s1 + s3 * c1;

        rotation(2, 0) = s3 * s2;
        rotation(2, 1) = -(s3 * c2 * c1 + c3 * s1);
        rotation(2, 2) = -s3 * c2 * s1 + c3 * c1;
    } else if (sequence.compare("132") == 0) {
        rotation(0, 0) = c3 * c2;
        rotation(0, 1) = c3 * s2 * c1 + s3 * s1;
        rotation(0, 2) = c3 * s2 * s1 - s3 * c1;

        rotation(1, 0) = -s2;
        rotation(1, 1) = c2 * c1;
        rotation(1, 2) = c2 * s1;

        rotation(2, 0) = s3 * c2;
        rotation(2, 1) = s3 * s2 * c1 - c3 * s1;
        rotation(2, 2) = s3 * s2 * s1 + c3 * c1;
    } else if (sequence.compare("212") == 0) {
        rotation(0, 0) = -s3 * c2 * s1 + c3 * c1;
        rotation(0, 1) = s3 * s2;
        rotation(0, 2) = -(s3 * c2 * c1 + c3 * s1);

        rotation(1, 0) = s2 * s1;
        rotation(1, 1) = c2;
        rotation(1, 2) = s2 * c1;

        rotation(2, 0) = c3 * c2 * s1 + s3 * c1;
        rotation(2, 1) = -c3 * s2;
        rotation(2, 2) = c3 * c2 * c1 - s3 * s1;
    } else if (sequence.compare("213") == 0) {
        rotation(0, 0) = s3 * s2 * s1 + c3 * c1;
        rotation(0, 1) = s3 * c2;
        rotation(0, 2) = s3 * s2 * c1 - c3 * s1;

        rotation(1, 0) = c3 * s2 * s1 - s3 * c1;
        rotation(1, 1) = c3 * c2;
        rotation(1, 2) = c3 * s2 * c1 + s3 * s1;

        rotation(2, 0) = c2 * s1;
        rotation(2, 1) = -s2;
        rotation(2, 2) = c2 * c1;
    } else if (sequence.compare("231") == 0) {
        rotation(0, 0) = c2 * c1;
        rotation(0, 1) = s2;
        rotation(0, 2) = -c2 * s1;

        rotation(1, 0) = -c3 * s2 * c1 + s3 * s1;
        rotation(1, 1) = c3 * c2;
        rotation(1, 2) = c3 * s2 * s1 + s3 * c1;

        rotation(2, 0) = s3 * s2 * c1 + c3 * s1;
        rotation(2, 1) = -s3 * c2;
        rotation(2, 2) = -s3 * s2 * s1 + c3 * c1;
    } else if (sequence.compare("232") == 0) {
        rotation(0, 0) = c3 * c2 * c1 - s3 * s1;
        rotation(0, 1) = c3 * s2;
        rotation(0, 2) = -(c3 * c2 * s1 + s3 * c1);

        rotation(1, 0) = -s2 * c1;
        rotation(1, 1) = c2;
        rotation(1, 2) = s2 * s1;

        rotation(2, 0) = s3 * c2 * c1 + c3 * s1;
        rotation(2, 1) = s3 * s2;
        rotation(2, 2) = -s3 * c2 * s1 + c3 * c1;
    } else if (sequence.compare("312") == 0) {
        rotation(0, 0) = -s3 * s2 * s1 + c3 * c1;
        rotation(0, 1) = s3 * s2 * c1 + c3 * s1;
        rotation(0, 2) = -s3 * c2;

        rotation(1, 0) = -c2 * s1;
        rotation(1, 1) = c2 * c1;
        rotation(1, 2) = s2;

        rotation(2, 0) = c3 * s2 * s1 + s3 * c1;
        rotation(2, 1) = -c3 * s2 * c1 + s3 * s1;
        rotation(2, 2) = c3 * c2;
    } else if (sequence.compare("313") == 0) {
        rotation(0, 0) = -s3 * c2 * s1 + c3 * c1;
        rotation(0, 1) = s3 * c2 * c1 + c3 * s1;
        rotation(0, 2) = s3 * s2;

        rotation(1, 0) = -(c3 * c2 * s1 + s3 * c1);
        rotation(1, 1) = c3 * c2 * c1 - s3 * s1;
        rotation(1, 2) = c3 * s2;

        rotation(2, 0) = s2 * s1;
        rotation(2, 1) = -s2 * c1;
        rotation(2, 2) = c2;
    } else if (sequence.compare("321") == 0) {
        rotation(0, 0) = c2 * c1;
        rotation(0, 1) = c2 * s1;
        rotation(0, 2) = -s2;

        rotation(1, 0) = s3 * s2 * c1 - c3 * s1;
        rotation(1, 1) = s3 * s2 * s1 + c3 * c1;
        rotation(1, 2) = s3 * c2;

        rotation(2, 0) = c3 * s2 * c1 + s3 * s1;
        rotation(2, 1) = c3 * s2 * s1 - s3 * c1;
        rotation(2, 2) = c3 * c2;
    } else if (sequence.compare("323") == 0) {
        rotation(0, 0) = c3 * c2 * c1 - s3 * s1;
        rotation(0, 1) = c3 * c2 * s1 + s3 * c1;
        rotation(0, 2) = -c3 * s2;

        rotation(1, 0) = -(s3 * c2 * c1 + c3 * s1);
        rotation(1, 1) = -s3 * c2 * s1 + c3 * c1;
        rotation(1, 2) = s3 * s2;

        rotation(2, 0) = s2 * c1;
        rotation(2, 1) = s2 * s1;
        rotation(2, 2) = c2;
    } else {
        // TODO::Need to add a default case in case the wrong sequence is input
    }
    
    return rotation;
}

/**
 * Converts euler angles into a quaternion. Referenced from S. W. Sheppard, "Quaternion from Rotation Matrix," Journal of
 * Guidance and Control, Vol. 1, No. 3, pp. 223 - 224, 1978
 * Input: sequence - String which determines the rotation sequence
 * Input: theta - The Euler angles
 * output: Attitude quaternion
**/
template<typename Scalar>
inline Quaternion<Scalar> eulerToQuaternion(const std::string& sequence, const EulerAngle<Scalar>& theta)
{
    const RotationMatrix<Scalar> rotation = eulerRotationMatrix(sequence, theta);

    Quaternion<Scalar> qTemp = Quaternion<Scalar>::Zero();

    if (rotation(1, 1) > -rotation(2, 2) && rotation(0, 0) > -rotation(1, 1) && rotation(0, 0) > -rotation(2, 2)) {
        const Scalar value = static_cast<Scalar>(std::sqrt(1.0 + rotation(0, 0) + rotation(1, 1) + rotation(2, 2)));

        qTemp(0) = static_cast<Scalar>(0.5) * value;
        qTemp(1) = static_cast<Scalar>(0.5) * (rotation(1, 2) - rotation(2, 1)) / value;
        qTemp(2) = static_cast<Scalar>(0.5) * (rotation(2, 0) - rotation(0, 2)) / value;
        qTemp(3) = static_cast<Scalar>(0.5) * (rotation(0, 1) - rotation(1, 0)) / value;
    } else if (rotation(1, 1) < -rotation(2, 2) && rotation(0, 0) > rotation(1, 1) && rotation(0, 0) > rotation(2, 2)) {
        const Scalar value = static_cast<Scalar>(std::sqrt(1.0 + rotation(0, 0) - rotation(1, 1) - rotation(2, 2)));

        qTemp(0) = static_cast<Scalar>(0.5) * (rotation(1, 2) - rotation(2, 1)) / value;
        qTemp(1) = static_cast<Scalar>(0.5) * value;
        qTemp(2) = static_cast<Scalar>(0.5) * (rotation(0, 1) + rotation(1, 0)) / value;
        qTemp(3) = static_cast<Scalar>(0.5) * (rotation(2, 0) + rotation(0, 2)) / value;
    } else if (rotation(1, 1) > rotation(2, 2) && rotation(0, 0) < rotation(1, 1) && rotation(0, 0) < -rotation(2, 2)) {
        const Scalar value = static_cast<Scalar>(std::sqrt(1.0 - rotation(0, 0) + rotation(1, 1) - rotation(2, 2)));

        qTemp(0) = static_cast<Scalar>(0.5) * (rotation(2, 0) - rotation(0, 2)) / value;
        qTemp(1) = static_cast<Scalar>(0.5) * (rotation(0, 1) + rotation(1, 0)) / value;
        qTemp(2) = static_cast<Scalar>(0.5) * value;
        qTemp(3) = static_cast<Scalar>(0.5) * (rotation(1, 2) + rotation(2, 1)) / value;
    } else {
        const Scalar value = static_cast<Scalar>(std::sqrt(1.0 - rotation(0, 0) - rotation(1, 1) + rotation(2, 2)));

        qTemp(0) = static_cast<Scalar>(0.5) * (rotation(0, 1) - rotation(1, 0)) / value;
        qTemp(1) = static_cast<Scalar>(0.5) * (rotation(2, 0) + rotation(0, 2)) / value;
        qTemp(2) = static_cast<Scalar>(0.5) * (rotation(1, 2) + rotation(2, 1)) / value;
        qTemp(3) = static_cast<Scalar>(0.5) * value;
    }

    return Quaternion<Scalar>{qTemp(1), qTemp(2), qTemp(3), qTemp(0)};
}

/**
 * Converts the quaternion into a rotation matrix. Referenced from Analytics of Space Systems by Hanspeter Schaub and
 * John L. Junkins and various other sources
 * Input: quat - The attitude quaternion 
 * output: Rotation matrix
**/
template<typename Scalar>
inline RotationMatrix<Scalar> quaternionRotationMatrix(const Quaternion<Scalar>& quat)
{
     // Quaternion vector skew symmetric matrix 
    const Eigen::Matrix<Scalar, 3, 3> qCross{{0.0, -quat(2), quat(1)}, {quat(2), 0.0, -quat(0)}, {-quat(1), quat(0), 0.0}};

    const Eigen::Vector<Scalar, 3> qVector{quat(0), quat(1), quat(2)};                    /// Quaternion vector component
    const Eigen::Matrix<Scalar, 3, 3> identity = Eigen::Matrix<Scalar, 3, 3>::Identity(); /// Identity matrix

    return (quat(3) * quat(3) - qVector.squaredNorm()) * identity - static_cast<Scalar>(2.0) * quat(3) * qCross
        + static_cast<Scalar>(2.0) * qVector * qVector.transpose();
}

/**
 * Converts attitude quaternions into the desired euler angle. Referenced from Analytics of Space Systems by Hanspeter
 * Schaub andvJohn L. Junkins, Pages 759 - 760
 * Input: sequence - String which determines the rotation sequence
 * Input: quat - The attitude quaternion
 * output: Euler angle in desired sequence
**/
template<typename Scalar>
inline EulerAngle<Scalar> quaternionToEuler(const std::string& sequence, const Quaternion<Scalar>& quat)
{
    const RotationMatrix<Scalar> rotation = quaternionRotationMatrix(quat);

    EulerAngle<Scalar> euler = EulerAngle<Scalar>::Zero();

    if (sequence.compare("121") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(rotation(0, 1), -rotation(0, 2)));
        euler(1) = static_cast<Scalar>(std::acos(rotation(0, 0)));
        euler(2) = static_cast<Scalar>(std::atan2(rotation(1, 0), rotation(2, 0)));
    } else if (sequence.compare("123") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(-rotation(2, 1), rotation(2, 2)));
        euler(1) = static_cast<Scalar>(std::asin(rotation(2, 0)));
        euler(2) = static_cast<Scalar>(std::atan2(-rotation(1, 0), rotation(0, 0)));
    } else if (sequence.compare("131") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(rotation(0, 2), rotation(0, 1)));
        euler(1) = static_cast<Scalar>(std::acos(rotation(0, 0)));
        euler(2) = static_cast<Scalar>(std::atan2(rotation(2, 0), -rotation(1, 0)));
    } else if (sequence.compare("132") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(rotation(1, 2), rotation(1, 1)));
        euler(1) = static_cast<Scalar>(std::asin(-rotation(1, 0)));
        euler(2) = static_cast<Scalar>(std::atan2(rotation(2, 0), rotation(0, 0)));
    } else if (sequence.compare("212") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(rotation(1, 0), rotation(1, 2)));
        euler(1) = static_cast<Scalar>(std::acos(rotation(1, 1)));
        euler(2) = static_cast<Scalar>(std::atan2(rotation(0, 1), -rotation(2, 1)));
    } else if (sequence.compare("213") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(rotation(2, 0), rotation(2, 2)));
        euler(1) = static_cast<Scalar>(std::asin(-rotation(2, 1)));
        euler(2) = static_cast<Scalar>(std::atan2(rotation(0, 1), rotation(1, 1)));
    } else if (sequence.compare("231") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(-rotation(0, 2), rotation(0, 0)));
        euler(1) = static_cast<Scalar>(std::asin(rotation(0, 1)));
        euler(2) = static_cast<Scalar>(std::atan2(-rotation(2, 1), rotation(1, 1)));
    } else if (sequence.compare("232") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(rotation(1, 2), -rotation(1, 0)));
        euler(1) = static_cast<Scalar>(std::acos(rotation(1, 1)));
        euler(2) = static_cast<Scalar>(std::atan2(rotation(2, 1), rotation(0, 1)));
    } else if (sequence.compare("312") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(-rotation(1, 0), rotation(1, 1)));
        euler(1) = static_cast<Scalar>(std::asin(rotation(1, 2)));
        euler(2) = static_cast<Scalar>(std::atan2(-rotation(0, 2), rotation(2, 2)));
    } else if (sequence.compare("313") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(rotation(2, 0), -rotation(2, 1)));
        euler(1) = static_cast<Scalar>(std::acos(rotation(2, 2)));
        euler(2) = static_cast<Scalar>(std::atan2(rotation(0, 2), rotation(1, 2)));
    } else if (sequence.compare("321") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(rotation(0, 1), rotation(0, 0)));
        euler(1) = static_cast<Scalar>(std::asin(-rotation(0, 2)));
        euler(2) = static_cast<Scalar>(std::atan2(rotation(1, 2), rotation(2, 2)));
    } else if (sequence.compare("323") == 0) {
        euler(0) = static_cast<Scalar>(std::atan2(rotation(2, 1), rotation(2, 0)));
        euler(1) = static_cast<Scalar>(std::acos(rotation(2, 2)));
        euler(2) = static_cast<Scalar>(std::atan2(rotation(1, 2), -rotation(0, 2)));
    } else {
        // TODO::Need to add a default case in case the wrong sequence is input
    }

    return euler;
}

/**
 * Performs the attitude quaternion inverse.
 * Input: quat - The attitude quaternion
 * output: The attitude quaternion inverse
**/
template<typename Scalar>
inline Quaternion<Scalar> quatInverse(const Quaternion<Scalar>& quat)
{
    return Quaternion<Scalar>{-quat(0), -quat(1), -quat(2), quat(3)};
}

/**
 * Performs the attitude quaternion multiplication operator.
 * Input: quat1 - The first attitude quaternion
 * Input: quat2 - The second attitude quaternion
 * output: The product of two attitude quaternions being multiplied
**/
template <typename Scalar>
inline Quaternion<Scalar> quatMultiply(const Quaternion<Scalar>& quat1, const Quaternion<Scalar>& quat2)
{
    const Eigen::Vector<Scalar, 3> quat1Vector{quat1(0), quat1(1), quat1(2)};
    const Eigen::Vector<Scalar, 3> quat2Vector{quat2(0), quat2(1), quat2(2)};
    const Eigen::Vector<Scalar, 3> crossProduct = quat1Vector.cross(quat2Vector);
    const Scalar dotProduct = quat1Vector.dot(quat2Vector);

    Quaternion<Scalar> quat = Quaternion<Scalar>::Zero();
    quat(0) = quat1(3) * quat2(0) + quat2(3) * quat1(0) - crossProduct(0);
    quat(1) = quat1(3) * quat2(1) + quat2(3) * quat1(1) - crossProduct(1);
    quat(2) = quat1(3) * quat2(2) + quat2(3) * quat1(2) - crossProduct(2);
    quat(3) = quat1(3) * quat2(3) - dotProduct;

    return quat;
}

/**
 * Calculates the error quaternion of quat 1 with respect to quat 2.
 * Input: quat1 - The first attitude quaternion
 * Input: quat2 - The second attitude quaternion
 * output: The error quaternion
**/
template <typename Scalar>
inline Quaternion<Scalar> errorQuaternion(const Quaternion<Scalar>& quat1, const Quaternion<Scalar>& quat2)
{
    const Quaternion<Scalar> quat2Inverse = quatInverse(quat2);

    return quatMultiply(quat1, quat2Inverse);
}

/**
 * Calculates the attitude quaternion rate.
 * Input: quat - The attitude quaternion
 * Input: omega - The vehicle/body angular rate
 * output: The attitude quaternion rate
**/
template <typename Scalar>
inline Quaternion<Scalar> quaternionKinematics(const Quaternion<Scalar>& quat, const BodyRate<Scalar>& omega)
{
    // E = [q(4)*eye(3,3) + qCross; -q(1:3)']
    const Eigen::Matrix<Scalar, 4, 3> E{{quat(3), -quat(2), quat(1)},
                                        {quat(2), quat(3), -quat(0)},
                                        {-quat(1), quat(0), quat(3)},
                                        {-quat(0), -quat(1), -quat(2)}};

    return static_cast<Scalar>(0.5) * E * omega;
}

} // namespace attitude

#endif // ATTITUDE_UTILS_H_
