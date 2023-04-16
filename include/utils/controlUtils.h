#ifndef CONTROL_UTILS_H_
#define CONTROL_UTILS_H_

#include <algorithm>

#include "types/typenames.h"

namespace attitude {

/**
 * Deadzone operator parameters
**/
template <typename Scalar>
struct DeadzoneParams
{
    Scalar del = 0.0;
    Scalar e0 = 0.0;
};

/**
 * Projection operator parameters
**/
template <typename Scalar, int Size>
struct ProjectionParams
{
    Eigen::Vector<Scalar, Size> epsilon = Eigen::Vector<Scalar, Size>::Zero();
    Eigen::Vector<Scalar, Size> thetaMax = Eigen::Vector<Scalar, Size>::Zero();
};

/**
 * The continuous deadzone operator that ensures bounded parameters when subject to measurement noise. Stops the adaptation laws
 * when the norm of the tracking error becomes smaller than the prescribed e0. 0 < params.del < 1
 * Referenced from "Robust and Adaptive Control with Aerospace Applications" by Eugene Lavretsky and Kevin A. Wise, chapter 11
 * Input: params - Deadzone params
 * Input: error - Tracking error vector
 * Input: deaadzone - The deadzone operator
 * output: Boolean if it passed or failed
**/
template <typename Scalar, int Size>
inline bool deadzoneOperator(const DeadzoneParams<Scalar>& params, const Eigen::Vector<Scalar, Size>& error, Scalar& deadzone)
{
    if (params.del <= 0.0 || params.del >= 1.0 || params.e0 <= 0.0) {
        return false;
    }

    Scalar e = error.norm();

    deadzone = (e - params.del * params.e0) / ((1.0 - params.del) * params.e0);
    deadzone = std::max(0.0, std::min(1.0, deadzone));

    return true;
}

/**
 * The generic projection operator that works for NxM parameter estimate matrices and not just vectors. The projection operator 
 * acts as an anti-windup method for the parameter estimates and bounds them. epsilon > 0, thetaMax > 0
 * Referenced from "Robust and Adaptive Control with Aerospace Applications" by Eugene Lavretsky and Kevin A. Wise, chapter 11
 * Input: params - projection params
 * Input: theta - Parameter estimates
 * Input: adaptationLaw - Parameter estimate adaptation laws
 * Input: thetaDot - The parameter estimate rate
 * output: Boolean if it passed or failed
**/
template <typename Scalar, int Size1, int Size2>
inline bool projectionOperator(const ProjectionParams<Scalar, Size2>& params,
                               const Eigen::Matrix<Scalar, Size1, Size2>& theta,
                               const Eigen::Matrix<Scalar, Size1, Size2>& adaptationLaw,
                               Eigen::Matrix<Scalar, Size1, Size2>& thetaDot)
{
    unsigned int cols = theta.cols();

    // If any of the parameters fail return false
    for (unsigned int i = 0; i < cols; i++) {
        if (params.epsilon(i) <= 0.0 || params.thetaMax(i) <= 0.0) {
            return false;
        }
    }

    for (unsigned int i = 0; i < cols; i++) {
        const Scalar F = ((1.0 + params.epsilon(i)) * theta.col(i).squaredNorm() - params.thetaMax(i) * params.thetaMax(i)) 
            / (params.epsilon(i) * params.thetaMax(i) * params.thetaMax(i));
    
        const Eigen::Vector<Scalar, Size1> deltaF = 2.0 * ((1.0 + params.epsilon(i)) / (params.epsilon(i) * params.thetaMax(i))) * theta.col(i);

        if (F > 0.0 && adaptationLaw.col(i).transpose() * deltaF > 0.0) {
            thetaDot.col(i) = adaptationLaw.col(i) - (deltaF * deltaF.transpose() / deltaF.squaredNorm()) * adaptationLaw.col(i) * F;
        } else {
            thetaDot.col(i) = adaptationLaw.col(i);
        }
    }

    return true;
}

} // namespace attitude

#endif // CONTROL_UTILS_H_
