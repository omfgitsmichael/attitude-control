#ifndef PASSIVITY_BASED_ADAPTIVE_CONTROL_H_
#define PASSIVITY_BASED_ADAPTIVE_CONTROL_H_

#include "utils/typenames.h"
#include "utils/controlUtils.h"

namespace attitude {

template <typename Scalar>
class PassivityBasedAdaptiveControl
{
  public:
    /**
    * The passivity-based adaptive control paramters
    **/
    struct Params
    {
        Scalar dt = 0;
        Eigen::Matrix<Scalar, 3, 3> lambda = Eigen::Matrix<Scalar, 3, 3>::Zero();   /// Diaginal matrix
        Eigen::Matrix<Scalar, 3, 3> k = Eigen::Matrix<Scalar, 3, 3>::Zero();        /// Diaginal matrix
        Eigen::Matrix<Scalar, 3, 3> gammaInv = Eigen::Matrix<Scalar, 3, 3>::Zero(); /// Diaginal matrix
        DeadzoneParams<Scalar> deadzoneParams;                                      /// Deadzone parameters
        ProjectionParams<Scalar, 1> projectionParams;                               /// Projection parameters
    };

    /**
    * Constructor
    **/
    PassivityBasedAdaptiveControl(const Params& params) : controllerParams_(params)
    {
    }

    /**
    * Calculate the passivity-based adaptive control augmented with the deadzone and projection operators. Referenced from
    * "Passivity Based Adaptive Attitude Control of Rigid Spacecraft" by O. Egeland, and J. -M. Godhavn, IEEE Transactions
    * on Automatic Control, Vol 39, issue No. 4, April 1994
    * Input: quat - The attitude quaternion
    * Input: quatDesired - The desired attitude quaternion
    * Input: omega - The current angular body rates
    * Input: omegaDesired - The desired angular body rates
    * Input: omegaDotDesired - The desired angular body acceleration
    * output: Control torque
    **/
    Eigen::Vector<Scalar, 3> calculateControl(const Quaternion<Scalar>& quat,
                                              const Quaternion<Scalar>& quatDesired,
                                              const BodyRate<Scalar>& omega,
                                              const BodyRate<Scalar>& omegaDesired,
                                              const BodyRate<Scalar>& omegaDotDesired)
    {
      // Calculate the errors
      Quaternion<Scalar> quatError = quatMultiply(quat, quatInverse(quatDesired));
      BodyRate<Scalar> omegaError = omega - omegaDesired;
      Quaternion<Scalar> quatErrorRate = quaternionKinematics(quatError, omegaError);

      const Eigen::Vector<Scalar, 3> quatErrorVector{quatError(0), quatError(1), quatError(2)};
      const Eigen::Vector<Scalar, 3> quatErrorRateVector{quatErrorRate(0), quatErrorRate(1), quatErrorRate(2)};

      // Calculate passivity terms
      const Eigen::Vector<Scalar, 3> s = omegaError + controllerParams_.lambda * quatErrorVector;
      const Eigen::Vector<Scalar, 3> omegaR = omegaDesired - controllerParams_.lambda * quatErrorVector;
      const Eigen::Vector<Scalar, 3> omegaRRate = omegaDotDesired - controllerParams_.lambda * quatErrorRateVector;

      // Create the regressor matrix
      const Eigen::Matrix<Scalar, 3, 3> regressor{{omegaRRate(0), -omega(1) * omegaR(2), omega(2) * omegaR(1)},
                                                  {omega(0) * omegaR(2), omegaRRate(1), -omega(2) * omegaR(0)},
                                                  {-omega(0) * omegaR(1), omega(1) * omegaR(0), omegaRRate(2)}};

      // Update parameter estimates
      const Scalar deadzone = deadzoneOperator(controllerParams_.deadzoneParams, s);
      const Eigen::Vector<Scalar, 3> adapationLaws = -controllerParams_.gammaInv * regressor.transpose() * s * deadzone;
      estimatedParams_ += projectionOperator(controllerParams_.projectionParams, estimatedParams_, adapationLaws) * controllerParams_.dt;

      // Calculate the adaptive control
      u_ = regressor * estimatedParams_ - controllerParams_.k * s;
      
      return u_;
    }

  private:
    Params controllerParams_;
    Eigen::Vector<Scalar, 3> estimatedParams_ = Eigen::Vector<Scalar, 3>::Zero();
    Eigen::Vector<Scalar, 3> u_ = Eigen::Vector<Scalar, 3>::Zero();
};

} // namespace attitude

#endif // PASSIVITY_BASED_ADAPTIVE_CONTROL_H_