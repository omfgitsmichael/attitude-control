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
    * Calculate the passivity-based adaptive control. Referenced from "Passivity Based Adaptive Attitude Control of Rigid
    * Spacecraft" by O. Egeland, and J. -M. Godhavn, IEEE Transactions on Automatic Control, Vol 39, issue No. 4, April 1994
    * Input: quat - The attitude quaternion
    * Input: quatDesired - The desired attitude quaternion
    * Input: omega - The current angular body rates
    * Input: omegaDesired - The desired angular body rates
    * Input: omegaDotDesired - The desired angular body acceleration
    * output: Control torque
    **/
    Scalar calculateControl(const Quaternion<Scalar>& quat,
                            const Quaternion<Scalar>& quatDesired,
                            const BodyRate<Scalar>& omega,
                            const BodyRate<Scalar>& omegaDesired,
                            const BodyRate<Scalar>& omegaDotDesired)
    {
        return u_;
    }

  private:
    Params controllerParams_;
    Eigen::Vector<Scalar, 3> estimatedParams_ = Eigen::Vector<Scalar, 3>::Zero();
    Eigen::Vector<Scalar, 3> u_ = Eigen::Vector<Scalar, 3>::Zero();
};

} // namespace attitude

#endif // PASSIVITY_BASED_ADAPTIVE_CONTROL_H_
