#ifndef SIM_H_
#define SIM_H_

#include <random>

#include "quaternions/quaternionMath.h"
#include "utils/attitudeUtils.h"

namespace attitude {

namespace sim{

template<typename Scalar>
BodyRate<Scalar> simModel(const Eigen::Matrix<Scalar, 3, 3>& intertialMatrix,
                          const BodyRate<Scalar>& omega,
                          const Control<Scalar>& externalForces,
                          const Control<Scalar>& control)
{
    const BodyRate<Scalar> Iw = intertialMatrix * omega;

    return intertialMatrix.inverse() * (omega.cross(Iw) + externalForces + control);
}

template <typename Scalar>
class Sim {
  public: 
    Sim(Scalar dt,
        const std::string& sequence,
        const EulerAngle<Scalar>& eulerInit,
        const BodyRate<Scalar>& omegaInit,
        const BodyRate<Scalar>& omegaDotInit,
        const Eigen::Matrix<Scalar, 3, 3>& inertialMatrix) :
    dt_(dt),
    sequence_(sequence),
    omega_(omegaInit),
    omegaDot_(omegaDotInit),
    intertialMatrix_(inertialMatrix)
    {
        OptionalQuaternion<Scalar> quat = eulerToQuaternion<Scalar>(sequence_, eulerInit);
        if (quat) {
            quaternion_ = *quat;
        }
    }

    void updateStates(const Control<Scalar>& externalForces, const Control<Scalar>& control)
    {
        // Update the attitude quaternion
        Quaternion<Scalar> quatRate = quaternionKinematics(quaternion_, omega_);
        quaternion_ += quatRate * dt_;
        quaternion_ /=  quaternion_.norm(); // Brute force normalization

        // Update the body rates
        omegaDot_ = simModel(intertialMatrix_, omega_, externalForces, control);
        omega_ += omegaDot_ * dt_;
    }

    void updateInertialMatrix(const Eigen::Matrix<Scalar, 3, 3>& intertialMatrix)
    {
        intertialMatrix_ = intertialMatrix;
    }

    Quaternion<Scalar> getAttitudeQuat() const
    {
        return quaternion_;
    }

    EulerAngle<Scalar> getAttitude() const
    {
        OptionalEulerAngle<Scalar> euler = quaternionToEuler<Scalar>(sequence_, quaternion_);

        if (euler) {
            return (*euler);
        }

        return EulerAngle<Scalar>::Zero();
    }

    BodyRate<Scalar> getOmega() const
    {
        return omega_;
    }

    BodyRate<Scalar> getOmegaDot() const
    {
        return omegaDot_;
    }

  private:
    Scalar dt_ = 0.0;
    std::string sequence_;
    Quaternion<Scalar> quaternion_ = Quaternion<Scalar>::Zero();
    BodyRate<Scalar> omega_ = BodyRate<Scalar>::Zero();
    BodyRate<Scalar> omegaDot_ = BodyRate<Scalar>::Zero();
    Eigen::Matrix<Scalar, 3, 3> intertialMatrix_ = Eigen::Matrix<Scalar, 3, 3>::Zero();

    std::default_random_engine generator_;
    Scalar attitudeNoise_;
    Scalar omegaNoise_;
    Scalar omegaDotNoise_;
};

} // namespace sim

} // namespace attitude

#endif // SIM_H_