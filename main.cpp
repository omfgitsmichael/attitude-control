#include <iostream>

#include <utils/attitudeUtils.h>
#include <controllers/passivityBasedAdativeControl.h>

int main() {
    attitude::EulerAngle<double> theta = {0.0125, 0.125, 0.0125}; 
    std::string sequence = "321";

    std::cout << theta << "\n" << std::endl;

    attitude::RotationMatrix<double> rotation_matrix_theta = attitude::eulerRotationMatrix(sequence, theta);

    std::cout << rotation_matrix_theta << "\n" << std::endl;

    attitude::Quaternion<double> quaternion = attitude::eulerToQuaternion(sequence, theta);

    std::cout << quaternion << "\n" << std::endl;

    attitude::RotationMatrix<double> rotation_matrix_quat = attitude::eulerRotationMatrix(sequence, theta);

    std::cout << rotation_matrix_quat << "\n" << std::endl;

    attitude::EulerAngle<double> theta_compare = attitude::quaternionToEuler(sequence, quaternion);

    std::cout << theta_compare << "\n" << std::endl; /// Seems like there is approximately 3% accuracy loss in the conversion

    attitude::Quaternion<double> quaternion2{std::sqrt(4.0) / 4.0, std::sqrt(4.0) / 4.0, std::sqrt(4.0) / 4.0, std::sqrt(4.0) / 4.0};

    attitude::Quaternion<double> quatProduct = attitude::quatMultiply(quaternion, quaternion2);

    std::cout << quatProduct << "\n" << std::endl;

    attitude::Quaternion<double> quatError = attitude::errorQuaternion(quaternion, quaternion2);

    std::cout << quatError << "\n" << std::endl;

    attitude::BodyRate<double> omegaError = {0.025, -0.1, 0.05};
    attitude::Quaternion<double> quatErrorRate = attitude::quaternionKinematics(quatError, omegaError);

    std::cout << quatErrorRate << "\n" << std::endl;

    attitude::PassivityBasedAdaptiveControl<double>::Params params;
    params.dt = 0.1;
    params.lambda = Eigen::Matrix<double, 3, 3>{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    params.k = Eigen::Matrix<double, 3, 3>{{2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 2.0}};
    params.gammaInv = Eigen::Matrix<double, 3, 3>{{0.1, 0.0, 0.0}, {0.0, 0.1, 0.0}, {0.0, 0.0, 0.1}};
    params.projectionParams.epsilon(0) = 0.5;
    params.projectionParams.thetaMax(0) = 3000.0;
    params.deadzoneParams.del = 0.5;
    params.deadzoneParams.e0 = 0.5;

    attitude::PassivityBasedAdaptiveControl<double> passivityBasedAdaptiveController(params);

    attitude::ProjectionParams<double, 1> projectionParams;
    projectionParams.epsilon(0) = 0.5;
    projectionParams.thetaMax(0) = 3000.0;

    Eigen::Vector<double, 3> paramEstimate = {0.0082, 0.0, -0.0009};
    Eigen::Vector<double, 3> law = {0.0909, -0.0088, 0.2462};
    Eigen::Matrix<double, 3, 3> gamma = Eigen::Matrix<double, 3, 3>::Identity();
    gamma(0, 0) = 1.0 / 0.1;
    gamma(1, 1) = 1.0 / 100000.0;
    gamma(2, 2) = 1.0 / 1.0;

    Eigen::Vector<double, 3> adaptationLaw = gamma * law;

    Eigen::Vector<double, 3> thetaDot = attitude::projection(projectionParams, paramEstimate, adaptationLaw);

    std::cout << thetaDot << "\n" << std::endl;

    return 0;
}
