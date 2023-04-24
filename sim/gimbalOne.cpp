#include <chrono>
#include <iostream>
#include <thread>

#include "controllers/passivityBasedAdaptiveControl.h"
#include "sim.h"
#include "utils/attitudeUtils.h"

using namespace std::chrono_literals; /// ns, us, ms, s, h, etc.

// Math constants
constexpr double deg2rad = M_PI / 180.0;
constexpr double rad2Deg = 180.0 / M_PI;

// Sim Parameters
constexpr double tInit = 0.0;
constexpr double simRate = 0.01;
constexpr double tFinal = 50.0;
constexpr int simSteps = static_cast<int>((tFinal - tInit) / simRate);

// Controller params
constexpr double controllerRate = 0.1;
constexpr int controllerSteps = static_cast<int>((controllerRate) / simRate);

int main() {
    double time = tInit;
    std::string sequence = "321";

    // System initial conditions
    const double yawDegrees = 30.0;
    const double pitchDegrees = 15.0;
    const double rollDegrees = -60.0;
    const attitude::EulerAngle<double> eulerInit{yawDegrees * deg2rad, pitchDegrees * deg2rad, rollDegrees * deg2rad};
    const attitude::BodyRate<double> omegaInit{0.0, 0.0, 0.0};
    const attitude::BodyRate<double> omegaDotInit{0.0, 0.0, 0.0};
    Eigen::Matrix<double, 3, 3> inertialMatrix{{1.0, 0.5, 0.5}, {0.5, 2.0, 0.5}, {0.5, 0.5, 3.0}};

    // RNGs
    const bool useNoise = true;
    const double attitudeNoise = 0.1 * deg2rad;
    const double omegaNoise = 0.1 * deg2rad;
    const double omegaDotNoise = 0.1 * deg2rad;
    std::default_random_engine generator;
    std::normal_distribution<double> attitudeNoiseGenerator(0.0, attitudeNoise);
    std::normal_distribution<double> omegaNoiseGenerator(0.0, omegaNoise);
    std::normal_distribution<double> omegaDotNoiseGenerator(0.0, omegaDotNoise);

    attitude::sim::Sim sim(simRate, sequence, eulerInit, omegaInit, omegaDotInit, inertialMatrix);

    // Controller initial conditions
    attitude::control::PassivityParams<double> params;
    params.deadzoneParams.del = 0.5;
    params.deadzoneParams.e0 = 0.025;
    params.projectionParams.epsilon(0) = {0.5};
    params.projectionParams.thetaMax(0) = {3000.0};
    params.lambda = Eigen::Matrix<double, 3, 3>{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    params.k = Eigen::Matrix<double, 3, 3>{{2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 2.0}};
    params.gammaInv = Eigen::Matrix<double, 3, 3>{{0.1, 0.0, 0.0}, {0.0, 0.1, 0.0}, {0.0, 0.0, 0.1}}.inverse();
    params.dt = controllerRate;

    // Desired states -- constant values for this sim
    const double yawDesDegrees = 0.0;
    const double pitchDesDegrees = 0.0;
    const double rollDesDegrees = 0.0;
    const attitude::EulerAngle<double> eulerDes{yawDesDegrees * deg2rad, pitchDesDegrees * deg2rad, rollDesDegrees * deg2rad};
    const attitude::OptionalQuaternion<double> quatDesired = attitude::eulerToQuaternion(sequence, eulerDes);
    const attitude::BodyRate<double> omegaDesired{0.0, 0.0, 0.0};
    const attitude::BodyRate<double> omegaDotDesired{0.0, 0.0, 0.0};

    // Initialize the data structure for the controller class
    attitude::PassivityControlData<double> data;
    data.theta = attitude::control::Theta<double>{0.0, 0.0, 0.0};
    data.quatDesired = *quatDesired;
    data.omegaDesired = omegaDesired;
    data.omegaDotDesired = omegaDotDesired;

    // External torques vector
    attitude::Control<double> externalTorques{0.0, 0.0, 0.0};

    for (int i = 0; i < simSteps; i++) { 
        if ( i % controllerSteps == 0) {
            // Receive measurements and do any necessary conversions
            attitude::EulerAngle<double> ypr = sim.getAttitude();
            attitude::BodyRate<double> omega = sim.getOmega();

            // Want to move this into the sim class, but not sure how
            if (useNoise) {
                ypr += attitude::EulerAngle<double>{attitudeNoiseGenerator(generator), attitudeNoiseGenerator(generator), attitudeNoiseGenerator(generator)};
                omega += attitude::BodyRate<double>{omegaNoiseGenerator(generator), omegaNoiseGenerator(generator), omegaNoiseGenerator(generator)};
            }

            attitude::OptionalQuaternion<double> quat = attitude::eulerToQuaternion(sequence, ypr);
            if (quat){
                data.quat = *quat;
            }

            data.omega = omega;

            // Run controller
            bool result = attitude::control::passivityBasedAdaptiveControl(params, data);
            if (!result) {
                // Do whatever you want if the controller failed
            }
        }

        // Continously update the parameter matrix
        inertialMatrix += Eigen::Matrix<double, 3, 3>{{0.005, 0.0, 0.0}, {0.0, 0.005, 0.0}, {0.0, 0.0, 0.005}};
        sim.updateInertialMatrix(inertialMatrix);

        // Pass control torque into the sim model
        sim.updateStates(externalTorques, data.u);

        time += simRate;

        std::cout << "Sim Time: " << time << std::endl;
        std::cout << "Sim Euler Angles: " << sim.getAttitude().transpose() << std::endl;
        std::cout << "Desired Euler Angles: " << eulerDes.transpose() << std::endl;
        std::cout << "Sim Body Rates: " << sim.getOmega().transpose() << std::endl;
        std::cout << "Desired Body Rates: " << omegaDesired.transpose() << std::endl;
        std::cout << "Control Torque: " << data.u.transpose() << std::endl;
        std::cout << "Estimated Parameters: " << data.theta.transpose() << std::endl;
        std::cout << std::endl;
        std::this_thread::sleep_for(10ms);
    }

    return 0;
}
