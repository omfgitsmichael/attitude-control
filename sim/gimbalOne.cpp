#include <chrono>
#include <iostream>
#include <thread>

#include "sim.h"
#include <utils/attitudeUtils.h>
#include <controllers/passivityBasedAdaptiveControl.h>

using namespace std::chrono_literals; /// ns, us, ms, s, h, etc.

// Math constants
constexpr double deg2rad = M_PI / 180.0;
constexpr double rad2Deg = 180.0 / M_PI;

// Sim Parameters
constexpr double tInit = 0.0;
constexpr double simRate = 0.01;
constexpr double tFinal = 25.0;
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
    const double attitudeNoise = 0.1 * deg2rad;
    const double omegaNoise = 0.1 * deg2rad;
    const double omegaDotNoise = 0.1 * deg2rad;
    const Eigen::Matrix<double, 3, 3> inertialMatrix{{1.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 3.0}};

    attitude::sim::Sim sim(simRate, sequence, eulerInit, attitudeNoise, omegaInit, omegaNoise, omegaDotInit, omegaDotNoise, inertialMatrix);

    // Controller initial conditions
    attitude::control::PassivityParams<double> params;
    params.deadzoneParams.del = 0.5;
    params.deadzoneParams.e0 = 0.5;
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
    data.theta = attitude::control::Theta<double>{1.0, 1.0, 1.0};
    data.quatDesired = *quatDesired;
    data.omegaDesired = omegaDesired;
    data.omegaDotDesired = omegaDotDesired;

    // External forces vector
    attitude::Control<double> externalForces{0.0, 0.0, 0.0};

    for (int i = 0; i < simSteps; i++) { 
        if ( i % controllerSteps == 0) {
            // Receive measurements and do any necessary conversions
            attitude::EulerAngle<double> ypr = sim.getAttitude(false);
            attitude::BodyRate<double> omega = sim.getOmega(false);

            attitude::OptionalQuaternion<double> quat = attitude::eulerToQuaternion(sequence, ypr);
            if (quat){
                data.quat = *quat;
            }

            data.omega = omega;

            // Filter measurements (if available)

            // Run controller
            bool result = attitude::control::passivityBasedAdaptiveControl(params, data);
            if (!result) {
                // Do whatever you want if the controller failed
                std::cout << "???????????????????WE FAILED????????????????????" << std::endl;
            }
        }

        // Pass control torque into the sim model
        sim.updateStates(externalForces, data.u);

        time += simRate;

        std::cout << "Sim Time: " << time << std::endl;
        std::cout << "Sim Euler Angles: " << sim.getAttitude(false).transpose() << std::endl;
        std::cout << "Desired Euler Angles: " << eulerDes.transpose() << std::endl;
        std::cout << "Sim Body Rates: " << sim.getOmega(false).transpose() << std::endl;
        std::cout << "Desired Body Rates: " << omegaDesired.transpose() << std::endl;
        std::cout << "Control Torque: " << data.u.transpose() << std::endl;
        std::cout << "Estimated Parameters: " << data.theta.transpose() << std::endl;
        std::this_thread::sleep_for(10ms);
    }

    return 0;
}
