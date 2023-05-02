#include <chrono>
#include <iostream>
#include <thread>

#include "controllers/passivityBasedAdaptiveControl.h"
#include "filters/AHRSKalmanFilter.h"
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
constexpr double gravity = 9.81;
constexpr double geomagneticFieldStrength = 50.0;

// Filter params
constexpr double filterRate = simRate;
constexpr int filterSteps = static_cast<int>((filterRate) / simRate);

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

    // Construct the sim class
    attitude::sim::Sim sim(simRate, sequence, eulerInit, omegaInit, omegaDotInit, inertialMatrix);

    // Filter initial conditions and params
    attitude::filter::ahrs::AHRSParams<double> filterParams;
    filterParams.gravity = 9.81; /// m/s/s
    filterParams.geomagneticFieldStrength = 50.0; /// micro tesla
    filterParams.dt = filterRate;
    filterParams.biasProcessNoise = 0.01; 
    filterParams.omegaProcessNoise = 0.01;
    filterParams.linearAccelNoise = 0.01;
    filterParams.magDisturbanceNoise = 0.5;

    attitude::filter::ahrs::AHRSData<double> filterData; /// Leave everything initialized to zero at the moment
    filterData.quaternion = attitude::Quaternion<double>{0.0, 0.0, 0.0, 1.0};
    filterData.P = 1e3 * attitude::filter::Covariance<double, 12>::Identity(); /// Set covariance as if we have no idea what our inital estimate is
    const attitude::filter::AttitudeVector<double> inertialRef1{0, 0, 1};
    const double intertialRef1Sigma = 0.025;

    const double inclinationAngle = 20.0 * deg2rad; /// True inclination angle of where the gimbal is on the globe
    const attitude::filter::AttitudeVector<double> inertialRef2{std::cos(inclinationAngle), 0.0, std::sin(inclinationAngle)};
    const double intertialRef2Sigma = 0.1;

    // Controller initial conditions
    attitude::control::PassivityParams<double> controllerParams;
    controllerParams.deadzoneParams.del = 0.5;
    controllerParams.deadzoneParams.e0 = 0.025;
    controllerParams.projectionParams.epsilon(0) = {0.5};
    controllerParams.projectionParams.thetaMax(0) = {3000.0};
    controllerParams.lambda = Eigen::Matrix<double, 3, 3>{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    controllerParams.k = Eigen::Matrix<double, 3, 3>{{2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 2.0}};
    controllerParams.gammaInv = Eigen::Matrix<double, 3, 3>{{0.1, 0.0, 0.0}, {0.0, 0.1, 0.0}, {0.0, 0.0, 0.1}}.inverse();
    controllerParams.dt = controllerRate;

    // Desired states -- constant values for this sim
    const double yawDesDegrees = 0.0;
    const double pitchDesDegrees = 0.0;
    const double rollDesDegrees = 0.0;
    const attitude::EulerAngle<double> eulerDes{yawDesDegrees * deg2rad, pitchDesDegrees * deg2rad, rollDesDegrees * deg2rad};
    const attitude::OptionalQuaternion<double> quatDesired = attitude::eulerToQuaternion(sequence, eulerDes);
    const attitude::BodyRate<double> omegaDesired{0.0, 0.0, 0.0};
    const attitude::BodyRate<double> omegaDotDesired{0.0, 0.0, 0.0};

    // Initialize the data structure for the controller class
    attitude::PassivityControlData<double> controllerData;
    controllerData.theta = attitude::control::Theta<double>{0.0, 0.0, 0.0};
    controllerData.quatDesired = *quatDesired;
    controllerData.omegaDesired = omegaDesired;
    controllerData.omegaDotDesired = omegaDotDesired;

    // External vectors
    attitude::Control<double> externalTorques{0.0, 0.0, 0.0};
    attitude::filter::AttitudeVector<double> linearAcceleration{0.0, -0.1, 0.1};
    attitude::filter::AttitudeVector<double> magneticDisturbance{0.5, 1.0, 0.5};


    for (int i = 0; i < simSteps; i++) { 
        // Generate any measurements
        attitude::filter::AttitudeVector<double> accelerometerMeas = inertialRef1;
        attitude::filter::AttitudeVector<double> magnetometerMeas = inertialRef2;
        attitude::EulerAngle<double> euler = sim.getAttitude();
        attitude::OptionalRotationMatrix<double> rotation = attitude::eulerRotationMatrix(sequence, euler); /// Shouldn't fail
        if (rotation) {
            accelerometerMeas =  (*rotation) * (gravity * inertialRef1 + linearAcceleration);
            magnetometerMeas = (*rotation) * (geomagneticFieldStrength * inertialRef2 + magneticDisturbance);
        }

        attitude::BodyRate<double> omega = sim.getOmega();

        // Want to move this into the sim class, but not sure how
        if (useNoise) {
            
            euler += attitude::EulerAngle<double>{attitudeNoiseGenerator(generator), attitudeNoiseGenerator(generator), attitudeNoiseGenerator(generator)};
            rotation = attitude::eulerRotationMatrix(sequence, euler); /// Shouldn't fail
            if (rotation) {
                accelerometerMeas = (*rotation) * (gravity * inertialRef1 + linearAcceleration);
                magnetometerMeas = (*rotation) * (geomagneticFieldStrength * inertialRef2 + magneticDisturbance);
            }

            omega += attitude::BodyRate<double>{omegaNoiseGenerator(generator), omegaNoiseGenerator(generator), omegaNoiseGenerator(generator)};
        }

        // Kalman Filter wrapper
        if (i % filterSteps == 0) {
            attitude::filter::AttitudeMeasurement<double> meas1;
            meas1.attitudeMeasVector = accelerometerMeas;
            meas1.attitudeRefVector = inertialRef1;
            meas1.sigma = intertialRef1Sigma;
            meas1.valid = true;
            filterData.accelerometerMeas = meas1;

            attitude::filter::AttitudeMeasurement<double> meas2;
            meas2.attitudeMeasVector = magnetometerMeas;
            meas2.attitudeRefVector = inertialRef2;
            meas2.sigma = intertialRef2Sigma;
            meas2.valid = true;
            filterData.magnetometerMeas = meas2;

            filterData.omegaMeas = omega;

            bool result = attitude::filter::ahrs::AHRSKalmanFilter(filterParams, filterData);
            if (!result) {
                // Do whatever you want if the filter failed
            }
        }

        if ( i % controllerSteps == 0) {
            // Use current filtered attitude and angular rate data
            controllerData.quat = filterData.quaternion;
            controllerData.omega = filterData.omega;

            // Run controller
            bool result = attitude::control::passivityBasedAdaptiveControl(controllerParams, controllerData);
            if (!result) {
                // Do whatever you want if the controller failed
            }
        }

        // Continously update the parameter matrix
        inertialMatrix += Eigen::Matrix<double, 3, 3>{{0.005, 0.0, 0.0}, {0.0, 0.005, 0.0}, {0.0, 0.0, 0.005}};
        sim.updateInertialMatrix(inertialMatrix);

        // Pass control torque into the sim model
        sim.updateStates(externalTorques, controllerData.u);

        time += simRate;

        std::cout << "Sim Time: " << time << std::endl;
        std::cout << "Sim Quaternion: " << sim.getAttitudeQuat().transpose() << std::endl;
        std::cout << "Desired Quaternion: " << (*quatDesired).transpose() << std::endl;
        std::cout << "Estimated Quaternion: " << filterData.quaternion.transpose() << std::endl;
        std::cout << "Sim Body Rates: " << sim.getOmega().transpose() << std::endl;
        std::cout << "Desired Body Rates: " << omegaDesired.transpose() << std::endl;
        std::cout << "Estimated Body Rates: " << filterData.omega.transpose() << std::endl;
        std::cout << "Estimated Gyro Bias: " << filterData.omegaBias.transpose() << std::endl;
        std::cout << "Estimated Linear Forces: " << filterData.linearAccelForces.transpose() << std::endl;
        std::cout << "Estimated Magnetic Disturbance: " << filterData.magneticDisturbances.transpose() << std::endl;
        std::cout << "Control Torque: " << controllerData.u.transpose() << std::endl;
        std::cout << "Estimated Parameters: " << controllerData.theta.transpose() << std::endl;
        std::cout << std::endl;
        std::this_thread::sleep_for(10ms);
    }

    return 0;
}
