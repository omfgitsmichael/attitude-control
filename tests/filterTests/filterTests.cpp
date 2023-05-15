#include <gtest/gtest.h>

#include "filters/AHRSKalmanFilter.h"
#include "filters/multiplicativeExtendedKalmanFilter.h"
#include "quaternions/quaternionMath.h"

constexpr double deg2rad = M_PI / 180.0;

GTEST_TEST(filterTests, MultiplicativeExtendedKalmanFilter)
{
    // Initialize the params
    attitude::filter::mekf::MEKFParams<double> filterParams;
    filterParams.dt = 0.01;
    filterParams.biasProcessNoise = 0.01; 
    filterParams.omegaProcessNoise = 0.01;

    // Initialize the data
    attitude::filter::mekf::MEKFData<double> filterData;
    filterData.quat = attitude::Quaternion<double>{0.0, 0.0, 0.0, 1.0};
    filterData.P = 1e9 * attitude::filter::Covariance<double, 6>::Identity();
    const attitude::filter::AttitudeVector<double> inertialRef1{1, 0, 0};
    const double intertialRef1Sigma = 0.025;
    const attitude::filter::AttitudeVector<double> inertialRef2{0, 0, 1};
    const double intertialRef2Sigma = 0.025;

    // Generate measurements
    const double yawDegrees = 30.0;
    const double pitchDegrees = 15.0;
    const double rollDegrees = -60.0;
    const attitude::EulerAngle<double> euler{yawDegrees * deg2rad, pitchDegrees * deg2rad, rollDegrees * deg2rad};
    attitude::OptionalRotationMatrix<double> rotation = attitude::eulerRotationMatrix("321", euler);

    ASSERT_TRUE(rotation);
    attitude::filter::AttitudeMeasurement<double> meas1;
    meas1.attitudeMeasVector = (*rotation) * inertialRef1;
    meas1.attitudeRefVector = inertialRef1;
    meas1.sigma = intertialRef1Sigma;
    meas1.valid = true;
    filterData.attitudeMeasurements.push_back(meas1);

    attitude::filter::AttitudeMeasurement<double> meas2;
    meas2.attitudeMeasVector = (*rotation) * inertialRef2;
    meas2.attitudeRefVector = inertialRef2;
    meas2.sigma = intertialRef2Sigma;
    meas2.valid = true;
    filterData.attitudeMeasurements.push_back(meas2);

    filterData.omegaMeas = attitude::BodyRate<double>{0.1, -0.1, 0.1};

    // Run the filter
    const bool result = attitude::filter::mekf::multiplicativeExtendedKalmanFilter(filterParams, filterData);

    // Compare results to matlab
    EXPECT_TRUE(result);
    EXPECT_NEAR(filterData.quat(0), -0.218338058, 1e-8);
    EXPECT_NEAR(filterData.quat(1), 0.066575877, 1e-8);
    EXPECT_NEAR(filterData.quat(2), 0.820168143, 1e-8);
    EXPECT_NEAR(filterData.quat(3), 0.524614488, 1e-8);
    EXPECT_NEAR(filterData.omega(0), 0.091680646, 1e-8);
    EXPECT_NEAR(filterData.omega(1), -0.121299575, 1e-8);
    EXPECT_NEAR(filterData.omega(2), 0.115997172, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(0), 0.008319353, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(1), 0.021299575, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(2), -0.015997172, 1e-8);
}

GTEST_TEST(filterTests, AHRSKalmanFilter)
{
    // Initialize the params
    attitude::filter::ahrs::AHRSParams<double> filterParams;
    filterParams.gravity = 9.81;
    filterParams.geomagneticFieldStrength = 50.0;
    filterParams.dt = 0.01;
    filterParams.biasProcessNoise = 0.01; 
    filterParams.omegaProcessNoise = 0.01;
    filterParams.linearAccelNoise = 0.01;
    filterParams.magDisturbanceNoise = 0.5;

    // Initialize the data
    attitude::filter::ahrs::AHRSData<double> filterData;
    filterData.quat = attitude::Quaternion<double>{0.0, 0.0, 0.0, 1.0};
    filterData.P = 1e3 * attitude::filter::Covariance<double, 12>::Identity();
    const attitude::filter::AttitudeVector<double> inertialRef1{0, 0, 1};
    const double intertialRef1Sigma = 0.025;

    const double inclinationAngle = 20.0 * deg2rad;
    const attitude::filter::AttitudeVector<double> inertialRef2{std::cos(inclinationAngle), 0.0, std::sin(inclinationAngle)};
    const double intertialRef2Sigma = 0.1;

    // Generate measurements
    const double yawDegrees = 30.0;
    const double pitchDegrees = 15.0;
    const double rollDegrees = -60.0;
    const attitude::EulerAngle<double> euler{yawDegrees * deg2rad, pitchDegrees * deg2rad, rollDegrees * deg2rad};
    attitude::OptionalRotationMatrix<double> rotation = attitude::eulerRotationMatrix("321", euler);

    ASSERT_TRUE(rotation);
    attitude::filter::AttitudeMeasurement<double> meas1;
    meas1.attitudeMeasVector =  (*rotation) * (filterParams.gravity * inertialRef1);;
    meas1.attitudeRefVector = inertialRef1;
    meas1.sigma = intertialRef1Sigma;
    meas1.valid = true;
    filterData.accelerometerMeas = meas1;

    attitude::filter::AttitudeMeasurement<double> meas2;
    meas2.attitudeMeasVector = (*rotation) * (filterParams.geomagneticFieldStrength * inertialRef2);
    meas2.attitudeRefVector = inertialRef2;
    meas2.sigma = intertialRef2Sigma;
    meas2.valid = true;
    filterData.magnetometerMeas = meas2;

    const attitude::BodyRate<double> omega{0.1, -0.1, 0.1};
    filterData.omegaMeas = omega;

    // Run the filter
    const bool result = attitude::filter::ahrs::AHRSKalmanFilter(filterParams, filterData);

    // Compare results to matlab - Initialization
    EXPECT_TRUE(result);
    EXPECT_NEAR(filterData.quat(0), -0.508086040, 1e-8);
    EXPECT_NEAR(filterData.quat(1), -0.019115728, 1e-8);
    EXPECT_NEAR(filterData.quat(2), 0.285266054, 1e-8);
    EXPECT_NEAR(filterData.quat(3), 0.812469348, 1e-8);
    EXPECT_NEAR(filterData.omega(0), 0.1, 1e-8);
    EXPECT_NEAR(filterData.omega(1), -0.1, 1e-8);
    EXPECT_NEAR(filterData.omega(2), 0.1, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(0), 0.0, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(1), 0.0, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(2), 0.0, 1e-8);

    // Propagate attitude
    const attitude::OptionalQuaternion<double> quat = attitude::eulerToQuaternion("321", euler);
    ASSERT_TRUE(quat);

    const attitude::Quaternion<double> quatDot = attitude::quaternionKinematics(*quat, omega);
    attitude::Quaternion<double> quat2 = *quat + quatDot * filterParams.dt;
    quat2 /= quat2.norm();

    // Generate the measurements
    rotation = attitude::quaternionRotationMatrix(quat2);

    meas1.attitudeMeasVector =  (*rotation) * (filterParams.gravity * inertialRef1);;
    meas1.attitudeRefVector = inertialRef1;
    meas1.sigma = intertialRef1Sigma;
    meas1.valid = true;
    filterData.accelerometerMeas = meas1;

    meas2.attitudeMeasVector = (*rotation) * (filterParams.geomagneticFieldStrength * inertialRef2);
    meas2.attitudeRefVector = inertialRef2;
    meas2.sigma = intertialRef2Sigma;
    meas2.valid = true;
    filterData.magnetometerMeas = meas2;

    // Run the filter
    const bool result2 = attitude::filter::ahrs::AHRSKalmanFilter(filterParams, filterData);

    // Compare results to matlab - Initialization
    EXPECT_TRUE(result2);
    EXPECT_NEAR(filterData.quat(0), -0.507546713, 1e-8);
    EXPECT_NEAR(filterData.quat(1), -0.019125289, 1e-8);
    EXPECT_NEAR(filterData.quat(2), 0.285935974, 1e-8);
    EXPECT_NEAR(filterData.quat(3), 0.812570720, 1e-8);
    EXPECT_NEAR(filterData.omega(0), 0.1, 1e-8);
    EXPECT_NEAR(filterData.omega(1), -0.1, 1e-8);
    EXPECT_NEAR(filterData.omega(2), 0.1, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(0), 0.0, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(1), 0.0, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(2), 0.0, 1e-8);
}
