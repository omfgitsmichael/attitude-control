#include <gtest/gtest.h>

#include "filters/AHRSKalmanFilter.h"
#include "filters/multiplicativeExtendedKalmanFilter.h"

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
    filterData.quaternion = attitude::Quaternion<double>{0.0, 0.0, 0.0, 1.0};
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
    EXPECT_NEAR(filterData.quaternion(0), -0.218338058, 1e-8);
    EXPECT_NEAR(filterData.quaternion(1), 0.066575877, 1e-8);
    EXPECT_NEAR(filterData.quaternion(2), 0.820168143, 1e-8);
    EXPECT_NEAR(filterData.quaternion(3), 0.524614488, 1e-8);
    EXPECT_NEAR(filterData.omega(0), 0.091680646, 1e-8);
    EXPECT_NEAR(filterData.omega(1), -0.121299575, 1e-8);
    EXPECT_NEAR(filterData.omega(2), 0.115997172, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(0), 0.008319353, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(1), 0.021299575, 1e-8);
    EXPECT_NEAR(filterData.omegaBias(2), -0.015997172, 1e-8);
}
