#include <gtest/gtest.h>

#include "utils/attitudeUtils.h"

GTEST_TEST(attitudeUtilsTests, eulerRotationMatrix)
{
    const attitude::EulerAngle<double> theta = {0.0125, 0.125, 0.0125};
    std::string sequence = "121";
    attitude::RotationMatrix<double> R = attitude::RotationMatrix<double>::Zero();

    bool result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.992197667, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.001558393, 1e-8);
    EXPECT_NEAR(R(0, 2), -0.124664993, 1e-8);
    EXPECT_NEAR(R(1, 0), 0.001558393, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.999688735, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.024899876, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.124664993, 1e-8);
    EXPECT_NEAR(R(2, 1), -0.024899876, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.991886402, 1e-8);

    sequence = "123";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.992120152, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.014056969, 1e-8);
    EXPECT_NEAR(R(0, 2), -0.124499012, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.012402147, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.999824278, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.014056969, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.124674733, 1e-8);
    EXPECT_NEAR(R(2, 1), -0.012402147, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.992120152, 1e-8);

    sequence = "131";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.992197667, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.124664993, 1e-8);
    EXPECT_NEAR(R(0, 2), 0.001558393, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.124664993, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.991886402, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.024899876, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.001558393, 1e-8);
    EXPECT_NEAR(R(2, 1), -0.024899876, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.999688735, 1e-8);

    sequence = "132";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.992120152, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.124811495, 1e-8);
    EXPECT_NEAR(R(0, 2), -0.010940426, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.124674733, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.992120152, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.012402147, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.012402147, 1e-8);
    EXPECT_NEAR(R(2, 1), -0.010940426, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.999863237, 1e-8);

    sequence = "212";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.999688735, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.001558393, 1e-8);
    EXPECT_NEAR(R(0, 2), -0.024899876, 1e-8);
    EXPECT_NEAR(R(1, 0), 0.001558393, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.992197667, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.124664993, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.024899876, 1e-8);
    EXPECT_NEAR(R(2, 1), -0.124664993, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.991886402, 1e-8);

    sequence = "213";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.999863237, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.012402147, 1e-8);
    EXPECT_NEAR(R(0, 2), -0.010940426, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.010940426, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.992120152, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.124811495, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.012402147, 1e-8);
    EXPECT_NEAR(R(2, 1), -0.124674733, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.992120152, 1e-8);

    sequence = "231";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.992120152, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.124674733, 1e-8);
    EXPECT_NEAR(R(0, 2), -0.012402147, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.124499012, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.992120152, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.014056969, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.014056969, 1e-8);
    EXPECT_NEAR(R(2, 1), -0.012402147, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.999824278, 1e-8);

    sequence = "232";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.991886402, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.124664993, 1e-8);
    EXPECT_NEAR(R(0, 2), -0.024899876, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.124664993, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.992197667, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.001558393, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.024899876, 1e-8);
    EXPECT_NEAR(R(2, 1), 0.001558393, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.999688735, 1e-8);

    sequence = "312";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.999824278, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.014056969, 1e-8);
    EXPECT_NEAR(R(0, 2), -0.012402147, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.012402147, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.992120152, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.124674733, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.014056969, 1e-8);
    EXPECT_NEAR(R(2, 1), -0.124499012, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.992120152, 1e-8);

    sequence = "313";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.999688735, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.024899876, 1e-8);
    EXPECT_NEAR(R(0, 2), 0.001558393, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.024899876, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.991886402, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.124664993, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.001558393, 1e-8);
    EXPECT_NEAR(R(2, 1), -0.124664993, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.992197667, 1e-8);

    sequence = "321";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.992120152, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.012402147, 1e-8);
    EXPECT_NEAR(R(0, 2), -0.124674733, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.010940426, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.999863237, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.012402147, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.124811495, 1e-8);
    EXPECT_NEAR(R(2, 1), -0.010940426, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.992120152, 1e-8);

    sequence = "323";
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R);

    // Compare results to the matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(R(0, 0), 0.991886402, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.024899876, 1e-8);
    EXPECT_NEAR(R(0, 2), -0.124664993, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.024899876, 1e-8);
    EXPECT_NEAR(R(1, 1), 0.999688735, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.001558393, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.124664993, 1e-8);
    EXPECT_NEAR(R(2, 1), 0.001558393, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.992197667, 1e-8);

    // Test the default case where the incorrect input was selected
    sequence = "999";
    attitude::RotationMatrix<double> R2 = attitude::RotationMatrix<double>::Zero();
    result = attitude::eulerRotationMatrix<double>(sequence, theta, R2);

    // Compare results to the matlab code
    EXPECT_FALSE(result);
    EXPECT_NEAR(R2(0, 0), 0.0, 1e-8);
    EXPECT_NEAR(R2(0, 1), 0.0, 1e-8);
    EXPECT_NEAR(R2(0, 2), 0.0, 1e-8);
    EXPECT_NEAR(R2(1, 0), 0.0, 1e-8);
    EXPECT_NEAR(R2(1, 1), 0.0, 1e-8);
    EXPECT_NEAR(R2(1, 2), 0.0, 1e-8);
    EXPECT_NEAR(R2(2, 0), 0.0, 1e-8);
    EXPECT_NEAR(R2(2, 1), 0.0, 1e-8);
    EXPECT_NEAR(R2(2, 2), 0.0, 1e-8);
}

GTEST_TEST(attitudeUtilsTests, eulerToQuaternion)
{
    // Test first if statement
    attitude::EulerAngle<double> theta = {0.0125, 0.125, 0.0125};
    std::string sequence = "121";
    attitude::Quaternion<double> quat = attitude::Quaternion<double>::Zero();

    bool result = attitude::eulerToQuaternion<double>(sequence, theta, quat);

    // Compare results to matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(quat(0), 0.012475269, 1e-8);
    EXPECT_NEAR(quat(1), 0.062459317, 1e-8);
    EXPECT_NEAR(quat(2), 0.0, 1e-8);
    EXPECT_NEAR(quat(3), 0.997969539, 1e-8);

    // Test second if statement
    theta = {1.5707963267949, 0.0, 1.5707963267949};
    sequence = "121";

    result = attitude::eulerToQuaternion<double>(sequence, theta, quat);

    // Compare results to matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(quat(0), 1.0, 1e-8);
    EXPECT_NEAR(quat(1), 0.0, 1e-8);
    EXPECT_NEAR(quat(2), 0.0, 1e-8);
    EXPECT_NEAR(quat(3), 0.0, 1e-8);

    // Test third if statement
    theta = {1.5707963267949, 0.0, 1.5707963267949};
    sequence = "212";

    result = attitude::eulerToQuaternion<double>(sequence, theta, quat);

    // Compare results to matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(quat(0), 0.0, 1e-8);
    EXPECT_NEAR(quat(1), 1.0, 1e-8);
    EXPECT_NEAR(quat(2), 0.0, 1e-8);
    EXPECT_NEAR(quat(3), 0.0, 1e-8);

    // Test fourth if statement
    theta = {1.56789737588422, 1.52629503466588, 0.00320881333415};
    sequence = "312";

    result = attitude::eulerToQuaternion<double>(sequence, theta, quat);

    // Compare results to matlab code
    EXPECT_TRUE(result);
    EXPECT_NEAR(quat(0), 0.488640453, 1e-8);
    EXPECT_NEAR(quat(1), 0.488863292, 1e-8);
    EXPECT_NEAR(quat(2), 0.511044053, 1e-8);
    EXPECT_NEAR(quat(3), 0.510957105, 1e-8);
}

GTEST_TEST(attitudeUtilsTests, quaternionRotationMatrix)
{
    const attitude::Quaternion<double> quat{0.48884476173003, 0.489803421811499, 0.51099650931451, 0.509907809686459};

    const attitude::RotationMatrix<double> R = attitude::quaternionRotationMatrix<double>(quat);

    // Compare results to the matlab code
    EXPECT_NEAR(R(0, 0), -0.002049649, 1e-8);
    EXPECT_NEAR(R(0, 1), 0.999997895, 1e-8);
    EXPECT_NEAR(R(0, 2), 0.000086753, 1e-8);
    EXPECT_NEAR(R(1, 0), -0.042246547, 1e-8);
    EXPECT_NEAR(R(1, 1), -0.000173267, 1e-8);
    EXPECT_NEAR(R(1, 2), 0.999107201, 1e-8);
    EXPECT_NEAR(R(2, 0), 0.999105113, 1e-8);
    EXPECT_NEAR(R(2, 1), 0.002044154, 1e-8);
    EXPECT_NEAR(R(2, 2), 0.042246813, 1e-8);
}

GTEST_TEST(attitudeUtilsTests, quaternionToEuler)
{
    // All tests should result back in this value
    const attitude::EulerAngle<double> theta = {0.0125, 0.125, 0.0125};

    // Test sequence
    std::string sequence = "121";
    attitude::Quaternion<double> quat = attitude::Quaternion<double>::Zero();
    attitude::EulerAngle<double> thetaCompare = attitude::EulerAngle<double>::Zero();
    bool result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    bool result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "123";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "131";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "132";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "212";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "213";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "231";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "232";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "312";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "313";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "321";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test sequence
    sequence = "323";
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare);

    // Compare results
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_NEAR(thetaCompare(0), theta(0), 1e-8);
    EXPECT_NEAR(thetaCompare(1), theta(1), 1e-8);
    EXPECT_NEAR(thetaCompare(2), theta(2), 1e-8);

    // Test default sequence
    sequence = "999";
    attitude::EulerAngle<double> thetaCompare2 = attitude::EulerAngle<double>::Zero();
    result1 = attitude::eulerToQuaternion<double>(sequence, theta, quat);
    result2 = attitude::quaternionToEuler<double>(sequence, quat, thetaCompare2);

    // Compare results
    EXPECT_FALSE(result1);
    EXPECT_FALSE(result2);
    EXPECT_NEAR(thetaCompare2(0), 0.0, 1e-8);
    EXPECT_NEAR(thetaCompare2(1), 0.0, 1e-8);
    EXPECT_NEAR(thetaCompare2(2), 0.0, 1e-8);
}

GTEST_TEST(attitudeUtilsTests, quaternionMutiplyInverse)
{
    const attitude::Quaternion<double> quat = {0.5, 0.5, 0.5, 0.5};

    // From previous tests we know other attitude functions should work correctly
    const attitude::EulerAngle<double> thetaDes = {40.0 * M_PI / 180.0, 15.0 * M_PI / 180.0, 0.0};
    const std::string sequence = "321";
    attitude::Quaternion<double> quatDes = attitude::Quaternion<double>::Zero();
    bool result = attitude::eulerToQuaternion<double>(sequence, thetaDes, quatDes);

    const attitude::Quaternion<double> quatError = attitude::quatMultiply<double>(quat, attitude::quatInverse<double>(quatDes));

    // Compare results to the matlab code
    EXPECT_NEAR(quatError(0), 0.596367810, 1e-8);
    EXPECT_NEAR(quatError(1), 0.212631109, 1e-8);
    EXPECT_NEAR(quatError(2), 0.379928196, 1e-8);
    EXPECT_NEAR(quatError(3), 0.674379723, 1e-8);
}

GTEST_TEST(attitudeUtilsTests, quaternionKinematics)
{
    const attitude::Quaternion<double> quat = {0.5, 0.5, 0.5, 0.5};
    const attitude::BodyRate<double> omega = {0.1, 0.2, 0.3};
    const attitude::Quaternion<double> quatRate = attitude::quaternionKinematics<double>(quat, omega);

    // Compare results to the matlab code
    EXPECT_NEAR(quatRate(0), 0.05, 1e-8);
    EXPECT_NEAR(quatRate(1), 0.0, 1e-8);
    EXPECT_NEAR(quatRate(2), 0.1, 1e-8);
    EXPECT_NEAR(quatRate(3), -0.15, 1e-8);
}
