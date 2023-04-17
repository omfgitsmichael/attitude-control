#include <gtest/gtest.h>

#include "utils/controlUtils.h"

GTEST_TEST(controlUtilsTests, deadzoneOperator)
{
    Eigen::Vector<double, 3> error{0.1, 0.1, 0.1};
    attitude::DeadzoneParams<double> params;

    std::optional<double> deadzone = attitude::deadzoneOperator(params, error);

    // Should fail if the parameters aren't set to proper values
    ASSERT_FALSE(deadzone);

    params.del = 0.5;
    params.e0 = 0.5;

    deadzone = attitude::deadzoneOperator(params, error);

    // Should pass with proper parameters and compare results with matlab
    ASSERT_TRUE(deadzone);
    EXPECT_NEAR(*deadzone, 0.0, 1e-8);

    error = {0.2, 0.2, 0.2};
    deadzone = attitude::deadzoneOperator(params, error);

    // Compare results with matlab
    ASSERT_TRUE(deadzone);
    EXPECT_NEAR(*deadzone, 0.385640646, 1e-8);

    error = {0.3, 0.3, 0.3};
    deadzone = attitude::deadzoneOperator(params, error);

    // Compare results with matlab
    ASSERT_TRUE(deadzone);
    EXPECT_NEAR(*deadzone, 1.0, 1e-8);
}

GTEST_TEST(controlUtilsTests, projectionOperator)
{
    const Eigen::Matrix<double, 3, 3> gamma{{10.0, 0.0, 0.0}, {0.0, 0.00001, 0.0}, {0.0, 0.0, 1.0}};
    Eigen::Vector<double, 3> theta{0.0, 0.0, 0.0};
    Eigen::Vector<double, 3> adaptiveLaws{0.0248815913789445, 0.000477324083807007, -0.0702842735971776};
    Eigen::Vector<double, 3> adaptationLaws = gamma * adaptiveLaws;
    attitude::ProjectionParams<double, 1> params;

    std::optional<Eigen::Vector<double, 3>> projection = attitude::projectionOperator(params, theta, adaptationLaws);
    
    // Should fail if the parameters aren't set to proper values
    ASSERT_FALSE(projection);

    params.epsilon(0) = {0.5};
    params.thetaMax(0) = {3000.0};

    projection = attitude::projectionOperator(params, theta, adaptationLaws);

    // Should pass with proper parameters and compare results with matlab
    ASSERT_TRUE(projection);
    EXPECT_NEAR((*projection)(0), 0.248815913, 1e-8);
    EXPECT_NEAR((*projection)(1), 4.77e-9, 1e-8);
    EXPECT_NEAR((*projection)(2), -0.070284273, 1e-8);

    theta = {2436.39217465102, 0.848434409920056, 260.410281334132};
    adaptiveLaws = {18.2265018846469, 14692.6850058268, 21.439413080977};
    adaptationLaws = gamma * adaptiveLaws;

    projection = attitude::projectionOperator(params, theta, adaptationLaws);

    // Should pass with proper parameters and compare results with matlab
    ASSERT_TRUE(projection);
    EXPECT_NEAR((*projection)(0), 182.032606511, 1e-8);
    EXPECT_NEAR((*projection)(1), 0.146845916, 1e-8);
    EXPECT_NEAR((*projection)(2), 21.414572022, 1e-8);
}
