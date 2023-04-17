#include <gtest/gtest.h>

#include "controllers/passivityBasedAdaptiveControl.h"
#include "utils/attitudeUtils.h"

GTEST_TEST(controllerTests, passivityBasedAdaptiveControl)
{
    // Initialize the params
    attitude::control::PassivityParams<double> params;
    params.deadzoneParams.del = 0.5;
    params.deadzoneParams.e0 = 0.5;
    params.projectionParams.epsilon(0) = {0.5};
    params.projectionParams.thetaMax(0) = {3000.0};
    params.lambda = Eigen::Matrix<double, 3, 3>{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    params.k = Eigen::Matrix<double, 3, 3>{{2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 2.0}};
    params.gammaInv = Eigen::Matrix<double, 3, 3>{{0.1, 0.0, 0.0}, {0.0, 0.1, 0.0}, {0.0, 0.0, 0.1}}.inverse();
    params.dt = 0.1;

    // Initialize the data
    attitude::PassivityControlData<double> data;
    data.theta = attitude::control::Theta<double>{1.0, 1.0, 1.0};
    data.quat = attitude::Quaternion<double>{std::sqrt(4.0) / 4.0, std::sqrt(4.0) / 4.0, std::sqrt(4.0) / 4.0, std::sqrt(4.0) / 4.0};

    attitude::EulerAngle<double> rpyDes{40.0 * M_PI / 180.0, 15.0 * M_PI / 180.0, 0.0};
    attitude::OptionalQuaternion<double> quadDes = attitude::eulerToQuaternion<double>("321", rpyDes);
    ASSERT_TRUE(quadDes);
    data.quatDesired = *quadDes;

    bool result = attitude::control::passivityBasedAdaptiveControl<double>(params, data);

    // Compare results to matlab
    EXPECT_TRUE(result);
    EXPECT_NEAR(data.u(0), -1.192735621, 1e-8);
    EXPECT_NEAR(data.u(1), -0.425262219, 1e-8);
    EXPECT_NEAR(data.u(2), -0.759856393, 1e-8);
    EXPECT_NEAR(data.theta(0), 1.0, 1e-8);
    EXPECT_NEAR(data.theta(1), 1.0, 1e-8);
    EXPECT_NEAR(data.theta(2), 1.0, 1e-8);

    data.omega = attitude::BodyRate<double>{-0.119530608034409, -0.0212043380053971, -0.025407576271625};
    data.quat = attitude::Quaternion<double>{0.498448741197208, 0.498552409626682, 0.500910323804368, 0.502078873047404};

    result = attitude::control::passivityBasedAdaptiveControl<double>(params, data);

    // Compare results to matlab
    EXPECT_TRUE(result);
    EXPECT_NEAR(data.u(0), -0.914028229, 1e-8);
    EXPECT_NEAR(data.u(1), -0.328929057, 1e-8);
    EXPECT_NEAR(data.u(2), -0.718842133, 1e-8);
    EXPECT_NEAR(data.theta(0), 0.981756035, 1e-8);
    EXPECT_NEAR(data.theta(1), 0.995109674, 1e-8);
    EXPECT_NEAR(data.theta(2), 0.999522835, 1e-8);
}