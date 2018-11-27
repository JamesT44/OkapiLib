/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/odometry/threeEncoderOdometry.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>
#include <memory>

using namespace okapi;

class MockModel : public SkidSteerModel {
  public:
  MockModel() : SkidSteerModel(std::make_shared<MockMotor>(), std::make_shared<MockMotor>(), 600) {
  }

  std::valarray<std::int32_t> getSensorVals() const override {
    return std::valarray<std::int32_t>{leftEnc, rightEnc};
  }

  void setSensorVals(std::int32_t left, std::int32_t right) {
    leftEnc = left;
    rightEnc = right;
  }

  std::int32_t leftEnc{0};
  std::int32_t rightEnc{0};
};

void assertOdomStateEquals(Odometry *odom, QLength x, QLength y, QAngle theta) {
  const auto error = 1e-4;
  EXPECT_NEAR(odom->getState().x.convert(meter), x.convert(meter), error);
  EXPECT_NEAR(odom->getState().y.convert(meter), y.convert(meter), error);
  EXPECT_NEAR(odom->getState().theta.convert(degree), theta.convert(degree), error);
}

class OdometryTest : public ::testing::Test {
  protected:
  void SetUp() override {
    model = new MockModel();
    odom = new Odometry(std::shared_ptr<MockModel>(model),
                        ChassisScales({{wheelDiam, wheelbaseWidth}, imev5GreenTPR}),
                        createConstantTimeUtil(10_ms));
  }

  void TearDown() override {
    delete odom;
  }

  QLength calculateDistanceTraveled(int ticks) {
    return (ticks / 360.0) * 1_pi * wheelDiam;
  }

  QLength wheelDiam = 4_in;
  QLength wheelbaseWidth = 10_in;
  MockModel *model;
  Odometry *odom;
};

TEST_F(OdometryTest, NoSensorMovementDoesNotAffectState) {
  assertOdomStateEquals(odom, 0_m, 0_m, 0_deg);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, 0_deg);
}

TEST_F(OdometryTest, MoveForwardTest) {
  model->setSensorVals(10, 10);
  odom->step();
  assertOdomStateEquals(odom, 0_m, calculateDistanceTraveled(10), 0_deg);

  model->setSensorVals(20, 20);
  odom->step();
  assertOdomStateEquals(odom, 0_m, calculateDistanceTraveled(20), 0_deg);

  model->setSensorVals(10, 10);
  odom->step();
  assertOdomStateEquals(odom, 0_m, calculateDistanceTraveled(10), 0_deg);
}

TEST_F(OdometryTest, TurnInPlaceTest) {
  model->setSensorVals(10, -10);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, -4_deg);

  model->setSensorVals(0, 0);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, 0_deg);

  model->setSensorVals(-10, 10);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, 4_deg);
}

TEST_F(OdometryTest, TurnAndDriveTest) {
  model->setSensorVals(90, -90);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, -36_deg);

  model->setSensorVals(180, 0);
  odom->step();
  assertOdomStateEquals(odom,
                        calculateDistanceTraveled(90) * std::sin((-36_deg).convert(radian)),
                        calculateDistanceTraveled(90) * std::cos((-36_deg).convert(radian)),
                        -36_deg);
}

class MockThreeEncoderModel : public ThreeEncoderSkidSteerModel {
  public:
  MockThreeEncoderModel()
    : ThreeEncoderSkidSteerModel(std::make_shared<MockMotor>(),
                                 std::make_shared<MockMotor>(),
                                 std::make_shared<MockMotor>()->getEncoder()) {
  }

  std::valarray<std::int32_t> getSensorVals() const override {
    return std::valarray<std::int32_t>{leftEnc, rightEnc, middleEnc};
  }

  void setSensorVals(std::int32_t left, std::int32_t right, std::int32_t middle) {
    leftEnc = left;
    rightEnc = right;
    middleEnc = middle;
  }

  std::int32_t leftEnc{0};
  std::int32_t rightEnc{0};
  std::int32_t middleEnc{0};
};

class ThreeEncoderOdometryTest : public ::testing::Test {
  protected:
  void SetUp() override {
    model = new MockThreeEncoderModel();
    odom = new ThreeEncoderOdometry(std::shared_ptr<MockThreeEncoderModel>(model),
                                    ChassisScales({{wheelDiam, wheelbaseWidth}, imev5GreenTPR}),
                                    createConstantTimeUtil(10_ms));
  }

  void TearDown() override {
    delete odom;
  }

  QLength calculateDistanceTraveled(int ticks) {
    return (ticks / 360.0) * 1_pi * wheelDiam;
  }

  QLength wheelDiam = 4_in;
  QLength wheelbaseWidth = 10_in;

  MockThreeEncoderModel *model;
  ThreeEncoderOdometry *odom;
};

TEST_F(ThreeEncoderOdometryTest, NoSensorMovementDoesNotAffectState) {
  assertOdomStateEquals(odom, 0_m, 0_m, 0_deg);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, 0_deg);
}

TEST_F(ThreeEncoderOdometryTest, MoveForwardTest) {
  model->setSensorVals(10, 10, 0);
  odom->step();
  assertOdomStateEquals(odom, 0_m, calculateDistanceTraveled(10), 0_deg);

  model->setSensorVals(20, 20, 0);
  odom->step();
  assertOdomStateEquals(odom, 0_m, calculateDistanceTraveled(20), 0_deg);

  model->setSensorVals(10, 10, 0);
  odom->step();
  assertOdomStateEquals(odom, 0_m, calculateDistanceTraveled(10), 0_deg);
}

TEST_F(ThreeEncoderOdometryTest, TurnInPlaceTest) {
  model->setSensorVals(10, -10, -10);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, -4_deg);

  model->setSensorVals(0, 0, 0);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, 0_deg);

  model->setSensorVals(-10, 10, 10);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, 4_deg);
}

TEST_F(ThreeEncoderOdometryTest, TurnAndDriveTest) {
  model->setSensorVals(90, -90, -90);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, -36_deg);

  model->setSensorVals(180, 0, -90);
  odom->step();
  assertOdomStateEquals(odom,
                        calculateDistanceTraveled(90) * std::sin((-36_deg).convert(radian)),
                        calculateDistanceTraveled(90) * std::cos((-36_deg).convert(radian)),
                        -36_deg);
}

TEST_F(ThreeEncoderOdometryTest, StrafeTest) {
  model->setSensorVals(0, 0, 10);
  odom->step();
  assertOdomStateEquals(odom, calculateDistanceTraveled(10), 0_in, 0_deg);
}

TEST_F(ThreeEncoderOdometryTest, DriveForwardWhileStrafingTest) {
  model->setSensorVals(10, 10, 10);
  odom->step();
  assertOdomStateEquals(odom, calculateDistanceTraveled(10), calculateDistanceTraveled(10), 0_deg);
}
