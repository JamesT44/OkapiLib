/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "api.h"

#include "okapi/api.hpp"
#include "test/testRunner.hpp"
#include "test/tests/impl/allImplTests.hpp"

void runHeadlessTests() {
  using namespace okapi;

  runHeadlessUtilTests();

  test_print_report();
}

void opcontrol() {
  using namespace okapi;
  pros::Task::delay(100);

  Logger::initialize(std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::debug);
  auto logger = Logger::instance();

  //  {
  //    auto model =
  //      std::make_shared<SkidSteerModel>(std::make_shared<Motor>(-1), std::make_shared<Motor>(2));
  //    auto cnt = AsyncControllerFactory::motionProfile(1.0, 2.0, 10.0, model, 10.5_in);
  //
  //    cnt.generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3_ft, 0_ft, 0_deg}}, "A");
  //    cnt.setTarget("B");
  //    cnt.waitUntilSettled();
  //  }

  //    auto drive =
  //      ChassisControllerFactory::create(-1, 2, AbstractMotor::gearset::red, {2.5_in, 10.5_in});
  //  drive.moveDistanceAsync(2_in);
  //  drive.waitUntilSettled();

  {
    AsyncControllerBuilder builder(TimeUtilFactory::create());
    auto cnt = builder.input(IntegratedEncoder(Motor(1))).posPid(1, 0, 0).output(Motor(-1)).build();
    cnt.setTarget(1800);
    while (true) {
      logger->debug("opcontrol: enc " + std::to_string(Motor(1).getPosition()));
      pros::Task::delay(20);
    }
  }

  //  runHeadlessTests();
  return;

  MotorGroup leftMotors({19_mtr, 20_mtr});
  MotorGroup rightMotors({13_rmtr, 14_rmtr});
  Motor armMotor = 15_mtr;
  armMotor.move(10);

  auto chassis =
    ChassisControllerFactory::create({19, 20}, {-14}, AbstractMotor::gearset::red, {4_in, 11.5_in});

  Controller controller;
  ControllerButton btn1(E_CONTROLLER_DIGITAL_A);
  ControllerButton btn2(E_CONTROLLER_DIGITAL_B);
  ControllerButton btn3(E_CONTROLLER_DIGITAL_Y);
  ControllerButton btn4(E_CONTROLLER_DIGITAL_X);

  while (true) {
    chassis.arcade(controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_Y),
                   controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_X));

    if (btn1.changedToPressed()) {
      printf("move distance\n");
      chassis.moveDistance(12_in);
    }

    if (btn2.changedToPressed()) {
      printf("turn angle\n");
      chassis.turnAngle(90_deg);
    }

    if (btn3.changedToPressed()) {
      printf("move arm\n");
      armMotor.moveRelative(-10, 127);
    }

    if (btn4.changedToPressed()) {
      printf("autonomous routine\n");
      for (int i = 0; i < 4; i++) {
        chassis.moveDistance(12_in);
        chassis.turnAngle(90_deg);
      }
    }

    pros::Task::delay(10);
  }
}
