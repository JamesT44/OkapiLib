/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include <cmath>

namespace okapi {
ChassisControllerPID::ChassisControllerPID(const TimeUtil &itimeUtil,
                                           std::unique_ptr<ChassisModel> imodel,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const AbstractMotor::GearsetRatioPair igearset,
                                           const ChassisScales &iscales)
  : ChassisControllerPID(itimeUtil,
                         std::move(imodel),
                         std::make_unique<IterativePosPIDController>(idistanceArgs, itimeUtil),
                         std::make_unique<IterativePosPIDController>(iangleArgs, itimeUtil),
                         igearset,
                         iscales) {
}

ChassisControllerPID::ChassisControllerPID(
  const TimeUtil &itimeUtil,
  std::unique_ptr<ChassisModel> imodel,
  std::unique_ptr<IterativePosPIDController> idistanceController,
  std::unique_ptr<IterativePosPIDController> iangleController,
  const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales)
  : ChassisController(std::move(imodel)),
    rate(std::move(itimeUtil.getRate())),
    distancePid(std::move(idistanceController)),
    anglePid(std::move(iangleController)),
    gearRatio(igearset.ratio),
    straightScale(iscales.straight),
    turnScale(iscales.turn) {
  if (igearset.ratio == 0) {
    throw std::invalid_argument("ChassisControllerPID: The gear ratio cannot be zero! Check if you "
                                "are using integer division.");
  }

  setGearing(igearset.internalGearset);
  setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

void ChassisControllerPID::moveDistance(const QLength itarget) {
  distancePid->reset();
  anglePid->reset();
  distancePid->flipDisable(false);
  anglePid->flipDisable(false);

  const double newTarget = itarget.convert(meter) * straightScale * gearRatio;
  distancePid->setTarget(newTarget);
  anglePid->setTarget(0);

  const auto encStartVals = model->getSensorVals();
  std::valarray<std::int32_t> encVals;
  double distanceElapsed = 0, angleChange = 0;

  while (!distancePid->isSettled() && !anglePid->isSettled()) {
    encVals = model->getSensorVals() - encStartVals;
    distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
    angleChange = static_cast<double>(encVals[1] - encVals[0]);
    model->driveVector(distancePid->step(distanceElapsed), anglePid->step(angleChange));
    rate->delayUntil(10_ms);
  }

  distancePid->flipDisable(true);
  anglePid->flipDisable(true);
  model->stop();
}

void ChassisControllerPID::moveDistance(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistance((itarget / straightScale) * meter);
}

void ChassisControllerPID::turnAngle(const QAngle idegTarget) {
  anglePid->reset();
  anglePid->flipDisable(false);

  const double newTarget = idegTarget.convert(degree) * turnScale * gearRatio;
  anglePid->setTarget(newTarget);

  const auto encStartVals = model->getSensorVals();
  std::valarray<std::int32_t> encVals;
  double angleChange = 0;

  while (!anglePid->isSettled()) {
    encVals = model->getSensorVals() - encStartVals;
    angleChange = static_cast<double>(encVals[1] - encVals[0]);
    model->rotate(anglePid->step(angleChange));
    rate->delayUntil(10_ms);
  }

  anglePid->flipDisable(true);
  model->stop();
}

void ChassisControllerPID::turnAngle(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngle((idegTarget / turnScale) * degree);
}
} // namespace okapi
