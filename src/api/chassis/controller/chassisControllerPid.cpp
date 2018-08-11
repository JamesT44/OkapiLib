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
ChassisControllerPID::ChassisControllerPID(
  const TimeUtil &itimeUtil, std::shared_ptr<ChassisModel> imodel,
  std::unique_ptr<IterativePosPIDController> idistanceController,
  std::unique_ptr<IterativePosPIDController> iangleController,
  std::unique_ptr<IterativePosPIDController> iturnController,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales)
  : ChassisController(imodel),
    rate(itimeUtil.getRate()),
    distancePid(std::move(idistanceController)),
    anglePid(std::move(iangleController)),
    turnPid(std::move(iturnController)),
    gearRatio(igearset.ratio),
    straightScale(iscales.straight),
    turnScale(iscales.turn),
    task(trampoline, this) {
  if (igearset.ratio == 0) {
    logger->error("ChassisControllerPID: The gear ratio cannot be zero! Check if you are using "
                  "integer division.");
    throw std::invalid_argument("ChassisControllerPID: The gear ratio cannot be zero! Check if you "
                                "are using integer division.");
  }

  setGearing(igearset.internalGearset);
  setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

ChassisControllerPID::~ChassisControllerPID() {
  dtorCalled = true;
}

void ChassisControllerPID::loop() {
  auto encStartVals = model->getSensorVals();
  std::valarray<std::int32_t> encVals;
  double distanceElapsed = 0, angleChange = 0;
  modeType pastMode = distance;

  while (!dtorCalled) {
    /**
     * doneLooping is set to false by moveDistanceAsync and turnAngleAsync and then set to true by
     * waitUntilSettled
     */
    if (!doneLooping) {
      if (mode != pastMode) {
        logger->debug("ChassisControllerPID: Changed mode");

        encStartVals = model->getSensorVals();
        distancePid->reset();
        anglePid->reset();
        turnPid->reset();
        model->stop();
      }

      switch (mode) {
      case distance:
        encVals = model->getSensorVals() - encStartVals;
        distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
        angleChange = static_cast<double>(encVals[0] - encVals[1]);
        model->driveVector(distancePid->step(distanceElapsed), anglePid->step(angleChange));

        break;
      case angle:
        encVals = model->getSensorVals() - encStartVals;
        angleChange = static_cast<double>(encVals[0] - encVals[1]);
        model->rotate(turnPid->step(angleChange));

        break;
      default:
        break;
      }

      pastMode = mode;
    }

    rate->delayUntil(10_ms);
  }
}

void ChassisControllerPID::trampoline(void *context) {
  if (context) {
    static_cast<ChassisControllerPID *>(context)->loop();
  }
}

void ChassisControllerPID::moveDistanceAsync(const QLength itarget) {
  logger->info("ChassisControllerPID: moving " + std::to_string(itarget.convert(meter)) +
               " meters");

  distancePid->reset();
  anglePid->reset();
  distancePid->flipDisable(false);
  anglePid->flipDisable(false);
  turnPid->flipDisable(true);
  mode = distance;

  const double newTarget = itarget.convert(meter) * straightScale * gearRatio;

  logger->info("ChassisControllerPID: moving " + std::to_string(newTarget) + " motor degrees");

  distancePid->setTarget(newTarget);
  anglePid->setTarget(0);

  doneLooping = false;
}

void ChassisControllerPID::moveDistanceAsync(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistanceAsync((itarget / straightScale) * meter);
}

void ChassisControllerPID::moveDistance(const QLength itarget) {
  moveDistanceAsync(itarget);
  waitUntilSettled();
}

void ChassisControllerPID::moveDistance(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistance((itarget / straightScale) * meter);
}

void ChassisControllerPID::turnAngleAsync(const QAngle idegTarget) {
  logger->info("ChassisControllerPID: turning " + std::to_string(idegTarget.convert(degree)) +
               " degrees");

  turnPid->reset();
  turnPid->flipDisable(false);
  distancePid->flipDisable(true);
  anglePid->flipDisable(true);
  mode = angle;

  const double newTarget = idegTarget.convert(degree) * turnScale * gearRatio;

  logger->info("ChassisControllerPID: turning " + std::to_string(newTarget) + " motor degrees");

  turnPid->setTarget(newTarget);

  doneLooping = false;
}

void ChassisControllerPID::turnAngleAsync(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngleAsync((idegTarget / turnScale) * degree);
}

void ChassisControllerPID::turnAngle(const QAngle idegTarget) {
  turnAngleAsync(idegTarget);
  waitUntilSettled();
}

void ChassisControllerPID::turnAngle(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngle((idegTarget / turnScale) * degree);
}

void ChassisControllerPID::waitUntilSettled() {
  logger->info("ChassisControllerPID: Waiting to settle");
  bool completelySettled = false;

  while (!completelySettled) {
    switch (mode) {
    case distance:
      completelySettled = waitForDistanceSettled();

      // Only disable the controllers and stop if we are totally settled and won't try again
      if (completelySettled) {
        stopAfterSettled();
      }

      break;

    case angle:
      completelySettled = waitForAngleSettled();

      // Only disable the controllers and stop if we are totally settled and won't try again
      if (completelySettled) {
        stopAfterSettled();
      }

      break;

    default:
      break;
    }

    logger->info("ChassisControllerPID: Done waiting to settle");
  }
}

/**
 * Wait for the distance setup (distancePid and anglePid) to settle.
 *
 * @return true if done settling; false if settling should be tried again
 */
bool ChassisControllerPID::waitForDistanceSettled() {
  logger->info("ChassisControllerPID: Waiting to settle in distance mode");

  while (!(distancePid->isSettled() && anglePid->isSettled())) {
    if (mode == angle) {
      // False will cause the loop to re-enter the switch
      logger->warn("ChassisControllerPID: Mode changed to angle while waiting in distance!");
      return false;
    }

    rate->delayUntil(10_ms);
  }

  // True will cause the loop to exit
  return true;
}

/**
 * Wait for the angle setup (anglePid) to settle.
 *
 * @return true if done settling; false if settling should be tried again
 */
bool ChassisControllerPID::waitForAngleSettled() {
  logger->info("ChassisControllerPID: Waiting to settle in angle mode");

  while (!turnPid->isSettled()) {
    if (mode == distance) {
      // False will cause the loop to re-enter the switch
      logger->warn("ChassisControllerPID: Mode changed to distance while waiting in angle!");
      return false;
    }

    rate->delayUntil(10_ms);
  }

  // True will cause the loop to exit
  return true;
}

void ChassisControllerPID::stopAfterSettled() {
  switch (mode) {
  case distance:
    doneLooping = true;
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    model->stop();
    break;

  case angle:
    doneLooping = true;
    turnPid->flipDisable(true);
    model->stop();
    break;

  default:
    break;
  }
}

void ChassisControllerPID::stop() {
  stopAfterSettled();
  ChassisController::stop();
}
} // namespace okapi
