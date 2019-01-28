/**
 * @author James Tan
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/TrikeDriveModel.hpp"
#include <algorithm>
#include <utility>

namespace okapi {
TrikeDriveModel::TrikeDriveModel(const std::shared_ptr<AbstractMotor> &ileftSideMotor,
								 const std::shared_ptr<AbstractMotor> &imiddleSideMotor,
								 const std::shared_ptr<AbstractMotor> &irightSideMotor,
								 const std::shared_ptr<ContinuousRotarySensor> &ileftEnc,
								 const std::shared_ptr<ContinuousRotarySensor> &irightEnc,
								 const double imaxVelocity,
								 const double imaxVoltage)
  : ChassisModel::ChassisModel(imaxVelocity, imaxVoltage),
    leftSideMotor(ileftSideMotor),
	middleSideMotor(imiddleSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc) {
}

TrikeDriveModel::TrikeDriveModel(const std::shared_ptr<AbstractMotor> &ileftSideMotor,
								 const std::shared_ptr<AbstractMotor> &imiddleSideMotor,
                                 const std::shared_ptr<AbstractMotor> &irightSideMotor,
                                 const double imaxVelocity,
                                 const double imaxVoltage)
  : ChassisModel::ChassisModel(imaxVelocity, imaxVoltage),
    leftSideMotor(ileftSideMotor),
	middleSideMotor(imiddleSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftSideMotor->getEncoder()),
    rightSensor(irightSideMotor->getEncoder()) {
}

void TrikeDriveModel::forward(const double ispeed) const {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  leftSideMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  middleSideMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  rightSideMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

void TrikeDriveModel::driveVector(const double iforwardSpeed, const double iyaw) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  const double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
  const double yaw = std::clamp(iyaw, -1.0, 1.0);

  double leftOutput = forwardSpeed + yaw;
  double middleOutput = forwardSpeed;
  double rightOutput = forwardSpeed - yaw;
  if (const double maxInputMag = std::max<double>(std::abs(leftOutput), std::abs(rightOutput));
      maxInputMag > 1) {
    leftOutput /= maxInputMag;
	middleOutput /= maxInputMag;
    rightOutput /= maxInputMag;
  }

  leftSideMotor->moveVelocity(static_cast<int16_t>(leftOutput * maxVelocity));
  middleSideMotor->moveVelocity(static_cast<int16_t>(middleOutput * maxVelocity));
  rightSideMotor->moveVelocity(static_cast<int16_t>(rightOutput * maxVelocity));
}

void TrikeDriveModel::rotate(const double ispeed) const {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  leftSideMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  rightSideMotor->moveVelocity(static_cast<int16_t>(-1 * speed * maxVelocity));
}

void TrikeDriveModel::stop() {
  leftSideMotor->moveVelocity(0);
  middleSideMotor->moveVelocity(0);
  rightSideMotor->moveVelocity(0);
}

void TrikeDriveModel::tank(const double ileftSpeed,
                          const double irightSpeed,
                          const double ithreshold) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  double leftSpeed = std::clamp(ileftSpeed, -1.0, 1.0);
  if (std::abs(leftSpeed) < ithreshold) {
    leftSpeed = 0;
  }

  double rightSpeed = std::clamp(irightSpeed, -1.0, 1.0);
  if (std::abs(rightSpeed) < ithreshold) {
    rightSpeed = 0;
  }

  leftSideMotor->moveVoltage(static_cast<int16_t>(leftSpeed * maxVoltage));
  middleSideMotor->moveVoltage(static_cast<int16_t>((leftSpeed + rightSpeed) * 0.5 * maxVoltage));
  rightSideMotor->moveVoltage(static_cast<int16_t>(rightSpeed * maxVoltage));
}

void TrikeDriveModel::arcade(const double iforwardSpeed,
                            const double iyaw,
                            const double ithreshold) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
  if (std::abs(forwardSpeed) < ithreshold) {
    forwardSpeed = 0;
  }

  double yaw = std::clamp(iyaw, -1.0, 1.0);
  if (std::abs(yaw) < ithreshold) {
    yaw = 0;
  }

  double maxInput = std::copysign(std::max(std::abs(forwardSpeed), std::abs(yaw)), forwardSpeed);
  double leftOutput = 0;
  double rightOutput = 0;

  if (forwardSpeed >= 0) {
    if (yaw >= 0) {
      leftOutput = maxInput;
      rightOutput = forwardSpeed - yaw;
    } else {
      leftOutput = forwardSpeed + yaw;
      rightOutput = maxInput;
    }
  } else {
    if (yaw >= 0) {
      leftOutput = forwardSpeed + yaw;
      rightOutput = maxInput;
    } else {
      leftOutput = maxInput;
      rightOutput = forwardSpeed - yaw;
    }
  }

  leftSideMotor->moveVoltage(static_cast<int16_t>(std::clamp(leftOutput, -1.0, 1.0) * maxVoltage));
  middleSideMotor->moveVoltage(static_cast<int16_t>(std::clamp(forwardSpeed, -1.0, 1.0) * maxVoltage));
  rightSideMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(rightOutput, -1.0, 1.0) * maxVoltage));
}

void TrikeDriveModel::left(const double ispeed) const {
  leftSideMotor->moveVelocity(static_cast<int16_t>(std::clamp(ispeed, -1.0, 1.0) * maxVelocity));
  middleSideMotor->moveVelocity(static_cast<int16_t>((std::clamp(ispeed, -1.0, 1.0) * maxVelocity + rightSideMotor->getTargetVelocity()) * 0.5));
}

void TrikeDriveModel::right(const double ispeed) const {
  rightSideMotor->moveVelocity(static_cast<int16_t>(std::clamp(ispeed, -1.0, 1.0) * maxVelocity));
  middleSideMotor->moveVelocity(static_cast<int16_t>((std::clamp(ispeed, -1.0, 1.0) * maxVelocity + leftSideMotor->getTargetVelocity()) * 0.5));
}

std::valarray<std::int32_t> TrikeDriveModel::getSensorVals() const {
  return std::valarray<std::int32_t>{static_cast<std::int32_t>(leftSensor->get()),
                                     static_cast<std::int32_t>(rightSensor->get())};
}

void TrikeDriveModel::resetSensors() const {
  leftSensor->reset();
  rightSensor->reset();
}

void TrikeDriveModel::setBrakeMode(const AbstractMotor::brakeMode mode) const {
  leftSideMotor->setBrakeMode(mode);
  middleSideMotor->setBrakeMode(mode);
  rightSideMotor->setBrakeMode(mode);
}

void TrikeDriveModel::setEncoderUnits(const AbstractMotor::encoderUnits units) const {
  leftSideMotor->setEncoderUnits(units);
  middleSideMotor->setEncoderUnits(units);
  rightSideMotor->setEncoderUnits(units);
}

void TrikeDriveModel::setGearing(const AbstractMotor::gearset gearset) const {
  leftSideMotor->setGearing(gearset);
  middleSideMotor->setGearing(gearset);
  rightSideMotor->setGearing(gearset);
}

void TrikeDriveModel::setPosPID(const double ikF,
                               const double ikP,
                               const double ikI,
                               const double ikD) const {
  leftSideMotor->setPosPID(ikF, ikP, ikI, ikD);
  middleSideMotor->setPosPID(ikF, ikP, ikI, ikD);
  rightSideMotor->setPosPID(ikF, ikP, ikI, ikD);
}

void TrikeDriveModel::setPosPIDFull(const double ikF,
                                   const double ikP,
                                   const double ikI,
                                   const double ikD,
                                   const double ifilter,
                                   const double ilimit,
                                   const double ithreshold,
                                   const double iloopSpeed) const {
  leftSideMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  middleSideMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  rightSideMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
}

void TrikeDriveModel::setVelPID(const double ikF,
                               const double ikP,
                               const double ikI,
                               const double ikD) const {
  leftSideMotor->setVelPID(ikF, ikP, ikI, ikD);
  middleSideMotor->setVelPID(ikF, ikP, ikI, ikD);
  rightSideMotor->setVelPID(ikF, ikP, ikI, ikD);
}

void TrikeDriveModel::setVelPIDFull(const double ikF,
                                   const double ikP,
                                   const double ikI,
                                   const double ikD,
                                   const double ifilter,
                                   const double ilimit,
                                   const double ithreshold,
                                   const double iloopSpeed) const {
  leftSideMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);  middleSideMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  rightSideMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
}

std::shared_ptr<AbstractMotor> TrikeDriveModel::getLeftSideMotor() const {
  return leftSideMotor;
}

std::shared_ptr<AbstractMotor> TrikeDriveModel::getMiddleSideMotor() const {
  return middleSideMotor;
}

std::shared_ptr<AbstractMotor> TrikeDriveModel::getRightSideMotor() const {
  return rightSideMotor;
}
} // namespace okapi
