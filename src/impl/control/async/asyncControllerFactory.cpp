/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/async/asyncControllerFactory.hpp"
#include "okapi/impl/filter/velMathFactory.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace okapi {
AsyncPosIntegratedController AsyncControllerFactory::posIntegrated(Motor imotor) {
  return AsyncPosIntegratedController(std::make_shared<Motor>(imotor), TimeUtilFactory::create());
}

AsyncPosIntegratedController AsyncControllerFactory::posIntegrated(MotorGroup imotor) {
  return AsyncPosIntegratedController(std::make_shared<MotorGroup>(imotor),
                                      TimeUtilFactory::create());
}

AsyncVelIntegratedController AsyncControllerFactory::velIntegrated(Motor imotor) {
  return AsyncVelIntegratedController(std::make_shared<Motor>(imotor), TimeUtilFactory::create());
}

AsyncVelIntegratedController AsyncControllerFactory::velIntegrated(MotorGroup imotor) {
  return AsyncVelIntegratedController(std::make_shared<MotorGroup>(imotor),
                                      TimeUtilFactory::create());
}

AsyncPosPIDController AsyncControllerFactory::posPID(Motor imotor,
                                                     const double ikP,
                                                     const double ikI,
                                                     const double ikD,
                                                     const double ikBias,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncPosPIDController(imotor.getEncoder(),
                               std::make_shared<Motor>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikI,
                               ikD,
                               ikBias,
                               std::move(iderivativeFilter));
}

AsyncPosPIDController AsyncControllerFactory::posPID(Motor imotor,
                                                     ADIEncoder ienc,
                                                     const double ikP,
                                                     const double ikI,
                                                     const double ikD,
                                                     const double ikBias,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncPosPIDController(std::make_shared<ADIEncoder>(ienc),
                               std::make_shared<Motor>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikI,
                               ikD,
                               ikBias,
                               std::move(iderivativeFilter));
}

AsyncPosPIDController AsyncControllerFactory::posPID(Motor imotor,
                                                     Potentiometer ipot,
                                                     const double ikP,
                                                     const double ikI,
                                                     const double ikD,
                                                     const double ikBias,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncPosPIDController(std::make_shared<Potentiometer>(ipot),
                               std::make_shared<Motor>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikI,
                               ikD,
                               ikBias,
                               std::move(iderivativeFilter));
}

AsyncPosPIDController AsyncControllerFactory::posPID(MotorGroup imotor,
                                                     const double ikP,
                                                     const double ikI,
                                                     const double ikD,
                                                     const double ikBias,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncPosPIDController(imotor.getEncoder(),
                               std::make_shared<MotorGroup>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikI,
                               ikD,
                               ikBias,
                               std::move(iderivativeFilter));
}

AsyncPosPIDController AsyncControllerFactory::posPID(MotorGroup imotor,
                                                     ADIEncoder ienc,
                                                     const double ikP,
                                                     const double ikI,
                                                     const double ikD,
                                                     const double ikBias,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncPosPIDController(std::make_shared<ADIEncoder>(ienc),
                               std::make_shared<MotorGroup>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikI,
                               ikD,
                               ikBias,
                               std::move(iderivativeFilter));
}

AsyncPosPIDController AsyncControllerFactory::posPID(MotorGroup imotor,
                                                     Potentiometer ipot,
                                                     const double ikP,
                                                     const double ikI,
                                                     const double ikD,
                                                     const double ikBias,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncPosPIDController(std::make_shared<Potentiometer>(ipot),
                               std::make_shared<MotorGroup>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikI,
                               ikD,
                               ikBias,
                               std::move(iderivativeFilter));
}

AsyncPosPIDController
AsyncControllerFactory::posPID(std::shared_ptr<ControllerInput<double>> iinput,
                               std::shared_ptr<ControllerOutput<double>> ioutput,
                               const double ikP,
                               const double ikI,
                               const double ikD,
                               const double ikBias,
                               std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncPosPIDController(iinput,
                               ioutput,
                               TimeUtilFactory::create(),
                               ikP,
                               ikI,
                               ikD,
                               ikBias,
                               std::move(iderivativeFilter));
}

AsyncVelPIDController AsyncControllerFactory::velPID(Motor imotor,
                                                     const double ikP,
                                                     const double ikD,
                                                     const double ikF,
                                                     const double iTPR,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncVelPIDController(imotor.getEncoder(),
                               std::make_shared<Motor>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikD,
                               ikF,
                               VelMathFactory::createPtr(iTPR),
                               std::move(iderivativeFilter));
}

AsyncVelPIDController AsyncControllerFactory::velPID(Motor imotor,
                                                     ADIEncoder ienc,
                                                     const double ikP,
                                                     const double ikD,
                                                     const double ikF,
                                                     const double iTPR,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncVelPIDController(std::make_shared<ADIEncoder>(ienc),
                               std::make_shared<Motor>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikD,
                               ikF,
                               VelMathFactory::createPtr(iTPR),
                               std::move(iderivativeFilter));
}

AsyncVelPIDController AsyncControllerFactory::velPID(Motor imotor,
                                                     Potentiometer ipot,
                                                     const double ikP,
                                                     const double ikD,
                                                     const double ikF,
                                                     const double iTPR,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncVelPIDController(std::make_shared<Potentiometer>(ipot),
                               std::make_shared<Motor>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikD,
                               ikF,
                               VelMathFactory::createPtr(iTPR),
                               std::move(iderivativeFilter));
}

AsyncVelPIDController AsyncControllerFactory::velPID(MotorGroup imotor,
                                                     const double ikP,
                                                     const double ikD,
                                                     const double ikF,
                                                     const double iTPR,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncVelPIDController(imotor.getEncoder(),
                               std::make_shared<MotorGroup>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikD,
                               ikF,
                               VelMathFactory::createPtr(iTPR),
                               std::move(iderivativeFilter));
}

AsyncVelPIDController AsyncControllerFactory::velPID(MotorGroup imotor,
                                                     ADIEncoder ienc,
                                                     const double ikP,
                                                     const double ikD,
                                                     const double ikF,
                                                     const double iTPR,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncVelPIDController(std::make_shared<ADIEncoder>(ienc),
                               std::make_shared<MotorGroup>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikD,
                               ikF,
                               VelMathFactory::createPtr(iTPR),
                               std::move(iderivativeFilter));
}

AsyncVelPIDController AsyncControllerFactory::velPID(MotorGroup imotor,
                                                     Potentiometer ipot,
                                                     const double ikP,
                                                     const double ikD,
                                                     const double ikF,
                                                     const double iTPR,
                                                     std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncVelPIDController(std::make_shared<Potentiometer>(ipot),
                               std::make_shared<MotorGroup>(imotor),
                               TimeUtilFactory::create(),
                               ikP,
                               ikD,
                               ikF,
                               VelMathFactory::createPtr(iTPR),
                               std::move(iderivativeFilter));
}

AsyncVelPIDController
AsyncControllerFactory::velPID(std::shared_ptr<ControllerInput<double>> iinput,
                               std::shared_ptr<ControllerOutput<double>> ioutput,
                               const double ikP,
                               const double ikD,
                               const double ikF,
                               const double iTPR,
                               std::unique_ptr<Filter> iderivativeFilter) {
  return AsyncVelPIDController(iinput,
                               ioutput,
                               TimeUtilFactory::create(),
                               ikP,
                               ikD,
                               ikF,
                               VelMathFactory::createPtr(iTPR),
                               std::move(iderivativeFilter));
}

AsyncMotionProfileController
AsyncControllerFactory::motionProfile(double imaxVel,
                                      double imaxAccel,
                                      double imaxJerk,
                                      std::shared_ptr<SkidSteerModel> imodel,
                                      QLength iwidth) {
  return AsyncMotionProfileController(
    TimeUtilFactory::create(), imaxVel, imaxAccel, imaxJerk, imodel, iwidth);
}

AsyncMotionProfileController
AsyncControllerFactory::motionProfile(double imaxVel,
                                      double imaxAccel,
                                      double imaxJerk,
                                      const ChassisController &ichassis,
                                      QLength iwidth) {
  return AsyncMotionProfileController(
    TimeUtilFactory::create(), imaxVel, imaxAccel, imaxJerk, ichassis.getChassisModel(), iwidth);
}
} // namespace okapi
