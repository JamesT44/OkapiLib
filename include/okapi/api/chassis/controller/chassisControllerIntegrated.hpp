/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLERINTEGRATED_HPP_
#define _OKAPI_CHASSISCONTROLLERINTEGRATED_HPP_

#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"

namespace okapi {
class ChassisControllerIntegrated : public virtual ChassisController {
  public:
  /**
   * ChassisController using the V5 motor's integrated control. Puts the motors into degree units.
   * Throws a std::invalid_argument exception if the gear ratio is zero.
   *
   * @param imodelArgs ChassisModelArgs
   * @param ileftControllerArgs left side controller params
   * @param irightControllerArgs right side controller params
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  ChassisControllerIntegrated(
    std::shared_ptr<ChassisModel> imodel,
    const AsyncPosIntegratedControllerArgs &ileftControllerArgs,
    const AsyncPosIntegratedControllerArgs &irightControllerArgs,
    const AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
    const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel
   */
  virtual void moveDistance(const QLength itarget) override;

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel in motor degrees
   */
  virtual void moveDistance(const double itarget) override;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for
   */
  virtual void turnAngle(const QAngle idegTarget) override;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for in motor degrees
   */
  virtual void turnAngle(const double idegTarget) override;

  protected:
  AsyncPosIntegratedController leftController;
  AsyncPosIntegratedController rightController;
  int lastTarget;
  const double gearRatio;
  const double straightScale;
  const double turnScale;
};
} // namespace okapi

#endif
