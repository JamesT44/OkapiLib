/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_IMPLMOCKS_HPP_
#define _OKAPI_IMPLMOCKS_HPP_

#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/control/util/flywheelSimulator.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <chrono>
#include <gtest/gtest.h>

namespace okapi {

class MockContinuousRotarySensor : public ContinuousRotarySensor {
  public:
  double controllerGet() override;

  int32_t reset() override;

  double get() const override;

  mutable std::int32_t value{0};
};

/**
 * A motor mock that saves the last set position, velocity, and voltage.
 */
class MockMotor : public AbstractMotor {
  public:
  MockMotor();

  void controllerSet(double ivalue) override;

  int32_t moveAbsolute(double iposition, std::int32_t ivelocity) const override;

  int32_t moveRelative(double iposition, std::int32_t ivelocity) const override;

  double getTargetPosition() const override;

  double getPosition() const override;

  int32_t getTargetVelocity() const override;

  double getActualVelocity() const override;

  int32_t tarePosition() const override;

  int32_t setBrakeMode(brakeMode imode) const override;

  int32_t setCurrentLimit(std::int32_t ilimit) const override;

  int32_t setEncoderUnits(encoderUnits iunits) const override;

  int32_t setGearing(gearset igearset) const override;

  int32_t setReversed(bool ireverse) const override;

  int32_t setVoltageLimit(std::int32_t ilimit) const override;

  std::shared_ptr<ContinuousRotarySensor> getEncoder() const override;

  std::int32_t moveVelocity(std::int16_t ivelocity) const override;

  std::int32_t moveVoltage(std::int16_t ivoltage) const override;

  std::shared_ptr<MockContinuousRotarySensor> encoder;
  mutable std::int16_t lastVelocity{0};
  mutable std::int16_t maxVelocity{0};
  mutable std::int16_t lastVoltage{0};
  mutable std::int16_t lastPosition{0};
};

/**
 * A timer mock that implements all features using the system timer.
 */
class MockTimer : public AbstractTimer {
  public:
  MockTimer();

  QTime millis() const override;

  QTime getDt() override;

  QTime getStartingTime() const override;

  QTime getDtFromStart() const override;

  void placeMark() override;

  QTime clearMark() override;

  void placeHardMark() override;

  QTime clearHardMark() override;

  QTime getDtFromMark() const override;

  QTime getDtFromHardMark() const override;

  bool repeat(QTime time) override;

  bool repeat(QFrequency frequency) override;

  std::chrono::system_clock::time_point epoch = std::chrono::high_resolution_clock::from_time_t(0);
  QTime firstCalled;
  QTime lastCalled;
  QTime mark;
  QTime hardMark;
  QTime repeatMark;
};

/**
 * A timer mock that always returns a constant dt and 0 for other methods.
 */
class ConstantMockTimer : public AbstractTimer {
  public:
  explicit ConstantMockTimer(QTime idt);

  QTime millis() const override;

  QTime getDt() override;

  QTime getStartingTime() const override;

  QTime getDtFromStart() const override;

  void placeMark() override;

  QTime clearMark() override;

  void placeHardMark() override;

  QTime clearHardMark() override;

  QTime getDtFromMark() const override;

  QTime getDtFromHardMark() const override;

  bool repeat(QTime time) override;

  bool repeat(QFrequency frequency) override;

  QTime dtToReturn;
};

class MockRate : public AbstractRate {
  public:
  MockRate();

  void delay(QFrequency ihz) override;

  void delay(int ihz) override;

  void delayUntil(QTime itime) override;

  void delayUntil(uint32_t ims) override;
};

std::unique_ptr<SettledUtil> createSettledUtilPtr(double iatTargetError = 50,
                                                  double iatTargetDerivative = 5,
                                                  QTime iatTargetTime = 250_ms);

TimeUtil createTimeUtil();

TimeUtil createTimeUtil(const Supplier<std::unique_ptr<AbstractTimer>> &itimerSupplier);

TimeUtil createTimeUtil(const Supplier<std::unique_ptr<SettledUtil>> &isettledUtilSupplier);

class SimulatedSystem : public ControllerInput<double>, public ControllerOutput<double> {
  public:
  explicit SimulatedSystem(FlywheelSimulator &simulator);

  virtual ~SimulatedSystem();

  double controllerGet() override;

  void controllerSet(double ivalue) override;

  void step();

  static void trampoline(void *system);

  void join();

  FlywheelSimulator &simulator;
  std::thread thread;
  MockRate rate{};
  bool shouldJoin = false;
};

class MockAsyncController : public AsyncPosIntegratedController {
  public:
  MockAsyncController()
    : AsyncPosIntegratedController(std::make_shared<MockMotor>(), createTimeUtil()) {
  }

  explicit MockAsyncController(const TimeUtil &itimeUtil)
    : AsyncPosIntegratedController(std::make_shared<MockMotor>(), itimeUtil) {
  }

  void waitUntilSettled() override;

  void setTarget(double itarget) override;

  double getError() const override;

  bool isSettled() override;

  void reset() override;

  void flipDisable() override;

  void flipDisable(bool iisDisabled) override;

  bool isDisabled() const override;

  double output{0};
  QTime sampleTime = 10_ms;
  double maxOutput{1};
  double minOutput{-1};
  double target{0};
  bool disabled{false};
  bool isSettledOverride{true};
};

class MockIterativeController : public IterativePosPIDController {
  public:
  MockIterativeController();

  double step(double inewReading) override;

  void setTarget(double itarget) override;

  double getOutput() const override;

  double getError() const override;

  bool isSettled() override;

  void setGains(double ikP, double ikI, double ikD, double ikBias) override;

  void setSampleTime(QTime isampleTime) override;

  void setOutputLimits(double imax, double imin) override;

  void setIntegralLimits(double imax, double imin) override;

  void setErrorSumLimits(double imax, double imin) override;

  void reset() override;

  void setIntegratorReset(bool iresetOnZero) override;

  void flipDisable() override;

  void flipDisable(bool iisDisabled) override;

  bool isDisabled() const override;

  QTime getSampleTime() const override;

  double output{0};
  QTime sampleTime = 10_ms;
  double maxOutput{1};
  double minOutput{-1};
  double target{0};
  bool disabled{false};
  bool isSettledOverride{true};
};

class MockSettledUtil : public SettledUtil {
  public:
  MockSettledUtil() : SettledUtil(std::make_unique<MockTimer>()) {
  }

  bool isSettled(double) override {
    return isSettledOverride;
  }

  bool isSettledOverride{true};
};

void assertMotorsHaveBeenStopped(MockMotor *leftMotor, MockMotor *rightMotor);
} // namespace okapi

#endif
