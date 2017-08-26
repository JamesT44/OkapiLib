---
weight: 210
title: Pid
---

# Pid

The `Pid` class implements the Pid algorithm, with some quality-of-life changes to support online tuning.

## Constructor

```c++
//Signature
Pid(const float ikP, const float ikI, const float ikD, const float ikBias = 0)
Pid(const PidParams& params)
```

Parameter | Description
----------|------------
ikP | Proportional gain
ikI | Integral gain
ikD | Derivative gain
ikBias | Controller bias (this value added to output, default 0)
params | `PidParams`

## loop

```c++
//Signature
virtual float loop(const float inewReading)
```

Do one iteration of Pid math to compute a new motor power. This needs to be called every so many milliseconds (15 ms works fine).

Parameter | Description
----------|------------
inewReading | New sensor reading

## setGains

```c++
//Signature
void setGains(const float ikP, const float ikI, const float ikD, const float ikBias = 0)
```

Set new controller gains and bias.

Parameter | Description
----------|------------
ikP | Proportional gain
ikI | Integral gain
ikD | Derivative gain
ikBias | Controller bias (this value added to output, default 0)

## setSampleTime

```c++
//Signature
void setSampleTime(const int isampleTime)
```

Set the timestep (in ms) between calls to `loop`.

Parameter | Description
----------|------------
isampleTime | Timestep between calls to `loop` in ms

## setOutputLimits

```c++
//Signature
void setOutputLimits(float imax, float imin)
```

Set the max and min value for the controller output.

Parameter | Description
----------|------------
imax | Max output
imin | Min output

## setIntegralLimits

```c++
//Signature
void setIntegralLimits(float imax, float imin)
```

Set the max and min value for the integrator sum.

Parameter | Description
----------|------------
imax | Max integrator value
imin | Min integrator value

## reset

```c++
//Signature
void reset()
```

Reset the controller so it will start from zero again.

## setIntegratorReset

```c++
//Signature
void setIntegratorReset(bool iresetOnZero)
```

Set whether the integrator should be cleared when the controller's error is zero or changes sign.


Parameter | Description
----------|------------
iresetOnZero | Whether the integrator should be cleared when the controller's error is zero or changes sign

## setTarget

```c++
//Signature
void setTarget(const float itarget)
```

Set the target value.

Parameter | Description
----------|------------
itarget | New target value

## getOutput

```c++
//Signature
float getOutput() const
```

Return the most recent controller output.

# PidParams

The `PidParams` class encapsulates the parameters a `Pid` takes.

Parameter | Description
----------|------------
kP | Proportional gain
kI | Integral gain
kD | Derivative gain
kBias | Controller bias (this value added to output)

## Constructor

```c++
//Signature
PidParams(const float ikP, const float ikI, const float ikD, const float ikBias = 0)
```

Parameter | Description
----------|------------
ikP | Proportional gain
ikI | Integral gain
ikD | Derivative gain
ikBias | Controller bias (this value added to output, default 0)