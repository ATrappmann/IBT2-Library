// NAME: IBT2Servo.cpp
//
// DESC: Motor controlled with IBT_2 motor driver to act as a servo.
//
// This code uses the Encoder library from Paul Stoffregen (https://github.com/PaulStoffregen/Encoder)
//
// This file is part of the IBT2-Library for the Arduino environment.
//
// MIT License
//
// Copyright (c) 2019 Andreas Trappmann
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#include "IBT2Servo.h"

#define DEBUG 1
#include "Debug.h"

IBT2Servo::IBT2Servo(const uint8_t EncoderPin1, const uint8_t EncoderPin2,
                     const uint8_t RightPwmPin, const uint8_t LeftPwmPin, const uint8_t EnablePin,
                     const uint16_t Rexternal, const uint16_t Rinternal)
          :IBT2(RightPwmPin, LeftPwmPin, EnablePin, Rexternal, Rinternal)
{
  positionEncoder = new Encoder(EncoderPin1, EncoderPin2);
  holdingCurrent = 0.0;
  currentPosition = 0L;
  Kp = 1.0;
  Ki = 0.01;
  Kd = 1.0;
  positionThreshold = 5;
}

bool IBT2Servo::moveToPosition(const long newPosition) {
  while (true) {
    long currentPosition = positionEncoder->read();
    if (abs(newPosition - currentPosition) < positionThreshold) { // position reached
      break;
    }

    double pid = calcPID(currentPosition, newPosition);

    int speed = 0;
    if (pid < 0) {
      speed = constrain(pid, -255, 0);
    }
    else {
      speed = constrain(pid, 0, 255);
    }
    enable();
    setSpeed(speed);

    double current = readCurrent();
    if (current < 0) { // overload
      disable();
      return false;
    }
    if ((holdingCurrent > 0.0) && (current >= holdingCurrent)) { // release servo, if holdingCurrent is reached
      disable();
    }
  }
  return true;
}

void IBT2Servo::setPID(const double Kp, const double Ki, const double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

double IBT2Servo::calcPID(const double currentPosition, const double targetPosition) {
  static double lastSetPoint = 0;
  static unsigned long lastTime = 0;
  static double lastError = 0;
  static double lastI = 0;

  // do we have a new setPoint?
  if (targetPosition != lastSetPoint) {
    lastSetPoint = targetPosition;
    lastTime = millis();
    lastError = 0;
    lastI = 0;
  }

  // calc time delta
  unsigned long now = millis();
  unsigned long dt = now - lastTime;
  lastTime = now;

  double error = targetPosition - currentPosition;
  double integral = lastI + error * dt;
  double derivate = (lastError - error) / dt;

  // calc proportional part: "Get to setPoint!"
  // react to distance
  double pidP = Kp * error;

  // calc integral part: "Korrigiere den Fehler den der P-Regler über die Zeit nicht weg bekommt!"
  double pidI = Ki * integral;
  //if (Ki > 0) pidI = lastI + Ki * error;
  lastI = pidI;

  // calc differential part: "Stop at setPoint!"
  // v = dx/dt
  // react to speed
  double pidD = Kd * derivate;
  //if (dt > 0) pidD = Kd * (error - lastError) / dt;
  lastError = error;

  SEROUT("Error=" << error << ", P=" << pidP << ", I=" << pidI << ", D=" << pidD << LF);

  // return sum
  return pidP + pidI + pidD;
}
