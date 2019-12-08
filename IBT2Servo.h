// NAME: IBT2Servo.h
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
#ifndef IBT2SERVO_H
#define IBT2SERVO_H

#include "IBT2.h"
#include <Encoder.h>

class IBT2Servo : public IBT2 {
private:
  Encoder *positionEncoder;
  double holdingCurrent;
  long currentPosition;
  double Kp, Ki, Kd;
  long positionThreshold;

public:
  IBT2Servo(const uint8_t EncoderPin1, const uint8_t EncoderPin2,
            const uint8_t RightPwmPin, const uint8_t LeftPwmPin, const uint8_t EnablePin,
            const uint8_t Ris, const uint8_t EffectiveCurrentSenseResistor = 1000);

  long getPosition() const { return currentPosition; }
  bool moveToPosition(const long newPosition); // blocking until position is reached

  void setPID(const double Kp, const double Ki, const double Kd);
  void setPositionThreshold(const long threshold) { this->positionThreshold = threshold; }
  void setHoldingCurrent(const double current) { this->holdingCurrent = current; }

private:
  double calcPID(const double currentPosition, const double targetPosition);

};

#endif /* IBT2SERVO_H */
