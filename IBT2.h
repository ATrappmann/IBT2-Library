// NAME: IBT2.h
//
// DESC: IBT2 motor driver support
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
#ifndef IBT2_H
#define IBT2_H

#include <Arduino.h>

class IBT2 {
private:
  uint8_t RPWM_PIN, LPWM_PIN;
  uint8_t EN_PIN;

  uint8_t RPWM_FB_PIN, LPWM_FB_PIN;
  uint8_t CS_PIN;
  uint8_t effectiveRIS;
  int absMaxAnalogSenseValue;

  int speed;
  uint8_t pwmFreq;

public:
  IBT2(const uint8_t RPWM_PIN, const uint8_t LPWM_PIN, const uint8_t EN_PIN);

  void enable() const;
  void disable() const;

  bool setSpeed(const int speed);   // -255 to +255
  void stop();

  void initCurrentSensing(const uint8_t RPWM_FB_PIN, const uint8_t LPWM_FB_PIN, const uint8_t CS_PIN, const uint8_t effRis);
  double readCurrent() const;
};

#endif /* IBT2_H */
