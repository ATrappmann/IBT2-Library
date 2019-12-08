// NAME: IBT2.h
//
// DESC: Library for IBT_2 modules with high current BTS7960 half bridge motor drivers.
//       The library uses PWM and supports current sensing.
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
public:
  uint8_t RPWM_PIN, LPWM_PIN;
  uint8_t EN_PIN;

  uint16_t Rintern;
  uint16_t Rextern;
  
  uint16_t absMaxAnalogSenseValue;
  
  int speed;
  uint8_t pwmFreq;

public:
  IBT2(const uint8_t RPWM_pin, const uint8_t LPWM_pin, const uint8_t EN_pin, 
       const uint16_t Rexternal, const uint16_t Rinternal = 10000);

  void enable() const;
  void disable() const;

  bool setSpeed(const int speed);   // -255 to +255
  void stop();

  void initCurrentSensing();
  double readCurrent() const;
};

#endif /* IBT2_H */
