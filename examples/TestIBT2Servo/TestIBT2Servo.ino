// NAME: TestIBT2Servo.ino
//
// DESC: Basic test of IBT_2 module with current sensing
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
#include "HardwareConfig.h"

IBT2Servo servo(ENCODER_PIN1, ENCODER_PIN2, RPWM_PIN, LPWM_PIN, EN_PIN, R_EXTERNAL, R_INTERNAL);

void setup() {
  Serial.begin(115200);
  Serial.println("TestIBT2Servo uploaded " __DATE__ " " __TIME__);

  servo.setPID(0.85, 0.005, 1.0);
  servo.setPositionThreshold(5);
  servo.setHoldingCurrent(1.5);
}

void loop() {
  Serial.print("Current position: "); Serial.print(servo.getPosition());
  Serial.print(", Move to position: "); Serial.println(500);
  servo.moveToPosition(500);
  delay(1000);

  Serial.print("Current position: "); Serial.print(servo.getPosition());
  Serial.print(", Move to position: "); Serial.println(500);
  servo.moveToPosition(1000);
  delay(1000);

  Serial.print("Current position: "); Serial.print(servo.getPosition());
  Serial.print(", Move to position: "); Serial.println(500);
  servo.moveToPosition(500);
  delay(1000);

  Serial.print("Current position: "); Serial.print(servo.getPosition());
  Serial.print(", Move to position: "); Serial.println(500);
  servo.moveToPosition(0);
  delay(1000);

  Serial.print("Current position: "); Serial.print(servo.getPosition());
  Serial.print(", Move to position: "); Serial.println(500);
  servo.moveToPosition(-500);
  delay(1000);

  Serial.print("Current position: "); Serial.print(servo.getPosition());
  Serial.print(", Move to position: "); Serial.println(500);
  servo.moveToPosition(-1000);
  delay(1000);

  Serial.print("Current position: "); Serial.print(servo.getPosition());
  Serial.print(", Move to position: "); Serial.println(500);
  servo.moveToPosition(-500);
  delay(1000);

  Serial.print("Current position: "); Serial.print(servo.getPosition());
  Serial.print(", Move to position: "); Serial.println(500);
  servo.moveToPosition(0);
  delay(1000);
}
