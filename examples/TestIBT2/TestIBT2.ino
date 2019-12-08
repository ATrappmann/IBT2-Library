// NAME: TestIBT2.ino
//
// DESC: Basic test of IBT_2 module with current sensing
//
// Copyright (c) 2019 by Andreas Trappmann. All rights reserved.
//
// This file is part of the IBT2-Library for the Arduino environment.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
#include "IBT2.h"
#include "HardwareConfig.h"

IBT2 motor(RPWM_PIN, LPWM_PIN, EN_PIN, R_EXTERNAL, R_INTERNAL);

void setup() {
  Serial.begin(115200);
  Serial.println("IBT2-MotorTest uploaded " __DATE__ " " __TIME__);
  motor.enable();
}

void loop() {
  for (int speed=0; speed<256; speed++) {
    Serial.print("Speed = "); Serial.print(speed); Serial.print(", ");
    motor.setSpeed(speed);

    double current = motor.readCurrent();
    if (current < 0) {
      Serial.println("\n*** ERROR!");
      Serial.flush();
      exit(-1);
    }
    Serial.print(", IL = "); Serial.print(current, 3); Serial.println("A");
    delay(100);
  }
  motor.stop();

  for (int speed=0; speed>-256; speed--) {
    Serial.print("Speed = "); Serial.print(speed); Serial.print(", ");
    motor.setSpeed(speed);

    double current = motor.readCurrent();
    if (current < 0) {
      Serial.println("\n*** ERROR!");
      Serial.flush();
      exit(-1);
    }
    Serial.print(", IL = "); Serial.print(current, 3); Serial.println("A");
    delay(100);
  }
  motor.stop();
  
  delay(1000);
}
