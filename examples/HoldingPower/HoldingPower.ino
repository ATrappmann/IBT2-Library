// NAME: HoldingPower.ino
//
// DESC: 
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
#include <Arduino.h>
#include <Encoder.h>

#include "IBT2.h"
#include "HardwareConfig.h"

/*
 * Define Arduino Analog Reference in mV
 */
#define A_REF   5000  // mV

/*
 * Define ACS712-Module supply voltage in mV
 */
#define ACS_REF   A_REF // from Arduino

/*
 * Define ACS712-Type specific sensitivity in mV/A
 */
//#define ACS_SENS 185   // mV/A, for ACS712-05B
#define ACS_SENS 100   // mV/A, for ACS712-20A
//#define ACS_SENS  66   // mV/A, for ACS712-30A

#define ACS712_PIN  A2

IBT2 m1(RPWM_PIN, LPWM_PIN, EN_PIN);

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder enc(2, 3);
//   avoid using pins with LEDs attached

// für 6V
//const double Kp=1.0, Ki=0.04, Kd=1.0;

// für 12V
const double Kp=0.85, Ki=0.005, Kd=1.0;

#define MIN_SPEED 0
#define SETPOINT 480*5
#define TIMEOUT 5000
#define HOLDING_POWER 1.0 // Amps

double calcPID(const double feedback, const double setPoint) {
  static double lastSetPoint = 0;
  static unsigned long lastTime = 0;
  static double lastError = 0;
  static double lastI = 0;

  // do we have a new setPoint?
  if (setPoint != lastSetPoint) {
    lastSetPoint = setPoint;
    lastTime = millis();
    lastError = 0;
    lastI = 0;
  }

  double error = setPoint - feedback;
  Serial.print(", Error: "); Serial.print(error);

  // calc proportional part: "Get to setPoint!"
  // react to distance
  double pidP = Kp * error;
  Serial.print(", P: "); Serial.print(pidP);

  // calc integral part: "Korrigiere den Fehler den der P-Regler über die Zeit nicht weg bekommt!"
  double pidI = 0;
  if (Ki > 0) pidI = lastI + Ki * error;
  Serial.print(", I: "); Serial.print(pidI);
  lastI = pidI;

  // calc differential part: "Stop at setPoint!"
  // v = dx/dt
  // react to speed
  unsigned long now = millis();
  unsigned long dt = now - lastTime;
  lastTime = now;
  double pidD = 0;
  if (dt > 0) pidD = Kd * (error - lastError) / dt;
  Serial.print(", D: "); Serial.print(pidD);
  lastError = error;

  // calc sum
  double pid = pidP + pidI + pidD;
  return pid;
}

// use ACS712
double readCurrentFromACS() {
  int value = analogRead(ACS712_PIN);
  int mV = map(value, 0, 1023, 0, A_REF); // map input 0-1023 to mV (0-5000)

  double adjustedMV = double(mV) - (ACS_REF/2);  // adjust 0 position
  double amps = adjustedMV / ACS_SENS;
  return amps;
}

void setup() {
  Serial.begin(115200); // Serielle Verbindung starten, damit die Daten am Seriellen Monitor angezeigt werden.
  Serial.println("Running...");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  m1.initCurrentSensing(RPWM_FB_PIN, LPWM_FB_PIN, CS_PIN, EFFECTIVE_R_IS);
  m1.enable();
}

unsigned long iterations = 0;
unsigned long startTime = millis();
double maxCurrent1 = 0;
double maxCurrent2 = 0;

void loop() {
  Serial.print("Setpoint: "); Serial.print(SETPOINT);

  long position = enc.read();
  Serial.print(", Position: "); Serial.print(position);

  double pid = calcPID(position, SETPOINT);
  Serial.print(", PID: "); Serial.print(pid);

  int speed = 0;
  if (pid < 0) {
    speed = constrain(pid, -255, -MIN_SPEED);
  }
  else {
    speed = constrain(pid, MIN_SPEED, 255);
  }
  Serial.print(", Speed: "); Serial.print(speed);
  m1.setSpeed(speed);

  double amps1 = m1.readCurrent();
  Serial.print(", Iibt: "); Serial.print(amps1, 1);
  if (amps1 > maxCurrent1) {
    maxCurrent1 = amps1;
  }
  Serial.print(", Imax1: "); Serial.print(maxCurrent1, 1);

  double amps2 = readCurrentFromACS();
  Serial.print(", Iacs: "); Serial.print(amps2, 3);
  if (amps2 > maxCurrent2) {
    maxCurrent2 = amps2;
  }
  Serial.print(", Imax2: "); Serial.print(maxCurrent2, 1);

  // check Overload
  if (amps1 < 0) {
    m1.disable();
    Serial.println("\n***** Overload shutdown!");
    Serial.flush();
    exit(0);
  }

  // Motor protection
  if (amps1 > HOLDING_POWER) {
    m1.disable();
    Serial.println("Overload!");
    Serial.flush();
  }

  if (millis() - startTime > TIMEOUT) {
    Serial.println("\n***** Timeout!");
    Serial.flush();
    m1.disable();
    digitalWrite(13, LOW);
    exit(0);
  }

  Serial.println();
}
