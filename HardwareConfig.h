// NAME: HardwareConfig.h
//
// DESC: This file contains the hardware setup for the included sketches
//       used as examples and for testing.
//
// This file is part of the IBT2-Library for the Arduino environment.
//
// Required Hardware:
// - IBT_2 Module
// - DC motor with Encoder
// - Arduino (Uno)
// - Breadboard & set of Dupont female-male and male-male wires
// - Ohmmeter
// - 820ohm Resistor (see Preliminaries)
//
// Preliminaries:
// Measure the resistance of R_IS and L_IS against ground and write it down.
// According to the datasheet there should be a 1Kohm resistor (marked as 102)
// on each IS pin of the IBT_2 against ground. Since we connect both pins on
// the breadboard, the resistors are now in parallel with a resulting resistance
// of 500ohm. Now you are fine.
//
// My modules all had a 10Kohm resistor (103) on these pins which resulted in
// a resistance of 5Kohm, if both pins are connected on a breadboard.
// If the IBT_2 is signaling a fault condition, there will be 7mA on the
// dedicated IS pin. This will exceed the allowed voltage on an analog pin on
// the Arduino.
//    U = Reff * Imax = 5000ohm * 7mA = 35V!
//
// If this is the case, we need another resistor in parallel, to keep the
// voltage below 5V.
//    Reff = U / Imax = 5V / 7mA = 714ohm
//    Rext = 1 / (1/Reff - 1/5Kohm) = 832ohm
//
// If we use a 820ohm resistor from the E12-series in parallel, the effective
// resistance will be:
//    Reff = 1 / (1/5Kohm + 1/820ohm) = 704ohm
// In fault condition, there will be a voltage of:
//    Umax = Reff * Imax = 704ohm * 7mA = 4,9V
// which is fine.
//
// Setup:
// - Connect R_PWM to the breadboard and from there make one connection to a
//   PWM pin of the Arduino (i.e. pin 6) and another connection to a digital
//   pin of the Arduino (i.e. pin 2).
// - Connect L_PWM to the breadboard and from there make one connection to a
//   PWM pin of the Arduino (i.e. pin 7) and another connection to a digital
//   pin of the Arduino (i.e. pin 3).
// - Connect R_EN and L_EN to the same row of the breadboard and from there
//   to a digital pin of the Arduino (i.e. pin 4)
// - Connect R_IS and L_IS to the same row of the breadboard and from there
//   an 820ohm resistor to the ground column of the breadboard. Add a wire
//   from the above breadboard row to an analog pin of the Arduino (i.e. pin A0)
// - Define all pins and the effective resistance of both IS lines in parallel
//   in the #define statements below.
//
#ifndef HARDWARECONFIG_H
#define HARDWARECONFIG_H

#define RPWM_PIN    6
#define LPWM_PIN    7
#define RPWM_FB_PIN 2
#define LPWM_FB_PIN 3
#define EN_PIN      4
#define CS_PIN      A0

#define ENCODER_PIN1  8
#define ENCODER_PIN2  9

#define EFFECTIVE_R_IS  704 // ohm

#endif /* HARDWARECONFIG_H */
