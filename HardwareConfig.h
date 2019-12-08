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
// - opt. 820ohm external resistor (see Preliminaries)
//
// Preliminaries:
// Measure the resistance of R_IS and L_IS against ground and write it down.
// According to the datasheet there should be a 1Kohm resistor (marked as 102)
// on each IS pin of the IBT_2 against ground. Since we connect both pins on
// the breadboard, the resistors are now in parallel with a resulting resistance
// of 500ohm. Now you are fine and do not need an external resistor.
//
// However: my modules all had a 10Kohm resistor (103) on these pins which resulted
// is a resistance of 5Kohm, if both pins are connected on a breadboard.
// If the IBT_2 is signaling a fault condition, there can be up to 7mA on each of
// the x_IS pins. Makeing it up to 14mA! This will exceed the allowed voltage of 5V 
// on the analog pins of an Arduino.
//    U = Reff * Imax = 5000ohm * 14mA = 70V!
// If this is the case, we need another resistor in parallel, to keep the
// voltage below 5V.
//    Reff = Umax / Imax = 5V / 14mA = 357ohm
// The external resistor can be calculated with:
//    Rext = 1 / (1/Reff - 1/5Kohm) = 384ohm
// With a 390ohm resistor from the E12-series in parallel, the effective
// resistance will be:
//    Reff = 1 / (1/5Kohm + 1/390ohm) = 362ohm
// In fault condition, there will be a voltage of:
//    Umax = Reff * Imax = 362ohm * 14mA = 5,07V
// which is ok, because it is only the worst-case scenario.
//
// Setup:
// - Connect R_PWM to the breadboard and from there make one connection to a
//   PWM pin of the Arduino (i.e. pin 10) and another connection to an input
//   of the OR-Gate of the 74LS32 (i.e. pin 1).
// - Connect L_PWM to the breadboard and from there make one connection to a
//   PWM pin of the Arduino (i.e. pin 11) and another connection to another
//   input of the OR-Gate (i.e. pin 2).
// - Connect the output of the OR-Gate (i.e. pin 3) to the positive input of
//   the Analog Comparator on pin 6.
// - Connect 3.3V to the negative input of the Analog Comparator on pin 7.
// - On the 74LS32 connect pin 7 to Ground and pin 14 to 5V.
// - Connect R_EN and L_EN to the same row of the breadboard and from there
//   to a digital pin of the Arduino (i.e. pin 12)
// - Connect R_IS and L_IS to the same row of the breadboard and from there
//   an 820ohm resistor to the ground column of the breadboard. Add a wire
//   from the above breadboard row to an analog pin A0 of the Arduino.
// - Define all pins and measure the effective resistance of both IS lines 
//   in parallel with the external resistor against ground. Note the 
//   measured value in the #define statement below.
//
#ifndef HARDWARECONFIG_H
#define HARDWARECONFIG_H

// Pins 6 and 7 are used for the Analog Comparator
// Pin A0 is used for Analog-to-Digital Conversion
#define ENCODER_PIN1  8
#define ENCODER_PIN2  9
#define RPWM_PIN      10    // 490Hz PWM
#define LPWM_PIN      11    // 490Hz PWM
#define EN_PIN        12

#define R_EXTERNAL    390 // ohm
#define R_INTERNAL  10000 // ohm

#endif /* HARDWARECONFIG_H */
