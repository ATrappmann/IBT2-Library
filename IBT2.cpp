// NAME: IBT2.cpp
//
// DESC: Library for IBT_2 modules with high current BTS7960 half bridge motor drivers.
//       The library uses PWM and supports current sensing.
//
// To uses current sensing with this libray an external resistor and a 74LS32 OR-Gate 
// is required.
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
#include "IBT2.h"

#define DEBUG 1
#include "Debug.h"

IBT2::IBT2(const uint8_t RPWM_pin, const uint8_t LPWM_pin, const uint8_t EN_pin, const uint16_t R_ext, const uint16_t R_int) {
#ifdef DEBUG
  if (!digitalPinHasPWM(RPWM_pin)) SEROUT("Pin " << RPWM_pin << " for R_PWM is NOT a PWM pin!" << LF);
  if (!digitalPinHasPWM(LPWM_pin)) SEROUT("Pin " << LPWM_pin << " for L_PWM is NOT a PWM pin!" << LF);
#endif

  this->RPWM_PIN = RPWM_pin;
  this->LPWM_PIN = LPWM_pin;
  this->EN_PIN = EN_pin;

  this->Rextern = R_ext;
  this->Rintern = R_int;

  // Umax = Ris * ISmax; ISmax = 43A/8500; maxAnalogValue = Umax * 1023/5V
  // Umax = 1/(1/Rextern + 2/Rintern) * 43A/8500
  this->absMaxAnalogSenseValue = 1.0 / (1.0/double(Rextern) + 2.0/double(Rintern)) * 1.035;
  
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(RPWM_PIN, LOW);
  digitalWrite(LPWM_PIN, LOW);
  digitalWrite(EN_PIN, LOW);

  pwmFreq = 0;
  initCurrentSensing();
}

void IBT2::enable() const {
  digitalWrite(EN_PIN, HIGH);
  stop();
}

void IBT2::disable() const {
  digitalWrite(EN_PIN, LOW);
  stop();
}

bool IBT2::setSpeed(const int speed) {
  if ((speed < -255) || (speed > 255)) {
    SEROUT("**** ERROR: Speed " << speed << " is out of range!" << LF);
    return false;
  }

  if (speed < 0) {
    pwmFreq = -speed;
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, pwmFreq);
  }
  else {
    pwmFreq = speed;
    analogWrite(LPWM_PIN, 0);
    analogWrite(RPWM_PIN, pwmFreq);
  }
}

void IBT2::stop() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  pwmFreq = 0;
}

// The BTS7960 Half Bridge Drivers have a current sense ratio of kILIS = IL / Iis = 8500.
// The external resistor Ris determines the voltage per output current. On the IBT2 there
// should be a 1kohm resistor for Ris. On my boards though, there are 10kohm resistors!
//    Iis = Vis / Ris. 
// Vis has to be measured on an analog port. 
// Ie. Vis=0,5V with Ris=10Kohm: Iis = 0,5V/10Kohm = 50uA; IL = 8500*Iis = 8500*50uA = 0,425A.
// Ie. Vis=0,05V with Ris=1Kohm: Iis = 0,05V/1Kohm = 50uA; IL = 8500*Iis = 8500*50uA = 0,425A.
//
// If there is an error, the BTS7960 provides a constant current which is independent of 
// the load current and provides Iis(typ) = 4.5mA, but up to Iis(lim)max = 7mA to signal 
// the fault condition.
// 
// The challenge is, that Iis can only be measured, when the corresponding PWM pin is high 
// (see section 4.4.5 "Truth Table" of the BTS7960 datasheet).
// 
// The smallest impulse of a 490Hz PWM signal is 1/255 = 1/490Hz * 1/255 = 8us. The time period
// is 1/490Hz = 2ms.
//                    |----|                                     |----|
//        PWM:    ____|~8us|____        .... 2ms ....        ____|~8us|____
//
// To know, when the PWM pulse is there, we use the Analog Comparator for the Arduino and feed
// its positive input with the PWM signal, we send to the IBT2 module. There are 2 PWM input
// pins on the IBT2. The usage is either the R_PWM or the L_PWM, depending on the direction. 
// Since there is only 1 Analog Comparator, we have to combine these PWM signals for the 
// Comparator by using a 74LS32 OR-Gate. For the negative input of the Comparator, we use the
// 3.3V output of the Arduino. Now we get a signal, which is in-sync with the PWM signals.
//
//                    |----|                                     |----|
//        ACO:    ____|    |____        .... 2ms ....        ____|~8us|____
//
// The output signal from the Analog Comparator can be configured to trigger an Interrupt,
// which can be used to start the Analog Digital Converter in the Arduino.
//
//                    ||
//        ACI:    ____||________
//
// After triggered by the Analog Comparators interrupt, the ADC needs 5 ADC clock cycles 
// for its Sample-and-Hold sequence. During this time, we have to assure, that the Iis 
// current is there to be measured. Meaning that 5 ADC clock cycles are less than 8us.
// This can be done, by configuring the ADC prescaler with 16, which gives an ADC clock
// cycle of 16MHz (Arduino Oszillator) / 16 = 1MHz. One ADC clock cycle is now 1us.
//
//                     |--| 
//        AD:     _____|  |_____ (5us für Sample-and-Hold)
//
// If the AD conversion is done, we use the AD interrupt to read the measured value.
//
//                        |-----
//        ADIF:   ________|
//

// Interrupt Service Routine (ISR) for the vector Analog Comparator Complete.
// Nothing to be done, but has do be there to avoid a call to the reset vector!
//
ISR(ANALOG_COMP_vect) { /* NOP */ }

// Interrupt Service Routine (ISR) for ADC Conversion Complete
//
volatile bool adcDone;
volatile byte adcValueHi, adcValueLo;
ISR(ADC_vect) { 
  adcValueLo = ADCL;
  adcValueHi = ADCH;  // read 8 bit value from ADC
  adcDone = true;
}

double IBT2::readCurrent() const {
  double value;
  
  if (0 == pwmFreq) {   // if no PWM pulse, we do not get a trigger signal from the Analog Comparator
    value = 0;
  }
  else if (pwmFreq < 255) {   // regular PWM signal which generates a rising edge for the Analog Comparator
    value = 0;
    for (int i=1; i<10; i++) {
      adcDone = false;
      while (!adcDone);   // wait
      value += (adcValueHi << 8) | adcValueLo; 
    }
    value /= 10;
  }   
  else {    // with the max. PWM output of 255, we have a constant HIGH signal. Since there is no
            // rising edge, we have to trigger the ADC manually.
    value = 0;
    for (int i=0; i<10; i++) {
      adcDone = false;
      ADCSRA |= (1 << ADSC);  // set AD Start Conversion
      while (!adcDone);   // wait    
      value += (adcValueHi << 8) | adcValueLo;
    }
    value /= 10;
  }

  double Vis = double(value) * double(5000.0/1023.0);     // mV
  double Iint = Vis / double(Rintern/2);                  // mA
  double Iext = Vis / double(Rextern);                    // mA
  double Iis = Iint + Iext;                               // mA
  double Iil = 8.5 * Iis;                                 // A

  SEROUT("PWM=" << pwmFreq << ", Value=" << value << ", Vis=" << Vis << "mV, Iext=" << Iext << "mA, Iis=" << Iis << "mA");

  if (value > absMaxAnalogSenseValue) {
    SEROUT(" - OVERLOAD!");
    disable();
    return -1;
  }

  return Iil;
}

void IBT2::initCurrentSensing() {
  // setup input pins for Analog Comparator as inputs
  // 
  pinMode(6, INPUT);   // AIN0_PIN
  pinMode(7, INPUT);   // AIN1_PIN
  
  // setup Analog Comparator
  //
  DIDR1 = DIDR1 | (1 << AIN1D) | (1 << AIN0D);    // disable digital input buffers
  
  byte maskACSR = 0;
  maskACSR |= (1 << ACD);     // set bit for Analog Comparator Disable
  maskACSR |= (1 << ACI);     // clear interrupt flag by writing 1
  maskACSR |= (1 << ACIE);    // enable interrupts
  maskACSR |= (1 << ACIS1);   // IRQ on rising edge of output
  maskACSR |= (1 << ACIS0);   // IRQ on rising edge of output
  ACSR = maskACSR;
  
  // setup Analog-to-Digital Converter
  //
  byte maskADMUX = 0;
  maskADMUX &= B11110000;     // clear MUX[3:0] to set A0 as analog input pin
  maskADMUX |= (1 << REFS0);  // set reference voltage to AVcc
  //maskADMUX |= (1 << ADLAR);  // left align ADC value to 8 bits from ADCH register
  ADMUX = maskADMUX;

  byte maskADCSRB = 0;
  maskADCSRB &= ~(1 << ACME); // disable Analog Comparator Multiplexer
  maskADCSRB |= (1 << ADTS0); // set Analog Comparator as trigger source
  ADCSRB = maskADCSRB;
  
  byte maskADCSRA = 0;
  maskADCSRA |= (1 << ADEN);  // enable ADC. This takes 12 ADC clock cycles!
  maskADCSRA |= (1 << ADATE); // enable auto trigger
  maskADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  maskADCSRA |= (1 << ADPS2); // 16 prescaler for 76.8KHz
  ADCSRA = maskADCSRA;

  sei();                  // enable global interrupts
  adcDone = false;        // initial ISR flag
  ACSR &= ~(1 << ACD);    // clear bit for AC Disable  
}
