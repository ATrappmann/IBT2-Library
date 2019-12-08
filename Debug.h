// NAME: Debug.h
//
// DESC: Helper methods for printing debugging information through the Serial interface.
//       Printing must be enabled by a "#define DEBUG 1" before including this header.
//
// This file is part of the IBT2-Library for the Arduino environment.
//
#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
inline Print& operator <<(Print &obj, int arg) { obj.print(arg); return obj; }
inline Print& operator <<(Print &obj, double arg) { obj.print(arg, 3); return obj; }

#ifdef DEBUG
#define LF  '\n'
#define SEROUT(msg)  Serial << msg
#else
#define SEROUT(msg)
#endif

#endif /* DEBUG_H */
