#ifndef CUSTOM_CHARS_H
#define CUSTOM_CHARS_H

#include <Arduino.h>

// Character definitions (only 8 possible with Hitachi HD44780) for the moving
// bar emulation.
extern byte CHAR10[8];
extern byte CHAR01[8];
extern byte CHAR11[8];
extern byte CHAR20[8];
extern byte CHAR02[8];
extern byte CHAR21[8];
extern byte CHAR12[8];
extern byte CHAR22[8];

extern const int NUM_DISPLAY_COLS;
extern const int NUM_DISPLAY_ROWS;

extern const byte SPACE;
extern const byte C10;
extern const byte C01;
extern const byte C11;
extern const byte C20;
extern const byte C02;
extern const byte C21;
extern const byte C12;
extern const byte C22;

extern byte PATTERN[3][3];

#endif
