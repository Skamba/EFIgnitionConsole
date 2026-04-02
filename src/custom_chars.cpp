#include "custom_chars.h"

byte CHAR10[8] = {
  B11100,
  B11100,
  B11100,
  B11100,
  B00000,
  B00000,
  B00000,
  B00000
};

byte CHAR01[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11100,
  B11100,
  B11100,
  B11100
};

byte CHAR11[8] = {
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100
};

byte CHAR20[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000
};

byte CHAR02[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111
};

byte CHAR21[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11100,
  B11100,
  B11100,
  B11100
};

byte CHAR12[8] = {
  B11100,
  B11100,
  B11100,
  B11100,
  B11111,
  B11111,
  B11111,
  B11111
};

byte CHAR22[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

const int NUM_DISPLAY_COLS = 20;
const int NUM_DISPLAY_ROWS = 4;

const byte SPACE = 32;
const byte C10 = 0;
const byte C01 = 1;
const byte C11 = 2;
const byte C20 = 3;
const byte C02 = 4;
const byte C21 = 5;
const byte C12 = 6;
const byte C22 = 7;

byte PATTERN[3][3] = {
  {SPACE, C01, C02},
  {C10, C11, C12},
  {C20, C21, C22}
};
