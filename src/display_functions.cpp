#include "display_functions.h"

#include <math.h>

#include "custom_chars.h"
#include "speeduino_data.h"

extern char numBuf[21];

void lcdprint(byte col, byte row, int num, const char *fmt) {
  lcd.setCursor(col, row);
  sprintf(numBuf, fmt, num);
  lcd.print(numBuf);
}

void lcdprint(byte col, byte row, const char *text) {
  lcd.setCursor(col, row);
  lcd.print(text);
}

char *engineStatus(byte status) {
  static char buf[7] = {' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  int bufPos = 5;

  // Display the most stable parameters to the right.
  for (int bitNr = BIT_ENGINE_RUN; bitNr <= BIT_ENGINE_MAPDCC; bitNr++) {
    if (status & (1 << bitNr)) {
      buf[bufPos] = ENGINE_STATUS_CHAR[bitNr];
      bufPos--;
    }
  }

  for (int i = bufPos; i >= 0; i--) {
    buf[i] = ' ';
  }

  return buf;
}

int activeCells(float value, float minValue, float maxValue, int num_cells) {
  float interval = (maxValue - minValue) / (num_cells + 1);
  int nrActiveCells = floor((value - minValue) / interval);

  if (nrActiveCells < 0) {
    nrActiveCells = 0;
  }

  if (nrActiveCells > num_cells) {
    nrActiveCells = num_cells;
  }

  return nrActiveCells;
}

void lcdBar(float topValue, float topMinValue, float topMaxValue, float bottomValue, float bottomMinValue, float bottomMaxValue, int num_cols) {
  int num_cells = 2 * num_cols;
  int nrActiveTopCells = activeCells(topValue, topMinValue, topMaxValue, num_cells);
  int nrActiveBottomCells = activeCells(bottomValue, bottomMinValue, bottomMaxValue, num_cells);
  lcdRawBar(nrActiveTopCells, nrActiveBottomCells, num_cells);
}

int numActiveCells(int pointer, int value) {
  if (pointer < value) {
    return 2;
  }

  if (pointer == value) {
    return 1;
  }

  return 0;
}

void lcdRawBar(int p, int q, int num_cells) {
  int numTop;
  int numBottom;

  for (int i = 1; i < num_cells; i = i + 2) {
    numTop = numActiveCells(i, p);
    numBottom = numActiveCells(i, q);
    lcd.write(byte(PATTERN[numTop][numBottom]));
  }
}
