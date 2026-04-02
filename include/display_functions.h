#ifndef DISPLAY_FUNCTIONS_H
#define DISPLAY_FUNCTIONS_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

extern LiquidCrystal_I2C lcd;

void lcdprint(byte col, byte row, int num, const char *fmt);
void lcdprint(byte col, byte row, const char *text);
char *engineStatus(byte status);
int activeCells(float value, float minValue, float maxValue, int num_cells);
void lcdBar(float topValue, float topMinValue, float topMaxValue, float bottomValue, float bottomMinValue, float bottomMaxValue, int num_cols);
int numActiveCells(int pointer, int value);
void lcdRawBar(int p, int q, int num_cells);

#endif
