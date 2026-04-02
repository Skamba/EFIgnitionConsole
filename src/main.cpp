#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <string.h>

#include "custom_chars.h"
#include "display_functions.h"
#include "speeduino_data.h"

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
SoftwareSerial mySerial2(10, 11);  // RX, TX

// Module for reading Speeduino's serial3 port and displaying it on a 20x4 LCD.
//
// Lex Sewuster (aka Zeiberstein)
// 20200918 | initial creation
// 20201021 | added moving bars
// 20210103 | fixed temperature reading
// 2024xxxx | updated for enhanced data ("n" command), added fuel pressure
//
// See https://speeduino.com/wiki/index.php/Secondary_Serial_IO_interface

const byte HEADER_SIZE = 3;
const byte MESSAGE_SIZE = 125;
const byte EXPECTED_PAYLOAD_LENGTH = 119;

byte incomingByte;
unsigned long start;
int numberOfBytesRead;
byte message[MESSAGE_SIZE];
unsigned long timeConsumedByReadingAndDisplaying;
char numBuf[21];

void setup() {
  lcd.begin(NUM_DISPLAY_COLS, NUM_DISPLAY_ROWS);

  lcd.createChar(C10, CHAR10);
  lcd.createChar(C01, CHAR01);
  lcd.createChar(C11, CHAR11);
  lcd.createChar(C20, CHAR20);
  lcd.createChar(C02, CHAR02);
  lcd.createChar(C21, CHAR21);
  lcd.createChar(C12, CHAR12);
  lcd.createChar(C22, CHAR22);

  lcd.setCursor(0, 0);
  lcd.print("Inspuiting wordt");
  lcd.setCursor(0, 1);
  lcd.print("    op druk gebracht");
  delay(3000);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Lomax is klaar");
  delay(500);

  lcd.backlight();
  lcd.clear();
  mySerial2.begin(115200);
}

void loop() {
  // Output an "n" to Speeduino for the enhanced real-time data block.
  numberOfBytesRead = 0;
  memset(message, 0, sizeof(message));
  mySerial2.print("n");

  // Stop reading when the payload is complete or when the timeout expires.
  start = millis();
  while ((millis() - start) < WAITING_INTERVAL && numberOfBytesRead < HEADER_SIZE + EXPECTED_PAYLOAD_LENGTH) {
    while (mySerial2.available() != 0 && numberOfBytesRead < HEADER_SIZE + EXPECTED_PAYLOAD_LENGTH) {
      incomingByte = mySerial2.read();

      if (numberOfBytesRead >= HEADER_SIZE) {
        const int messageIndex = numberOfBytesRead - HEADER_SIZE;
        if (messageIndex < MESSAGE_SIZE) {
          message[messageIndex] = incomingByte;
        }
      }

      if (numberOfBytesRead < HEADER_SIZE + MESSAGE_SIZE) {
        numberOfBytesRead++;
      }
    }
  }

  lcdprint(0, 0, message[RPM_HB] * 255 + message[RPM_LB], "%4drpm ");
  lcdprint(8, 0, message[ADVANCE_ANGLE], ADVANCE_FORMAT);
  lcdprint(12, 0, ((int)message[COOLANT_PLUS_OFFSET]) - TEMPERATURE_OFFSET, "%3dC");
  lcdprint(16, 0, ((int)message[IAT_PLUS_OFFSET]) - TEMPERATURE_OFFSET, "%3dC");

  lcdprint(0, 1, message[MAP_HB] * 255 + message[MAP_LB], "%4dkPa ");
  lcdprint(8, 1, message[IDLE_LOAD], "%3d ");
  lcdprint(12, 1, message[TPS], "%3d ");
  lcdprint(16, 1, message[FUEL_PRESSURE] / 100, "%3dbar");

  lcdprint(0, 2, message[OXIGEN] / 10, "%2d.");
  lcdprint(3, 2, message[OXIGEN] % 10, "%01dafr  ");
  lcdprint(14, 2, engineStatus(message[ENGINE]));

  lcd.setCursor(0, 3);
  lcdBar(
    float(message[RPM_HB] * 255 + message[RPM_LB]),
    0.0,
    7000.0,
    float(message[OXIGEN]) / 10.0,
    10.0,
    20.0,
    NUM_DISPLAY_COLS
  );

  timeConsumedByReadingAndDisplaying = millis() - start;
  if (timeConsumedByReadingAndDisplaying < POLLING_INTERVAL) {
    delay(POLLING_INTERVAL - timeConsumedByReadingAndDisplaying);
  }
}
