#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <string.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
SoftwareSerial mySerial2(10, 11);  // RX, TX

// Module for reading Speeduino's serial3 port and displaying it on a 20x4 LCD.
//
// Lex Sewuster (aka Zeiberstein)
// 20200918 | initial creation
// 20201021 | added moving bars
// 20210103 | fixed temperature reading
// 20260402 | migrated to enhanced "n" command and added fuel pressure
//
// See https://speeduino.com/wiki/index.php/Secondary_Serial_IO_interface

// Position numbers in Speeduino's real-time data block.
const byte SQUIRT               =   1;
const byte ENGINE               =   2;
const byte DWELL                =   3;
const byte MAP_LB               =   4;
const byte MAP_HB               =   5;
const byte IAT_PLUS_OFFSET      =   6;
const byte COOLANT_PLUS_OFFSET  =   7;
const byte BAT_CORRECTION       =   8;
const byte BATTERY10            =   9;
const byte OXIGEN               =  10;
const byte EGO_CORRECTION       =  11;
const byte IAT_CORRECTION       =  12;
const byte WUE_CORRECTION       =  13;
const byte RPM_LB               =  14;
const byte RPM_HB               =  15;
const byte TAE_AMOUNT           =  16;
const byte CORRECTIONS          =  17;
const byte VE                   =  18;
const byte AFR_TARGET           =  19;
const byte PW1_LB               =  20;
const byte PW1_HB               =  21;
const byte TPS_DOT              =  22;
const byte ADVANCE_ANGLE        =  23;
const byte TPS                  =  24;
const byte LOOPS_PER_SECOND_LB  =  25;
const byte LOOPS_PER_SECOND_HB  =  26;
const byte FREE_RAM_LB          =  27;
const byte FREE_RAM_HB          =  28;
const byte BOOST_TARGET         =  29;
const byte BOOST_DUTY           =  30;
const byte SPARK                =  31;
const byte RPM_DOT_LB           =  32;
const byte RPM_DOT_HB           =  33;
const byte ETHANOL_PCT          =  34;
const byte FLEX_CORRECTION      =  35;
const byte FLEX_IGN_CORRECTION  =  36;
const byte IDLE_LOAD            =  37;
const byte TEST_OUTPUTS         =  38;
const byte OXIGEN2              =  39;
const byte BARO                 =  40;
const byte FUEL_PRESSURE        = 103;
const byte OIL_PRESSURE         = 104;

const byte BIT_ENGINE_RUN       =   0;
const byte BIT_ENGINE_CRANK     =   1;
const byte BIT_ENGINE_ASE       =   2;
const byte BIT_ENGINE_WARMUP    =   3;
const byte BIT_ENGINE_ACC       =   4;
const byte BIT_ENGINE_DCC       =   5;
const byte BIT_ENGINE_MAPACC    =   6;
const byte BIT_ENGINE_MAPDCC    =   7;

const char ENGINE_STATUS_CHAR[] = {'R', 'C', 'A', 'W', 'a', 'd', '<', '>'};
const char ADVANCE_FORMAT[] = {'%', '3', 'd', char(223), ' ', '\0'};

const int TEMPERATURE_OFFSET = 40;
const unsigned long WAITING_INTERVAL = 100UL;
const unsigned long EXTRA_BYTE_WAITING_INTERVAL = 5UL;
const unsigned long POLLING_INTERVAL = 1000UL;

const byte HEADER_SIZE = 3;
const byte PAYLOAD_OFFSET = HEADER_SIZE - 1;
const int MAX_PACKET_SIZE = 300;

const byte PACKET_STATUS_OK                = 0;
const byte PACKET_STATUS_OVERFLOW          = 1;
const byte PACKET_STATUS_HEADER_INCOMPLETE = 2;
const byte PACKET_STATUS_HEADER_INVALID    = 3;
const byte PACKET_STATUS_INCOMPLETE        = 4;
const byte PACKET_STATUS_TOO_LONG          = 5;

namespace {

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

}  // namespace

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

const byte PATTERN[3][3] = {
  {SPACE, C01, C02},
  {C10, C11, C12},
  {C20, C21, C22}
};

byte packet[MAX_PACKET_SIZE];  // More than enough for the maximum payload plus header.
byte packetStatusCode = PACKET_STATUS_OK;
unsigned long timeConsumedByReadingAndDisplaying;
void registerCustomChars() {
  lcd.createChar(C10, CHAR10);
  lcd.createChar(C01, CHAR01);
  lcd.createChar(C11, CHAR11);
  lcd.createChar(C20, CHAR20);
  lcd.createChar(C02, CHAR02);
  lcd.createChar(C21, CHAR21);
  lcd.createChar(C12, CHAR12);
  lcd.createChar(C22, CHAR22);
}

void lcdprint(byte col, byte row, int num, const char *fmt) {
  static char numBuf[21];
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

void lcdBar(float topValue, float topMinValue, float topMaxValue, float bottomValue, float bottomMinValue, float bottomMaxValue, int num_cols) {
  int num_cells = 2 * num_cols;
  int nrActiveTopCells = activeCells(topValue, topMinValue, topMaxValue, num_cells);
  int nrActiveBottomCells = activeCells(bottomValue, bottomMinValue, bottomMaxValue, num_cells);
  lcdRawBar(nrActiveTopCells, nrActiveBottomCells, num_cells);
}

byte requestAndReadPacket() {
  int bytesInPacket = 0;
  int expectedPacketSize = -1;
  byte incomingByte = 0;
  unsigned long readStart = millis();
  bool extraBytesReceived = false;

  memset(packet, 0, sizeof(packet));
  mySerial2.print("n");

  // Read until the expected packet length is complete or the timeout expires.
  while ((millis() - readStart) < WAITING_INTERVAL) {
    if (mySerial2.available() == 0) {
      continue;
    }

    incomingByte = mySerial2.read();

    if (bytesInPacket < MAX_PACKET_SIZE) {
      packet[bytesInPacket] = incomingByte;
    }
    bytesInPacket++;

    if (bytesInPacket == HEADER_SIZE) {
      if ((packet[0] != 'n') || (packet[1] != '2')) {
        return PACKET_STATUS_HEADER_INVALID;
      }

      expectedPacketSize = HEADER_SIZE + packet[2];
    }

    if ((expectedPacketSize >= HEADER_SIZE) && (bytesInPacket == expectedPacketSize)) {
      break;
    }
  }

  if (bytesInPacket < HEADER_SIZE) {
    return PACKET_STATUS_HEADER_INCOMPLETE;
  }

  if (bytesInPacket < expectedPacketSize) {
    return PACKET_STATUS_INCOMPLETE;
  }

  if (bytesInPacket > expectedPacketSize) {
    return PACKET_STATUS_TOO_LONG;
  }

  // Wait a little longer for unexpected trailing bytes and discard them if seen.
  readStart = millis();
  while ((millis() - readStart) < EXTRA_BYTE_WAITING_INTERVAL) {
    if (mySerial2.available() == 0) {
      continue;
    }

    extraBytesReceived = true;
    mySerial2.read();
  }

  if (extraBytesReceived) {
    return PACKET_STATUS_TOO_LONG;
  }

  return PACKET_STATUS_OK;
}

void setup() {
  lcd.begin(NUM_DISPLAY_COLS, NUM_DISPLAY_ROWS);
  registerCustomChars();

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
  unsigned long cycleStart = millis();

  packetStatusCode = requestAndReadPacket();

  if (packetStatusCode == PACKET_STATUS_OK) {
    lcdprint(0, 0, packet[PAYLOAD_OFFSET + RPM_HB] * 255 + packet[PAYLOAD_OFFSET + RPM_LB], "%4drpm ");
    lcdprint(8, 0, packet[PAYLOAD_OFFSET + ADVANCE_ANGLE], ADVANCE_FORMAT);
    lcdprint(12, 0, ((int)packet[PAYLOAD_OFFSET + COOLANT_PLUS_OFFSET]) - TEMPERATURE_OFFSET, "%3dC");
    lcdprint(16, 0, ((int)packet[PAYLOAD_OFFSET + IAT_PLUS_OFFSET]) - TEMPERATURE_OFFSET, "%3dC");

    lcdprint(0, 1, packet[PAYLOAD_OFFSET + MAP_HB] * 255 + packet[PAYLOAD_OFFSET + MAP_LB], "%4dkPa ");
    lcdprint(8, 1, packet[PAYLOAD_OFFSET + IDLE_LOAD], "%3d ");
    lcdprint(12, 1, packet[PAYLOAD_OFFSET + TPS], "%3d ");
    lcdprint(16, 1, packet[PAYLOAD_OFFSET + FUEL_PRESSURE] / 100, "%3dbar");

    lcdprint(0, 2, packet[PAYLOAD_OFFSET + OXIGEN] / 10, "%2d.");
    lcdprint(3, 2, packet[PAYLOAD_OFFSET + OXIGEN] % 10, "%01dafr  ");
    lcdprint(14, 2, engineStatus(packet[PAYLOAD_OFFSET + ENGINE]));

    lcd.setCursor(0, 3);
    lcdBar(
      float(packet[PAYLOAD_OFFSET + RPM_HB] * 255 + packet[PAYLOAD_OFFSET + RPM_LB]),
      0.0,
      7000.0,
      float(packet[PAYLOAD_OFFSET + OXIGEN]) / 10.0,
      10.0,
      20.0,
      NUM_DISPLAY_COLS
    );
  }

  timeConsumedByReadingAndDisplaying = millis() - cycleStart;
  if (timeConsumedByReadingAndDisplaying < POLLING_INTERVAL) {
    delay(POLLING_INTERVAL - timeConsumedByReadingAndDisplaying);
  }
}
