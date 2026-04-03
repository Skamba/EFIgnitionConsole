#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
HardwareSerial &speeduinoSerial = Serial1;  // RX1 = 19, TX1 = 18

// Module for reading Speeduino's serial3 port and displaying it on a 20x4 LCD.
//
// Lex Sewuster (aka Zeiberstein)
// 20200918 | initial creation
// 20201021 | added moving bars
// 20210103 | fixed temperature reading
// 20260402 | migrated to enhanced "n" command and added fuel pressure
// 20260403 | removed moving bars and moved fuel pressure to last line
//
// See https://speeduino.com/wiki/index.php/Secondary_Serial_IO_interface

// Position numbers in Speeduino's real-time data block.
constexpr byte SQUIRT               =   1;
constexpr byte ENGINE               =   2;
constexpr byte DWELL                =   3;
constexpr byte MAP_LB               =   4;
constexpr byte MAP_HB               =   5;
constexpr byte IAT_PLUS_OFFSET      =   6;
constexpr byte COOLANT_PLUS_OFFSET  =   7;
constexpr byte BAT_CORRECTION       =   8;
constexpr byte BATTERY10            =   9;
constexpr byte OXIGEN               =  10;
constexpr byte EGO_CORRECTION       =  11;
constexpr byte IAT_CORRECTION       =  12;
constexpr byte WUE_CORRECTION       =  13;
constexpr byte RPM_LB               =  14;
constexpr byte RPM_HB               =  15;
constexpr byte TAE_AMOUNT           =  16;
constexpr byte CORRECTIONS          =  17;
constexpr byte VE                   =  18;
constexpr byte AFR_TARGET           =  19;
constexpr byte PW1_LB               =  20;
constexpr byte PW1_HB               =  21;
constexpr byte TPS_DOT              =  22;
constexpr byte ADVANCE_ANGLE        =  23;
constexpr byte TPS                  =  24;
constexpr byte LOOPS_PER_SECOND_LB  =  25;
constexpr byte LOOPS_PER_SECOND_HB  =  26;
constexpr byte FREE_RAM_LB          =  27;
constexpr byte FREE_RAM_HB          =  28;
constexpr byte BOOST_TARGET         =  29;
constexpr byte BOOST_DUTY           =  30;
constexpr byte SPARK                =  31;
constexpr byte RPM_DOT_LB           =  32;
constexpr byte RPM_DOT_HB           =  33;
constexpr byte ETHANOL_PCT          =  34;
constexpr byte FLEX_CORRECTION      =  35;
constexpr byte FLEX_IGN_CORRECTION  =  36;
constexpr byte IDLE_LOAD            =  37;
constexpr byte TEST_OUTPUTS         =  38;
constexpr byte OXIGEN2              =  39;
constexpr byte BARO                 =  40;
constexpr byte FUEL_PRESSURE        = 103;

constexpr byte BIT_ENGINE_RUN       =   0;
constexpr byte BIT_ENGINE_CRANK     =   1;
constexpr byte BIT_ENGINE_ASE       =   2;
constexpr byte BIT_ENGINE_WARMUP    =   3;
constexpr byte BIT_ENGINE_ACC       =   4;
constexpr byte BIT_ENGINE_DCC       =   5;
constexpr byte BIT_ENGINE_MAPACC    =   6;
constexpr byte BIT_ENGINE_MAPDCC    =   7;

const char ENGINE_STATUS_CHAR[] = {'R', 'C', 'A', 'W', 'a', 'd', '<', '>'};
const char ADVANCE_FORMAT[] = {'%', '3', 'd', char(223), ' ', '\0'};

constexpr int TEMPERATURE_OFFSET = 40;
constexpr unsigned long WAITING_INTERVAL = 100UL;
constexpr unsigned long EXTRA_BYTE_WAITING_INTERVAL = 5UL;
constexpr unsigned long POLLING_INTERVAL = 1000UL;

constexpr byte HEADER_SIZE = 3;
constexpr byte PAYLOAD_OFFSET = HEADER_SIZE - 1;
constexpr int MAX_PACKET_SIZE = 300;

constexpr byte PACKET_STATUS_OK                = 0;
constexpr byte PACKET_STATUS_OVERFLOW          = 1;
constexpr byte PACKET_STATUS_HEADER_INCOMPLETE = 2;
constexpr byte PACKET_STATUS_HEADER_INVALID    = 3;
constexpr byte PACKET_STATUS_INCOMPLETE        = 4;
constexpr byte PACKET_STATUS_TOO_LONG          = 5;

constexpr int NUM_DISPLAY_COLS = 20;
constexpr int NUM_DISPLAY_ROWS = 4;

byte packet[MAX_PACKET_SIZE];  // More than enough for the maximum payload plus header.
byte packetStatusCode = PACKET_STATUS_OK;
unsigned long timeConsumedByReadingAndDisplaying;

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

byte requestAndReadPacket() {
  int bytesInPacket = 0;
  int expectedPacketSize = -1;
  byte incomingByte = 0;
  unsigned long readStart = millis();
  bool extraBytesReceived = false;

  memset(packet, 0, sizeof(packet));
  speeduinoSerial.print("n");

  // Read until the expected packet length is complete or the timeout expires.
  while ((millis() - readStart) < WAITING_INTERVAL) {
    if (speeduinoSerial.available() == 0) {
      continue;
    }

    incomingByte = speeduinoSerial.read();

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

  // Wait a little longer for unexpected trailing bytes and discard them if seen.
  readStart = millis();
  while ((millis() - readStart) < EXTRA_BYTE_WAITING_INTERVAL) {
    if (speeduinoSerial.available() == 0) {
      continue;
    }
    extraBytesReceived = true;
    speeduinoSerial.read();
  }

  if (extraBytesReceived) {
    return PACKET_STATUS_TOO_LONG;
  }

  return PACKET_STATUS_OK;
}

void setup() {
  lcd.begin(NUM_DISPLAY_COLS, NUM_DISPLAY_ROWS);

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
  speeduinoSerial.begin(115200);
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
    lcdprint(16, 1, "    ");

    lcdprint(0, 2, packet[PAYLOAD_OFFSET + OXIGEN] / 10, "%2d.");
    lcdprint(3, 2, packet[PAYLOAD_OFFSET + OXIGEN] % 10, "%01dafr  ");
    lcdprint(14, 2, engineStatus(packet[PAYLOAD_OFFSET + ENGINE]));

    int fuelBar10 = (long(packet[PAYLOAD_OFFSET + FUEL_PRESSURE]) * 6895L + 5000L) / 10000L;
    lcdprint(0, 3, fuelBar10 / 10, "%1d.");
    lcdprint(2, 3, fuelBar10 % 10, "%1dbar            ");

    lcdprint(19, 3, packetStatusCode, "%1d");
  }
  else {
    lcdprint(0, 0, "Fout bij lezen:      ");
    lcdprint(0, 1, packetStatusCode, "%1d                   ");
    lcdprint(0, 2, "                    ");
    lcdprint(0, 3, "                    ");
  }

  timeConsumedByReadingAndDisplaying = millis() - cycleStart;
  if (timeConsumedByReadingAndDisplaying < POLLING_INTERVAL) {
    delay(POLLING_INTERVAL - timeConsumedByReadingAndDisplaying);
  }
}
