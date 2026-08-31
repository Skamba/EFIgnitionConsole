#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <mcp_can.h>

// Module for listening to an EFIgnition 46 (MegaSquirt-2/Extra) over CAN and
// showing the result on a 20x4 LCD.
//
// Lex Sewuster (aka Zeiberstein)
// 20200918 | initial creation
// 20201021 | added moving bars
// 20210103 | fixed temperature reading
// 20260402 | migrated to enhanced "n" command and added fuel pressure
// 20260403 | removed moving bars and moved fuel pressure to last line
// 20260412 | cleaned up naming and payload access for display fields
// 20260412 | improved serial buffer cleanup before and after packet reads
// 20260412 | switched advance display to inline LCD degree escape and hid status 0
// 20260901 | rewritten for the EFIgnition 46: CAN instead of Speeduino serial
//
// Why this changed
// ----------------
// The Speeduino spoke a secondary-serial protocol ("n" request, "n2" reply) that
// the EFIgnition 46 does not implement. MegaSquirt-2/Extra instead offers "Dash
// Broadcasting": it transmits a fixed set of channels over CAN on its own, with
// no request needed. That is simpler and more robust than polling, and it does
// not compete with TunerStudio for a port, so tuning and watching can happen at
// the same time.
//
// Enable it in TunerStudio under CAN bus / Testmodes > Dash Broadcasting. Leave
// the configuration on "Automatic", which is base identifier 1512 at 20 Hz.
//
// Two channels the Speeduino used to provide are NOT part of the dash broadcast:
// the idle valve position and the engine status bits (running/cranking/ASE/
// warmup). Their places are taken by injector pulse width, target AFR and the
// calculated injector duty cycle, which matter more while the tune is being
// built. If the status bits are ever needed, MegaSquirt can also broadcast its
// complete realtime block ("CAN Broadcasting" -> outpc groups); that carries
// everything but needs more configuration than one switch.
//
// See docs/howToGetStarted.txt for wiring.

// ---------------------------------------------------------------------------
// Things you may need to change
// ---------------------------------------------------------------------------

// Chip select of the MCP2515 module. The other three SPI lines are fixed by the
// Mega's hardware SPI: SI = 51, SO = 50, SCK = 52.
constexpr byte CAN_CS_PIN = 53;

// Crystal fitted on the MCP2515 module. Boards ship with either, the silkscreen
// or the metal can tells you which. Getting this wrong is the single most common
// reason no frames arrive: the controller still initialises, it just listens at
// the wrong bit rate. Use MCP_8MHZ if your module has an 8 MHz crystal.
constexpr byte CAN_CRYSTAL = MCP_16MHZ;

// Fuel pressure is measured by this board, not by the ECU. The EFIgnition 46 has
// three analogue inputs and all three are taken (lambda, MAP, throttle), so the
// sensor that used to hang on the Speeduino moves here. Nothing is lost by that:
// the Speeduino only displayed the value, it never corrected fuelling with it.
constexpr bool FUEL_PRESSURE_CONNECTED = true;
constexpr byte FUEL_PRESSURE_PIN = A0;

// Calibration, in hundredths of a bar, at 0 V and at 5 V. Most automotive
// pressure sensors output 0.5 V at zero pressure and 4.5 V at full scale, so
// those two points fall outside the sensor's own range and have to be
// extrapolated -- exactly like the MAP sensor calibration in TunerStudio.
//
// The values below are for a 0.5-4.5 V sensor rated 0-60 psi (0-4.14 bar):
//     at 0 V: -0.5 V * (4.14 bar / 4.0 V) = -0.52 bar
//     at 5 V:  4.14 + 0.52                =  4.66 bar
//
// CHECK THIS AGAINST THE SENSOR ON THE CAR. The old Speeduino tune had 0 bar at
// 0 V and 4.19 bar at 5 V, a straight line across the whole range, which is not
// how a 0.5-4.5 V sensor behaves. If that is the sensor fitted, the old reading
// was offset high at rest and a few percent low at working pressure.
constexpr int FUEL_PRESSURE_CBAR_AT_0V = -52;
constexpr int FUEL_PRESSURE_CBAR_AT_5V = 466;

// Injector openings per injector per engine cycle, as a fraction. Sequential
// injection on this engine gives one full opening per 720 degrees, so 1/1. Set
// this to match the tune: with semi-sequential (two half pulses per cycle) the
// duty calculation needs 2/2, which is the same number, but with a batch setup
// firing every revolution it would be 2/1. Only the duty display uses it.
constexpr byte SQUIRTS_PER_CYCLE = 1;
constexpr byte ALTERNATE_DIVIDER = 1;

// ---------------------------------------------------------------------------
// The dash broadcast
// ---------------------------------------------------------------------------

// Base identifier, and the four consecutive messages that follow it. These are
// the "Automatic" defaults; change the base if the tune uses Advanced mode.
constexpr unsigned long CAN_BASE_ID = 1512;  // 0x5E8

constexpr unsigned long CAN_ID_ENGINE   = CAN_BASE_ID + 0;  // map, rpm, clt, tps
constexpr unsigned long CAN_ID_FUELLING = CAN_BASE_ID + 1;  // pw1, pw2, mat, advance
constexpr unsigned long CAN_ID_MIXTURE  = CAN_BASE_ID + 2;  // afr target, afr, ego corr
constexpr unsigned long CAN_ID_ELECTRIC = CAN_BASE_ID + 3;  // battery, spare adc, knock

constexpr byte NUM_CAN_MESSAGES = 4;

// The ECU is a 16-bit Motorola part, so every 16-bit value travels most
// significant byte first. An AVR is the other way round, hence the explicit
// assembly below rather than a memcpy.
static int readSigned16(const byte *data, byte offset) {
  return (int)(((unsigned int)data[offset] << 8) | data[offset + 1]);
}

static unsigned int readUnsigned16(const byte *data, byte offset) {
  return ((unsigned int)data[offset] << 8) | data[offset + 1];
}

// The firmware always sends temperatures in tenths of a degree Fahrenheit. The
// Celsius setting in TunerStudio only affects what TunerStudio itself draws, so
// the conversion has to happen here.
static int fahrenheitTenthsToCelsius(int tenthsF) {
  return ((long)tenthsF - 320L) * 5L / 90L;
}

// Pulse width arrives in timer ticks of 0.666 us, not in microseconds. Returned
// in hundredths of a millisecond so it can be printed without floating point.
static unsigned int pulseWidthTicksToHundredthMs(unsigned int ticks) {
  return (unsigned int)(((unsigned long)ticks * 666UL) / 10000UL);
}

// Injector duty, in whole percent.
//
//   cycle time (ms) = 120000 / rpm            (four-stroke: two revolutions)
//   pulse width (ms) = ticks * 0.000666
//   duty = 100 * squirts / divider * pw / cycle time
//
// Folding the constants together gives ticks * rpm / 1801802 for one opening per
// cycle, which stays inside a 32-bit integer for any pulse width this engine can
// produce.
static byte injectorDutyPercent(unsigned int pulseWidthTicks, unsigned int rpm) {
  if (rpm == 0) {
    return 0;
  }
  unsigned long duty = (unsigned long)pulseWidthTicks * (unsigned long)rpm;
  duty = duty * SQUIRTS_PER_CYCLE / ALTERNATE_DIVIDER / 1801802UL;
  return (duty > 255UL) ? 255 : (byte)duty;
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
MCP_CAN can(CAN_CS_PIN);

constexpr int NUM_DISPLAY_COLS = 20;
constexpr int NUM_DISPLAY_ROWS = 4;

// How long a message may stay away before its values are no longer believed.
// The ECU sends at 20 Hz, so a second is already forty missed messages.
constexpr unsigned long DATA_TIMEOUT = 1000UL;

// The LCD is driven over I2C and redrawing it costs time, so it is refreshed far
// slower than the messages arrive. Everything in between is still read, so the
// values shown are always the most recent ones.
constexpr unsigned long DISPLAY_INTERVAL = 250UL;

struct EngineData {
  int mapKpaTenths;
  unsigned int rpm;
  int coolantTenthsF;
  int throttleTenths;

  unsigned int pulseWidthTicks;
  int inletTenthsF;
  int advanceTenths;

  byte afrTargetTenths;
  byte afrTenths;

  int batteryTenths;
};

EngineData engineData;
unsigned long lastSeen[NUM_CAN_MESSAGES];
bool canStarted = false;

static bool messageFresh(byte index, unsigned long now) {
  return (lastSeen[index] != 0) && ((now - lastSeen[index]) < DATA_TIMEOUT);
}

static byte staleMessageCount(unsigned long now) {
  byte stale = 0;
  for (byte i = 0; i < NUM_CAN_MESSAGES; i++) {
    if (!messageFresh(i, now)) {
      stale++;
    }
  }
  return stale;
}

// ---------------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------------

// Empties the receive buffer and files whatever was in it. Called far more often
// than the display is redrawn so the MCP2515 never overflows.
void readCanMessages() {
  unsigned long id = 0;
  byte length = 0;
  byte data[8];

  while (can.checkReceive() == CAN_MSGAVAIL) {
    if (can.readMsgBuf(&id, &length, data) != CAN_OK) {
      return;
    }
    if (length < 8) {
      continue;  // every dash broadcast message is a full eight bytes
    }

    switch (id) {
      case CAN_ID_ENGINE:
        engineData.mapKpaTenths = readSigned16(data, 0);
        engineData.rpm = readUnsigned16(data, 2);
        engineData.coolantTenthsF = readSigned16(data, 4);
        engineData.throttleTenths = readSigned16(data, 6);
        lastSeen[0] = millis();
        break;

      case CAN_ID_FUELLING:
        engineData.pulseWidthTicks = readUnsigned16(data, 0);
        // bytes 2-3 hold the second injector, which mirrors the first here
        engineData.inletTenthsF = readSigned16(data, 4);
        engineData.advanceTenths = readSigned16(data, 6);
        lastSeen[1] = millis();
        break;

      case CAN_ID_MIXTURE:
        // Both are single bytes holding AFR times ten, so they run out at 25.5.
        // A sensor in free air reads past that; clamping keeps the display sane.
        engineData.afrTargetTenths = data[0];
        engineData.afrTenths = data[1];
        lastSeen[2] = millis();
        break;

      case CAN_ID_ELECTRIC:
        engineData.batteryTenths = readSigned16(data, 0);
        // bytes 2-5 are the ECU's spare analogue inputs, unused on this car
        lastSeen[3] = millis();
        break;

      default:
        break;  // something else on the bus; not ours
    }
  }
}

// Fuel pressure in tenths of a bar, measured by this board. Averaged over a few
// samples because a fuel rail pulses and the display should not flicker.
int readFuelPressureTenths() {
  constexpr byte SAMPLES = 8;
  unsigned int total = 0;

  for (byte i = 0; i < SAMPLES; i++) {
    total += analogRead(FUEL_PRESSURE_PIN);
  }

  long counts = total / SAMPLES;
  long span = FUEL_PRESSURE_CBAR_AT_5V - FUEL_PRESSURE_CBAR_AT_0V;
  long centibar = FUEL_PRESSURE_CBAR_AT_0V + (span * counts) / 1023L;

  return (int)(centibar / 10);
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

void lcdprint(byte col, byte row, int num, const char *fmt) {
  static char numBuf[21];
  lcd.setCursor(col, row);
  snprintf(numBuf, sizeof(numBuf), fmt, num);
  lcd.print(numBuf);
}

void lcdprint(byte col, byte row, const char *text) {
  lcd.setCursor(col, row);
  lcd.print(text);
}

void lcdprintTenths(byte col, byte row, int value10, const char *suffix, byte wholeWidth = 2) {
  static char numBuf[21];
  lcd.setCursor(col, row);
  if (wholeWidth <= 1) {
    snprintf(numBuf, sizeof(numBuf), "%d.%1d%s", value10 / 10, abs(value10) % 10, suffix);
  }
  else {
    snprintf(numBuf, sizeof(numBuf), "%2d.%1d%s", value10 / 10, abs(value10) % 10, suffix);
  }
  lcd.print(numBuf);
}

void lcdprintHundredths(byte col, byte row, unsigned int value100, const char *suffix) {
  static char numBuf[21];
  lcd.setCursor(col, row);
  snprintf(numBuf, sizeof(numBuf), "%2u.%02u%s", value100 / 100, value100 % 100, suffix);
  lcd.print(numBuf);
}

void showWaitingForData() {
  lcd.clear();
  lcdprint(0, 0, "Geen data van de");
  lcdprint(0, 1, "ECU. Staat Dash");
  lcdprint(0, 2, "Broadcasting aan?");
  lcdprint(0, 3, "Zie werkboek blad 9");
}

void updateDisplay() {
  unsigned long now = millis();

  // Row 0 -- what the engine is doing
  lcdprint(0, 0, (int)engineData.rpm, "%4drpm ");
  lcdprint(8, 0, engineData.advanceTenths / 10, "%3d\xDF ");  // 0xDF is the HD44780 degree sign
  lcdprint(12, 0, fahrenheitTenthsToCelsius(engineData.coolantTenthsF), "%3dC");
  lcdprint(16, 0, fahrenheitTenthsToCelsius(engineData.inletTenthsF), "%3dC");

  // Row 1 -- load, throttle and how long the injector is open
  lcdprint(0, 1, engineData.mapKpaTenths / 10, "%4dkPa ");
  lcdprint(8, 1, engineData.throttleTenths / 10, "%3d%% ");
  lcdprintHundredths(13, 1, pulseWidthTicksToHundredthMs(engineData.pulseWidthTicks), "ms");

  // Row 2 -- mixture against its target, and the board voltage the ECU sees
  lcdprintTenths(0, 2, engineData.afrTenths, "afr");
  lcd.setCursor(8, 2);
  {
    static char buf[8];
    snprintf(buf, sizeof(buf), "(%2d.%1d)", engineData.afrTargetTenths / 10,
             engineData.afrTargetTenths % 10);
    lcd.print(buf);
  }
  lcdprintTenths(15, 2, engineData.batteryTenths, "V");

  // Row 3 -- fuel pressure from our own sensor, and how hard the injector works
  if (FUEL_PRESSURE_CONNECTED) {
    lcdprintTenths(0, 3, readFuelPressureTenths(), "bar");
  }
  else {
    lcdprint(0, 3, "       ");
  }
  lcdprint(8, 3, injectorDutyPercent(engineData.pulseWidthTicks, engineData.rpm), "%3d%%duty");

  // Bottom right corner: blank while all four messages keep arriving, otherwise
  // how many of them have gone quiet.
  byte stale = staleMessageCount(now);
  if (stale == 0) {
    lcdprint(19, 3, " ");
  }
  else {
    lcdprint(19, 3, stale, "%1d");
  }
}

// ---------------------------------------------------------------------------

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

  memset(&engineData, 0, sizeof(engineData));
  memset(lastSeen, 0, sizeof(lastSeen));

  // 500 kbit/s and 11-bit identifiers: both fixed in the ECU's firmware and not
  // adjustable from TunerStudio.
  canStarted = (can.begin(MCP_ANY, CAN_500KBPS, CAN_CRYSTAL) == CAN_OK);

  if (canStarted) {
    can.setMode(MCP_NORMAL);  // listen and acknowledge; the bus needs the ack
    pinMode(CAN_CS_PIN, OUTPUT);
  }
  else {
    lcdprint(0, 0, "CAN start mislukt");
    lcdprint(0, 1, "Controleer bedrading");
    lcdprint(0, 2, "en het kristal op de");
    lcdprint(0, 3, "module (8 of 16MHz)");
  }
}

void loop() {
  static unsigned long lastDisplay = 0;
  static bool showedWaiting = false;

  if (!canStarted) {
    delay(1000);
    return;
  }

  readCanMessages();

  unsigned long now = millis();
  if ((now - lastDisplay) < DISPLAY_INTERVAL) {
    return;
  }
  lastDisplay = now;

  // Nothing at all coming in is a different problem from a single missing
  // message, and it deserves a screen that says what to check.
  if (staleMessageCount(now) == NUM_CAN_MESSAGES) {
    if (!showedWaiting) {
      showWaitingForData();
      showedWaiting = true;
    }
    return;
  }

  if (showedWaiting) {
    lcd.clear();
    showedWaiting = false;
  }

  updateDisplay();
}
