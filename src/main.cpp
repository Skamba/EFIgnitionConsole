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
// 20260901 | per-message staleness, bus error reporting, fixed 16-bit decoding
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

// Crystal fitted on the MCP2515 module. Getting this wrong is the single most
// common reason no frames arrive: the controller still initialises, it just
// listens at the wrong bit rate, and nothing on the display says why.
//
// Set to 8 MHz because that is what the common blue MCP2515 + TJA1050 board
// carries, which is the one this project uses. Do not take that on trust: read
// the number off the silver can on the board itself. Some batches ship 16 MHz,
// and then this has to be MCP_16MHZ.
constexpr byte CAN_CRYSTAL = MCP_8MHZ;

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

// The broadcast carries AFR in one byte (tenths, so 25.5 at most) and wraps
// beyond that: a sensor in free air reads about AFR 30 and arrives as 4.4,
// which would show as a terrifyingly rich mixture while the sensor is in fact
// reading extremely lean. A running engine sits between roughly 10 and 20, so
// anything below this floor is a wrapped over-range reading, shown as ">25".
constexpr byte AFR_WRAP_FLOOR_TENTHS = 70;

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

// Index into lastSeen[] per message, so a value can be blanked when the message
// that carries it stops arriving.
constexpr byte MSG_ENGINE   = 0;
constexpr byte MSG_FUELLING = 1;
constexpr byte MSG_MIXTURE  = 2;
constexpr byte MSG_ELECTRIC = 3;
constexpr byte NUM_CAN_MESSAGES = 4;

// The ECU is a 16-bit Motorola part, so every 16-bit value travels most
// significant byte first. An AVR is the other way round, hence the explicit
// assembly below. The int16_t cast is what makes negative values (retard, sub
// zero temperatures) come out right, and it keeps working on a board where int
// is 32 bits instead of 16.
static int16_t readSigned16(const byte *data, byte offset) {
  return (int16_t)(((uint16_t)data[offset] << 8) | data[offset + 1]);
}

static uint16_t readUnsigned16(const byte *data, byte offset) {
  return ((uint16_t)data[offset] << 8) | data[offset + 1];
}

// Integer division that rounds to nearest and keeps working for negatives,
// where plain / truncates towards zero (-25/10 gives -2, not -3).
static int divRound(int value, int divisor) {
  return (value >= 0) ? (value + divisor / 2) / divisor
                      : (value - divisor / 2) / divisor;
}

// The firmware always sends temperatures in tenths of a degree Fahrenheit. The
// Celsius setting in TunerStudio only affects what TunerStudio itself draws, so
// the conversion has to happen here.
static int fahrenheitTenthsToCelsius(int tenthsF) {
  return (int)(((long)tenthsF - 320L) * 5L / 90L);
}

// Pulse width arrives in timer ticks of 0.666 us, not in microseconds. Returned
// in hundredths of a millisecond so it can be printed without floating point.
static unsigned int pulseWidthTicksToHundredthMs(uint16_t ticks) {
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
static byte injectorDutyPercent(uint16_t pulseWidthTicks, uint16_t rpm) {
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

// Upper bound on frames handled in one pass. Without it a busy or misbehaving
// bus could keep the reader spinning and the display would freeze.
constexpr byte MAX_FRAMES_PER_PASS = 32;

// Bits in the MCP2515 error register that mean a real bus problem: receive and
// transmit error passive, and bus-off. The two overflow bits are deliberately
// left out. Overflow happens whenever a frame arrives while the LCD is being
// redrawn, which is normal here and harms nothing, and this library offers no
// way to clear those bits -- treating them as faults would light a permanent
// warning for a bus that is working fine.
constexpr byte CAN_BUS_FAULT_MASK = MCP_EFLG_RXEP | MCP_EFLG_TXEP | MCP_EFLG_TXBO;

struct EngineData {
  int16_t mapKpaTenths;
  uint16_t rpm;
  int16_t coolantTenthsF;
  int16_t throttleTenths;

  uint16_t pulseWidthTicks;
  int16_t inletTenthsF;
  int16_t advanceTenths;

  byte afrTargetTenths;
  byte afrTenths;

  int16_t batteryTenths;
};

EngineData engineData;
unsigned long lastSeen[NUM_CAN_MESSAGES];
bool everSeen[NUM_CAN_MESSAGES];
bool canStarted = false;

// Counters that make the difference between "the bus is dead" and "the bus is
// fine but nobody is saying what we expect". Without them every failure looks
// identical from the driver's seat.
unsigned long framesTotal = 0;   // any frame at all, ours or not
unsigned long framesOurs = 0;    // frames with one of our four identifiers
unsigned long lastOtherId = 0;   // the most recent identifier that was not ours

static bool fresh(byte index, unsigned long now) {
  return everSeen[index] && ((now - lastSeen[index]) < DATA_TIMEOUT);
}

static byte staleMessageCount(unsigned long now) {
  byte stale = 0;
  for (byte i = 0; i < NUM_CAN_MESSAGES; i++) {
    if (!fresh(i, now)) {
      stale++;
    }
  }
  return stale;
}

static void markSeen(byte index) {
  lastSeen[index] = millis();
  everSeen[index] = true;
}

// ---------------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------------

// Empties the receive buffer and files whatever was in it. Called on every pass
// through the main loop, far more often than the display is redrawn.
void readCanMessages() {
  unsigned long id = 0;
  byte ext = 0;
  byte length = 0;
  byte data[8];

  for (byte frame = 0; frame < MAX_FRAMES_PER_PASS; frame++) {
    if (can.checkReceive() != CAN_MSGAVAIL) {
      return;
    }
    if (can.readMsgBuf(&id, &ext, &length, data) != CAN_OK) {
      return;
    }
    framesTotal++;

    // The dash broadcast uses 11-bit identifiers. Ignoring extended frames stops
    // a 29-bit identifier from colliding with ours by coincidence.
    if (ext || length < 8) {
      continue;
    }

    if (id >= CAN_BASE_ID && id <= CAN_ID_ELECTRIC) {
      framesOurs++;
    }
    else {
      lastOtherId = id;
    }

    switch (id) {
      case CAN_ID_ENGINE:
        engineData.mapKpaTenths = readSigned16(data, 0);
        engineData.rpm = readUnsigned16(data, 2);
        engineData.coolantTenthsF = readSigned16(data, 4);
        engineData.throttleTenths = readSigned16(data, 6);
        markSeen(MSG_ENGINE);
        break;

      case CAN_ID_FUELLING:
        engineData.pulseWidthTicks = readUnsigned16(data, 0);
        // bytes 2-3 hold the second injector, which mirrors the first here
        engineData.inletTenthsF = readSigned16(data, 4);
        engineData.advanceTenths = readSigned16(data, 6);
        markSeen(MSG_FUELLING);
        break;

      case CAN_ID_MIXTURE:
        // Both are single bytes holding AFR times ten; anything past 25.5
        // wraps. The display treats impossibly low values as wrapped readings
        // (see AFR_WRAP_FLOOR_TENTHS); the raw bytes are stored unchanged.
        engineData.afrTargetTenths = data[0];
        engineData.afrTenths = data[1];
        markSeen(MSG_MIXTURE);
        break;

      case CAN_ID_ELECTRIC:
        engineData.batteryTenths = readSigned16(data, 0);
        // bytes 2-5 are the ECU's spare analogue inputs, unused on this car
        markSeen(MSG_ELECTRIC);
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

  // The calibration runs below zero on purpose, because the sensor's own range
  // starts at 0.5 V. An unconnected or shorted input therefore lands on a
  // negative pressure, which means nothing; showing it as zero is honest and
  // avoids a minus sign the tenths printer cannot render.
  if (centibar < 0) {
    centibar = 0;
  }
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

// Prints a number, or a placeholder of exactly the same width when the message
// carrying it has gone quiet. Showing a frozen value as if it were live is the
// one thing a gauge must never do.
void lcdfield(byte col, byte row, bool isFresh, int value, const char *fmt, const char *blank) {
  if (isFresh) {
    lcdprint(col, row, value, fmt);
  }
  else {
    lcdprint(col, row, blank);
  }
}

void lcdfieldTenths(byte col, byte row, bool isFresh, int value10, const char *suffix,
                    const char *blank) {
  static char numBuf[21];
  lcd.setCursor(col, row);
  if (isFresh) {
    snprintf(numBuf, sizeof(numBuf), "%2d.%1d%s", value10 / 10, abs(value10) % 10, suffix);
    lcd.print(numBuf);
  }
  else {
    lcd.print(blank);
  }
}

void lcdfieldHundredths(byte col, byte row, bool isFresh, unsigned int value100,
                        const char *suffix, const char *blank) {
  static char numBuf[21];
  lcd.setCursor(col, row);
  if (isFresh) {
    snprintf(numBuf, sizeof(numBuf), "%2u.%02u%s", value100 / 100, value100 % 100, suffix);
    lcd.print(numBuf);
  }
  else {
    lcd.print(blank);
  }
}

void showMessage(const char *l0, const char *l1, const char *l2, const char *l3) {
  lcd.clear();
  lcdprint(0, 0, l0);
  lcdprint(0, 1, l1);
  lcdprint(0, 2, l2);
  lcdprint(0, 3, l3);
}

void updateDisplay() {
  unsigned long now = millis();
  bool engineFresh = fresh(MSG_ENGINE, now);
  bool fuelFresh = fresh(MSG_FUELLING, now);
  bool mixFresh = fresh(MSG_MIXTURE, now);
  bool elecFresh = fresh(MSG_ELECTRIC, now);

  // Row 0 -- engine speed and the two temperatures.
  //
  // Every field gets room for three digits. That is not padding: a cylinder head
  // on this engine sits at 150 and peaks past 200, so three digits is the normal
  // case here, and a tighter layout runs the two temperatures into each other
  // exactly when the engine is hot enough that you want to read them.
  lcdfield(0, 0, engineFresh, (int)engineData.rpm, "%4drpm", " ---rpm");
  lcdfield(9, 0, engineFresh, fahrenheitTenthsToCelsius(engineData.coolantTenthsF), "%4dC", " ---C");
  lcdfield(15, 0, fuelFresh, fahrenheitTenthsToCelsius(engineData.inletTenthsF), "%4dC", " ---C");

  // Row 1 -- load, throttle and ignition advance
  lcdfield(0, 1, engineFresh, divRound(engineData.mapKpaTenths, 10), "%4dkPa", " ---kPa");
  lcdfield(9, 1, engineFresh, divRound(engineData.throttleTenths, 10), "%4d%%", " ---%");
  // 0xDF is the degree sign in the HD44780 character set
  lcdfield(15, 1, fuelFresh, divRound(engineData.advanceTenths, 10), "%4d\xDF", " ---\xDF");

  // Row 2 -- mixture against its target, and the board voltage the ECU sees.
  // An impossibly low AFR is a wrapped over-range byte (free air), not a rich
  // mixture; showing "4.4afr" there would send someone hunting a fuelling
  // fault that does not exist.
  if (mixFresh && engineData.afrTenths < AFR_WRAP_FLOOR_TENTHS) {
    lcdprint(0, 2, " >25afr");
  }
  else {
    lcdfieldTenths(0, 2, mixFresh, engineData.afrTenths, "afr", "--.-afr");
  }
  lcd.setCursor(8, 2);
  if (mixFresh) {
    static char buf[8];
    snprintf(buf, sizeof(buf), "(%2d.%1d)", engineData.afrTargetTenths / 10,
             engineData.afrTargetTenths % 10);
    lcd.print(buf);
  }
  else {
    lcd.print("(--.-)");
  }
  lcdfieldTenths(15, 2, elecFresh, engineData.batteryTenths, "V", "--.-V");

  // Row 3 -- fuel pressure from our own sensor, how long the injector is open,
  // and how hard it is working
  if (FUEL_PRESSURE_CONNECTED) {
    lcdfieldTenths(0, 3, true, readFuelPressureTenths(), "bar", "--.-bar");
  }
  else {
    lcdprint(0, 3, "       ");
  }
  lcdfieldHundredths(8, 3, fuelFresh, pulseWidthTicksToHundredthMs(engineData.pulseWidthTicks),
                     "ms", "--.--ms");
  // Duty needs both the pulse width and the engine speed, so it is only shown
  // when the two messages carrying them are both current.
  lcdfield(15, 3, engineFresh && fuelFresh,
           injectorDutyPercent(engineData.pulseWidthTicks, engineData.rpm),
           "%3d%%", " --%");

  // Bottom right corner: blank while all four messages keep arriving, otherwise
  // how many of them have gone quiet. An E means the CAN controller itself is
  // reporting a bus fault, which points at wiring or termination rather than at
  // a setting in the tune.
  if (can.getError() & CAN_BUS_FAULT_MASK) {
    lcdprint(19, 3, "E");
  }
  else {
    byte stale = staleMessageCount(now);
    if (stale == 0) {
      lcdprint(19, 3, " ");
    }
    else {
      lcdprint(19, 3, stale, "%1d");
    }
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
  memset(everSeen, 0, sizeof(everSeen));

  // 500 kbit/s and 11-bit identifiers: both fixed in the ECU's firmware and not
  // adjustable from TunerStudio.
  canStarted = (can.begin(MCP_ANY, CAN_500KBPS, CAN_CRYSTAL) == CAN_OK);

  if (canStarted) {
    // Normal mode, not listen-only: a CAN transmitter needs at least one other
    // node to acknowledge its frames, and here that is us.
    can.setMode(MCP_NORMAL);
  }
  // If it did not start, the main loop puts the diagnostic screen up. Doing it
  // there keeps every failure message in one place.
}

// Which screen the current situation calls for. Each one names a different
// thing to go and check, which is the whole point of separating them.
enum Screen : byte {
  SCREEN_VALUES,     // at least one message is current
  SCREEN_NO_CHIP,    // the MCP2515 did not answer over SPI
  SCREEN_QUIET,      // nothing on the wire at all
  SCREEN_NOISE,      // electrical activity that will not decode
  SCREEN_OTHER_IDS,  // the bus works, but nobody is sending our identifiers
};

Screen chooseScreen(unsigned long now) {
  if (!canStarted) {
    return SCREEN_NO_CHIP;
  }
  if (staleMessageCount(now) < NUM_CAN_MESSAGES) {
    return SCREEN_VALUES;
  }

  // Traffic that is not ours proves the wiring, the bit rate and the crystal are
  // all right, which narrows the problem down to one setting: the base
  // identifier in the tune, or Dash Broadcasting being switched off.
  if (framesOurs == 0 && framesTotal > 0) {
    return SCREEN_OTHER_IDS;
  }

  // Receive errors mean the controller is seeing edges it cannot turn into
  // frames. That is a different fault from a wire with nothing on it: it points
  // at the bit rate (so the crystal setting), at swapped CAN-H and CAN-L, or at
  // termination, rather than at a silent ECU.
  bool busFault = (can.getError() & CAN_BUS_FAULT_MASK) != 0;
  return (can.errorCountRX() > 0 || busFault) ? SCREEN_NOISE : SCREEN_QUIET;
}

void showDiagnosticScreen(Screen screen) {
  static char idLine[21];

  switch (screen) {
    case SCREEN_NO_CHIP:
      showMessage("CAN chip not found",
                  "Check SPI wiring,",
                  "CS on pin 53, and",
                  "power to the module");
      break;

    case SCREEN_QUIET:
      showMessage("No CAN data received",
                  "Bus is quiet. Is",
                  "Dash Broadcasting",
                  "on? Is CAN-H/L on?");
      break;

    case SCREEN_NOISE:
      showMessage("No CAN data received",
                  "Signal but no decode",
                  "Crystal 8 or 16MHz?",
                  "H/L swapped? 60ohm?");
      break;

    case SCREEN_OTHER_IDS:
      // 11-bit identifiers stop at 2047, so this is 20 characters at its longest.
      snprintf(idLine, sizeof(idLine), "Saw id %lu not ours", lastOtherId);
      showMessage("CAN bus is alive",
                  idLine,
                  "Expect 1512 to 1515",
                  "Check Base CAN id");
      break;

    default:
      break;
  }
}

void loop() {
  static unsigned long lastDisplay = 0;
  static Screen lastScreen = SCREEN_VALUES;
  static bool everDrawn = false;

  if (!canStarted) {
    // Nothing to read, but the screen still has to be put up once.
    if (!everDrawn) {
      showDiagnosticScreen(SCREEN_NO_CHIP);
      everDrawn = true;
    }
    delay(1000);
    return;
  }

  readCanMessages();

  unsigned long now = millis();
  if ((now - lastDisplay) < DISPLAY_INTERVAL) {
    return;
  }
  lastDisplay = now;

  Screen screen = chooseScreen(now);

  if (screen != lastScreen || !everDrawn) {
    lcd.clear();
    lastScreen = screen;
    everDrawn = true;
    if (screen != SCREEN_VALUES) {
      showDiagnosticScreen(screen);
      return;
    }
  }

  if (screen == SCREEN_VALUES) {
    updateDisplay();
  }
}
