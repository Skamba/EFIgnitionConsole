# EFIgnition Console

A 20x4 character display for the Lomax, driven by a helper Arduino Mega 2560 that
listens to an **EFIgnition 46** engine management unit over **CAN**.

This repository was cloned from the original Speeduino display project so the
existing Git history stays intact, and the files are organized for day-to-day
development in VS Code.

## What changed with the EFIgnition 46

The car moved from a Speeduino to an EFIgnition 46, which is based on
MegaSquirt-2/Extra. It does not speak the Speeduino secondary-serial protocol, so
the previous firmware went quiet on the new ECU.

Rather than reimplementing a request/response protocol, this version uses
MegaSquirt's **Dash Broadcasting**: the ECU transmits a fixed set of channels over
CAN on its own, several times a second. Nothing has to be asked for.

| | Speeduino (before) | EFIgnition 46 (now) |
| --- | --- | --- |
| Transport | secondary serial, 115200 baud | CAN, 500 kbit/s, 11-bit identifiers |
| Style | display polls, ECU answers | ECU broadcasts, display listens |
| Shared with TunerStudio | no (separate serial port) | no (different bus entirely) |
| Setup in the tune | secondary serial set to Generic (Fixed List) | Dash Broadcasting on, Automatic |

Two practical gains: there is no request timing or packet validation to get right
any more, and tuning on the laptop and watching the display can happen at the same
time.

## What you need

Everything from the Speeduino build carries over. **One part has to be bought:**

| Part | Status |
| --- | --- |
| **MCP2515 CAN module** with a TJA1050 transceiver | **New — buy this.** A few euros. An Arduino Mega has no CAN controller of its own. |
| Arduino Mega 2560 helper board | already have |
| 20x4 character LCD with I2C backpack | already have |
| Fuel pressure sensor | already have — it moves here from the Speeduino, see below |
| 120 Ω resistor | only if the CAN module has none fitted |

**Read the crystal off the board.** These modules ship with either an 8 MHz or a
16 MHz crystal, stamped on the silver can, and the choice has to match
`CAN_CRYSTAL` in [`src/main.cpp`](src/main.cpp). Get it wrong and the module still
initialises without a word of complaint — it simply listens at the wrong bit rate
and nothing ever arrives. This is the single most common reason one of these
boards appears dead, and no shop listing mentions it.

The firmware is set to **8 MHz**, because that is what the common blue
MCP2515 + TJA1050 board carries. Check yours anyway: some batches ship 16 MHz, and
then the constant has to change.

Those boards also have a 120 Ω terminating resistor fitted, usually on a jumper
(J1). That is one of the two the bus needs — see [Before powering
up](#before-powering-up).

## Wiring

Full detail, including what to remove and how to calibrate the fuel pressure
sensor, is in [`docs/howToGetStarted.txt`](docs/howToGetStarted.txt). In short:

**CAN module (MCP2515) to the Mega — SPI**

| Module | Also printed as | Mega |
| --- | --- | --- |
| CS | NSS, SS | pin 53 |
| SI | MOSI, SDI | pin 51 |
| SO | MISO, SDO | pin 50 |
| SCK | CLK | pin 52 |
| VCC | 5V | 5V |
| GND | | GND |
| INT | | not connected |

The silkscreen wording varies between makers, hence the middle column. **SI and
SO are the pair that gets crossed**: SI is the module's input, so it comes from
the Mega's output on 51. Swap those two and the module never answers, which shows
up as `CAN chip not found`.

Pins 50-52 are the Mega's hardware SPI and cannot be moved. Only the chip select
is free (`CAN_CS_PIN`).

**CAN module to the ECU**

| Module | ECU |
| --- | --- |
| CAN-H | CAN-H |
| CAN-L | CAN-L |

Twisted pair. A CAN bus needs exactly two 120 ohm terminating resistors, one at
each end — most MCP2515 modules already have one, so check whether the ECU does
too.

**LCD to the Mega — I2C**

| LCD backpack | Mega |
| --- | --- |
| SDA | pin 20 |
| SCL | pin 21 |
| VCC / GND | 5V / GND |

Backpack address `0x27`. Some boards ship as `0x3F`.

**Fuel pressure sensor to the Mega — analogue**

| Sensor | Mega |
| --- | --- |
| signal | A0 |
| 5V / ground | 5V / GND |

This sensor used to hang on the Speeduino. The EFIgnition 46 has three analogue
inputs and all three are taken (wideband, MAP, throttle), so it moves to the
helper board. Nothing is lost by that: the Speeduino only ever *displayed* the
value — its firmware has no fuel pressure correction or protection, unlike oil
pressure, which does have one.

Set `FUEL_PRESSURE_CBAR_AT_0V` and `FUEL_PRESSURE_CBAR_AT_5V` in
[`src/main.cpp`](src/main.cpp) to match the sensor: pressure in hundredths of a
bar at 0 V and at 5 V, the same two-point form TunerStudio uses. The defaults suit
a 0.5-4.5 V sensor rated 0-60 psi. Do not copy the old Speeduino calibration,
which described a straight 0-5 V line and was probably wrong for that sensor.

**Removed:** the serial link on Mega pins 18 and 19, and the 10k resistors in
those wires. That went to the Speeduino.

### Before powering up

Note the crystal frequency on the module and set `CAN_CRYSTAL` in
[`src/main.cpp`](src/main.cpp) to match — see [What you need](#what-you-need).

With the power off, measure between CAN-H and CAN-L. About **60 Ω** is right: two
120 Ω terminating resistors in parallel, one at each end of the bus. 120 Ω means
only one is fitted, and an open circuit means neither.

## Setting up the ECU

One switch. In TunerStudio: **CAN bus / Testmodes → Dash Broadcasting**, set
Enable to on, leave Configuration on **Automatic**.

Automatic means base identifier 1512 at 20 Hz, which is what the firmware expects.
If you move to Advanced and change the identifier, change `CAN_BASE_ID` to match.

## What the display shows

```
4200rpm   210C   31C
 105kPa    85%   28°
12.8afr (12.5) 14.1V
 3.0bar  3.40ms  77%
```

| Row | Values |
| --- | --- |
| 1 | engine speed, cylinder head temperature, inlet air temperature |
| 2 | manifold pressure, throttle position, ignition advance |
| 3 | measured AFR, target AFR in brackets, board voltage |
| 4 | fuel pressure, injector pulse width, injector duty cycle |

Every field is wide enough for three digits. That is deliberate rather than
generous: a cylinder head on this engine sits around 150 °C and peaks past 200,
so three digits is the normal case here, and a tighter layout would run the two
temperatures together exactly when the engine is hot enough that you want to
read them.

**A field showing dashes means the CAN message carrying it has stopped
arriving**, not that the value is zero. Values are blanked per message rather
than left frozen, so a stale reading can never be mistaken for a live one.

The character in the bottom right is blank while all four CAN messages keep
arriving; a digit is the number that have gone quiet, and an **E** means the CAN
controller is reporting a bus fault — error-passive or bus-off.

## When nothing arrives

Rather than one "no data" message, the display works out *which* kind of nothing
it is looking at and names it. Each screen points at a different thing to check,
which is the whole reason for separating them.

| Screen | What it means | What to check |
| --- | --- | --- |
| `CAN chip not found` | The MCP2515 never answered over SPI, so the fault is between the Mega and the module. | SPI wiring, chip select on pin 53, power to the module. |
| `No CAN data received` / `Bus is quiet` | The controller works but there is nothing at all on the wire — not even something it fails to decode. | Is Dash Broadcasting switched on in the tune? Are CAN-H and CAN-L actually connected? |
| `No CAN data received` / `Signal but no decode` | Electrical activity is arriving that will not turn into valid frames. The receive error counter is climbing. | Crystal setting (8 vs 16 MHz) — by far the most common cause. CAN-H and CAN-L swapped. Termination: a healthy bus measures about 60 Ω across the pair with the power off. |
| `CAN bus is alive` / `Saw id 1234 not ours` | Frames are arriving and decoding cleanly, so wiring, bit rate and crystal are all proven good. Nobody is sending *our* identifiers. | The Base CAN identifier in the tune. The screen shows the identifier it actually saw, so you can compare it against the 1512-1515 this firmware expects. |

That last one is the most useful of the four: it turns "it does not work" into a
number you can read off the screen and type into TunerStudio.

The distinction between the two middle screens comes from the MCP2515's receive
error counter. A silent wire and a wire carrying something unintelligible look
identical from the outside, but they are completely different faults.

**Injector duty is calculated here**, not sent by the ECU. It assumes one injector
opening per engine cycle, which is what sequential injection gives — adjust
`SQUIRTS_PER_CYCLE` and `ALTERNATE_DIVIDER` if the tune changes to semi-sequential
or batch.

**Two values from the Speeduino version are gone**, because they are not part of
the dash broadcast: the idle valve position and the engine status letters (R, C,
A, W). Their places went to pulse width, target AFR and duty cycle, which are more
useful while the tune is still being built. If the status bits are ever wanted,
MegaSquirt can also broadcast its complete realtime block (CAN Broadcasting →
outpc groups); that carries everything but needs more configuration than one
switch.

## Project layout

- `src/main.cpp`: the whole firmware, deliberately in one file
- `platformio.ini`: PlatformIO environment for the Mega 2560 helper board
- `docs/howToGetStarted.txt`: wiring and setup notes
- `docs/images/`: reference photos and diagrams
- `lib/NewLiquidCrystal/`: trimmed copy of the LCD library

## Development

1. Open this folder in VS Code, or open `speeduino_console.code-workspace`.
2. Install the recommended extensions when VS Code asks for them.
3. Build and upload with the `megaatmega2560` PlatformIO environment.

```
pio run                  # build
pio run --target upload  # flash the helper board
```

The MCP2515 driver is fetched automatically on the first build; see `lib_deps` in
`platformio.ini`. The LCD support uses a vendored subset of fmalpartida's
`New-LiquidCrystal` in `lib/NewLiquidCrystal`, which keeps the original behaviour
while avoiding unused upstream sources that break AVR builds in PlatformIO.

If you move to a board other than a Mega 2560, revisit the SPI and I2C pin numbers
above — they are fixed by the Mega's hardware.

## Background

The message contents, their scaling and where those numbers were verified are
documented in the Efignition project repository, `docs/12-console.md`. Short
version: four messages of eight bytes from identifier 1512, all 16-bit values
big-endian, temperatures in tenths of a degree Fahrenheit, and injector pulse
width in 0.666 microsecond timer ticks.
