# SpeeduinoConsole

PlatformIO and VS Code project for reading Speeduino data and showing it on a 20x4 character LCD.

This repository was cloned from the original GitHub project so the existing Git history stays intact, and the files are now organized for day-to-day development in VS Code.

## Current Lomax Version

- Uses Speeduino's lowercase `"n"` command instead of the older `"a"` request, so the helper receives the larger fixed secondary-serial data block.
- Uses the helper Arduino's hardware `Serial1` port (`RX1 = 19`, `TX1 = 18`) instead of a `SoftwareSerial` port.
- Uses a stricter packet read loop that validates the `n2` header, tracks payload length, rejects incomplete or overlong packets, and recovers faster from bad serial data.

## Project Layout

- `src/main.cpp`: main Arduino application
- `platformio.ini`: PlatformIO environment for an Arduino Mega 2560 helper board
- `docs/howToGetStarted.txt`: wiring and setup notes for the current Lomax hardware
- `docs/images/`: reference photos and diagrams

## Features

- Uses Speeduino's enhanced `"n"` command for the larger real-time data block
- Expects the Speeduino secondary serial protocol to be set to `Generic (Fixed List)`
- Reads Speeduino through helper-board `Serial1` on pins 19/18
- Detects invalid, incomplete, and unexpectedly long packets before updating the display
- Displays fuel pressure on the last line
- Keeps the firmware logic in a single `main.cpp` while the project is still evolving

## Development

1. Open this folder in VS Code, or open `speeduino_console.code-workspace`.
2. Install the recommended extensions when VS Code asks for them.
3. Build and upload with the `megaatmega2560` PlatformIO environment.
4. If you move the project to a non-Mega helper board, update the serial-port choice in `src/main.cpp` because the current wiring expects hardware `Serial1`.

## Dependency

The LCD support uses a vendored subset of fmalpartida's `New-LiquidCrystal` library in `lib/NewLiquidCrystal`, which keeps the original behavior while avoiding unused upstream sources that break AVR builds in PlatformIO.
