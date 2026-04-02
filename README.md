# SpeeduinoConsole

PlatformIO and VS Code project for reading Speeduino data and showing it on a 20x4 character LCD.

This repository was cloned from the original GitHub project so the existing Git history stays intact, and the files are now organized for day-to-day development in VS Code.

## Project Layout

- `src/main.cpp`: main Arduino application
- `src/custom_chars.cpp`: LCD bar character definitions
- `src/display_functions.cpp`: LCD helper procedures
- `src/speeduino_data.cpp`: Speeduino constants and display timing values
- `include/`: matching header files for the split source layout
- `platformio.ini`: PlatformIO environment for an Arduino Mega 2560
- `docs/howToGetStarted.txt`: original wiring and setup notes
- `docs/images/`: reference photos and diagrams

## Features

- Uses Speeduino's enhanced `"n"` command for the larger real-time data block
- Displays fuel pressure in addition to the original values
- Keeps the modular source split from the Linux working copy while staying in the current Git-managed Windows project

## Development

1. Open this folder in VS Code, or open `speeduino_console.code-workspace`.
2. Install the recommended extensions when VS Code asks for them.
3. Build and upload with the `megaatmega2560` PlatformIO environment.

## Dependency

The LCD support uses a vendored subset of fmalpartida's `New-LiquidCrystal` library in `lib/NewLiquidCrystal`, which keeps the original behavior while avoiding unused upstream sources that break AVR builds in PlatformIO.
