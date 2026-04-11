# Lomax Pre-Release 2026-04-11

Tag: `lomax-pre-release-20260411`
Firmware commit: `671f904`

This pre-release captures the current working Lomax firmware and documentation update.
It has been flashed to the Arduino Mega Pro in the Lomax and verified visually on the car.

## Highlights

- Switched the helper Arduino from software serial to hardware `Serial1` (`RX1 = 19`, `TX1 = 18`)
- Switched the Speeduino request from the older `"a"` command to the lowercase `"n"` command for the larger fixed-list data block
- Hardened the serial packet read loop:
  - validates the `n2` header
  - uses the payload length from the packet itself
  - detects incomplete packets
  - detects and discards unexpected extra bytes
- Displays fuel pressure on the last LCD line
- Shows engine status on the display again
- Refreshes the wiring and setup documentation for the current Lomax hardware
- Keeps the project in a PlatformIO / VS Code layout for easier maintenance

## Compatibility

- Speeduino secondary serial must be enabled in TunerStudio
- Secondary serial protocol must be set to `Generic (Fixed List)`
- Helper board target is an Arduino Mega 2560 / Mega Pro compatible board

## Notes

- This is marked as a pre-release because the project is now in a good shareable state, while still leaving room for future display and protocol refinements
- The tested firmware snapshot itself is the tagged commit `lomax-pre-release-20260411`
- The public default branch for the repository is `master`
