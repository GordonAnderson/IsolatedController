# IsolatedControllerR1.0

Firmware for the IsolatedControllerR1.0 board, developed by GAA Custom
Electronics, LLC. It runs on an Adafruit ItsyBitsy M4 Express (Atmel
SAMD51, 120MHz ARM Cortex-M4) and is built with
[PlatformIO](https://platformio.org/).

## Overview

IsolatedControllerR1.0 is a satellite MIPS controller with two roles:

- It hosts a local "MIPS bus" (I2C + SPI on the EXT1/EXT2 headers) that up
  to two real, physical module cards can plug into, selected via `BRDSEL`
  the same way a full MIPS mainframe addresses stacked A/B boards.
- It bridges to a separate, full MIPS controller (the one with the display
  and front-panel buttons) over an isolated RS232 link plus two trigger
  lines, via four fiber-optic isolators. This board has no display or user
  controls of its own.

Firmware is built on [GAACE_Core](https://github.com/GordonAnderson/GAACE_Core)
(commandProcessor, threads, persistence, hardware drivers) and follows the
command protocol of the main [MIPS](https://github.com/GordonAnderson/MIPS)
firmware, so module cards and host software are compatible across both
systems.

## Hardware

- **Controller:** Adafruit ItsyBitsy M4 Express (ATSAMD51G19A, 120MHz ARM Cortex-M4)
- **Isolation:** 4x Broadcom AFBR-395025RZ fiber-optic isolators (RS232 RX/TX + 2 trigger lines)
- **I2C:** PCA9540 2-channel mux, PCA9517D buffer, 2x AD5593R (onboard DIO + analog I/O)
- **Communication:** USB (native serial), isolated RS232 to the full MIPS controller, I2C/SPI "MIPS bus" (EXT1/EXT2) for external module cards
- **Module bus:** TWI + SPI with `BRDSEL` board-select addressing (up to 2 external cards per connector)

## Supported Modules

| Module | Description | Status |
|---|---|---|
| **DCbias** | Multi-channel DC bias supply control, including offset control | In progress — core + offset commands implemented, ADC readback and combined-offset hardware application still open |
| **DIO** | Digital I/O (onboard AD5593R) | Not yet implemented |
| **Analog** | Analog I/O (onboard AD5593R) | Not yet implemented |

See [NOTES.md](NOTES.md) for the current architecture, hardware findings,
and open questions in detail.

## Build

Requires [PlatformIO](https://platformio.org/).

```bash
pio run
```

To upload:

```bash
pio run --target upload
```

## Project Structure

```
├── src/            # Firmware source files (.cpp)
├── include/        # Project header files (.h)
├── platformio.ini
├── NOTES.md         # Architecture decisions, hardware findings, open questions
└── CLAUDE.md        # Project context for AI-assisted development sessions
```

## Documentation

- [NOTES.md](NOTES.md) — architecture decisions, the DCbias EEPROM
  byte-compatibility requirement, pin table, and open questions.
- [CLAUDE.md](CLAUDE.md) — condensed project context for picking up
  development in a new session.

## Author

Gordon Anderson
GAA Custom Electronics, LLC
