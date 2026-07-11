# CLAUDE.md — IsolatedControllerR1.0 Firmware

Context file for Claude sessions working on this project. Read this first,
then read the actual files in the repo before changing anything — this
project has been fixed up by hand outside of chat sessions, so treat the
files on disk as the source of truth over anything summarized here.

## What this board is

GAA Custom Electronics **IsolatedControllerR1.0** — a satellite firmware
module built on an Adafruit ItsyBitsy M4 Express (ATSAMD51G19A, 120 MHz).
It plays two roles at once:

1. **Host for an external "MIPS bus"** (the EXT1/EXT2 headers) — carries
   both I2C and SPI, and up to two real, physical DCbias module cards can
   be plugged into it, distinguished by `BRDSEL` (board A / board B), the
   same convention the full MIPS mainframe uses.
2. **Isolated bridge to a separate, full MIPS controller** (the one with
   the display and buttons) over RS232 + 2 trigger lines, via four
   AFBR-395025RZ fiber-optic isolators. This board has no display and no
   user buttons of its own.

Two onboard AD5593R chips (IC4/IC5) implement local **DIO** and **Analog**
I/O — functionality that used to live on the MIPS mainframe itself. They
are **not** DCbias hardware and are **not implemented yet** in this
firmware (deferred past milestone 1).

## Software architecture

Built on Gordon's own **GAACE_Core** library
(`https://github.com/GordonAnderson/GAACE_Core`) plus `ivanseidel/ArduinoThread`
and `cmaglie/FlashStorage`. The real **MIPS** firmware
(`https://github.com/GordonAnderson/MIPS`) is the reference implementation
for command names, argument conventions, and struct layouts — when in
doubt about a command's behavior, that repo is the authority, not this
one.

**Module pattern:** each functional module owns its own `Command[]` table
and exposes a `<Module>_commands()` accessor (mirrors GAACE_Core's own
`debug.h`), rather than one central command table like MIPS's
`Serial.cpp`. `IsolatedController.cpp` is the thin host: framework setup,
the three I/O streams, `SelectBoard()`/`SelectedBoard()` (BRDSEL), and the
EEPROM read/write helpers used for module discovery. It calls each
module's own `_scan()`/`_init()` — there is deliberately no central
`ScanHardware()` dispatcher. That's fine with one discoverable module type
(DCbias); if a second EEPROM-discoverable external module type gets added,
revisit whether a shared single-read dispatch is worth it (currently each
module would re-read the same signature independently).

**Files:**
- `platformio.ini` — board `adafruit_itsybitsy_m4`, platform `atmelsam`.
- `include/IsolatedController.h` / `src/IsolatedController.cpp` — host:
  pin definitions, PCA9540 mux select, BRDSEL/`SelectBoard()`, CS
  (SWCLK/PA30) direct-PORT helpers, `ReadEEPROM()`/`WriteEEPROM()`,
  commandProcessor setup, the three streams (`Serial`, `Serial1`,
  `wh.sb`), thread scheduler.
- `include/DCbias.h` / `src/DCbias.cpp` — the DCbias module: discovery,
  `DCbDarray[2]` state, command table, minimal AD5668 SPI DAC driver
  (GAACE_Core has no AD5668 driver — MIPS's own is written against the
  Due's multi-CS SPI/DMA hardware and doesn't port over).

## Three I2C/bus channels — don't mix these up

1. **`PCA9540_CHAN_LOCAL`** — the two onboard AD5593Rs (DIO + Analog, not
   yet implemented).
2. **`PCA9540_CHAN_EXT`** — buffered (via IC3/PCA9517D) out to the
   EXT1/EXT2 "MIPS bus" headers, where external DCbias cards attach. Also
   carries the SPI bus (MOSI/MISO/SCK) those cards' AD5668 DACs use.
3. **`Serial1`** (hardware UART, not I2C) — the isolated RS232 link to the
   separate full MIPS controller, via OPTO1/OPTO2. Registered as a normal
   commandProcessor stream, same ASCII protocol as USB serial.

IC1 (PCA9540) is an exclusive 2-channel mux (fixed address 0x70, no A0
pin) — only one channel is live at a time, and it **powers up with
neither channel selected**. Always select before talking to either side.

## BRDSEL / board selection

`SelectBoard(board)` / `SelectedBoard()` live in `IsolatedController.h`/
`.cpp` since any module using the external bus needs the same
board-select state. Convention matches MIPS exactly: even board number →
A (`BRDSEL` high), odd → B (low). Skips redundant `digitalWrite()` calls
via a static tracker, same optimization MIPS's own implementation makes.
`DCbias_scan()` toggles both states at each of the four known module
addresses (`0x50, 0x52, 0x54, 0x56`) looking for a `"DCbias"` signature.

## DCbiasData — byte-compatibility is a hard requirement

Physical DCbias module cards move between systems (this board and real
MIPS mainframes), so `DCbiasData` in `DCbias.h` must stay **byte-identical**
to the real MIPS struct (`MIPS/src/DCbias.h`) — same field order, types,
and names, including fields this milestone doesn't act on
(`Offsetable`, `DACspi`, `DCoffset`, `UseOneOffset`, `OffsetReadback`,
`OffsetOffset`, `OffsetChanMsk`, `ChannelOffset`, `ADCgainOff`,
`ADCgainCh`, `PolDIO`). Verified by hand against ARM struct alignment:
the field list totals exactly **320 bytes**, matching MIPS's own "Aug 14,
2024" size comment. **Do not reorder, resize, rename, or drop fields** —
if the real MIPS struct ever changes, mirror the change here exactly.

There is no separate magic-number signature — validity is checked by
matching the `Name` field against `"DCbias"`, exactly like MIPS's
`ScanHardware()`. The struct also has **no power-enable field** — MIPS
handles that elsewhere (a relay/enable line outside `DCbiasData`);
`SDCPWR`/`GDCPWR` here are currently a local-static stub, not persisted.

## Commands implemented (milestone 1)

Host-level: `GVER`, `?NAME`, `SAVE`, `RESTORE`, `BLOAD`, `STWIADD`, `GTWIADD`.

DCbias (argument conventions pulled directly from MIPS's `Serial.cpp`/
`DCbias.cpp` — don't guess these, they're load-bearing for host
compatibility):
- **Channel-indexed** (channel resolves to a board internally):
  `SDCB`, `GDCB`, `GDCBV`, `SDCBOF`, `GDCBOF`, `SDCBOFFENA`.
- **Board-indexed directly**: `SDCBOFOF`, `GDCBOFOF`, `SDCBCHOF`,
  `GDCBCHOF`, `SDCBCHMK`, `GDCBCHMK`.
- **Always board 0**: `SDCBONEOFF`, `DCBOFFRBENA`.
- **Bulk / setup**: `SDCBALL`, `GDCBALL`, `GDCBALLV`, `SDCBCHNS`.
- `SDCPWR`/`GDCPWR` — stub, see note above.

## Explicitly deferred (not in scope yet — don't add without asking)

- Onboard AD5593-based DIO and Analog modules.
- DCbias profiles, pulse commands, waveform generation, list/segment/table
  mode, serial calibration (`CDCBCH` etc.).
- ADC-change-detector / Level-Detect hooks into offset control
  (`DCBCADCOF`/`DCBCADCCO`/`DCBCLDOF`/`DCBCLDCH`).
- `UseSPIflash` is off — GAACE_Core's `FlashFS` is excluded from its own
  build by default (needs `Adafruit_SPIFlash` + `SdFat - Adafruit Fork`,
  and per its README, a consuming project has to re-add it via its own
  `lib_extra_dirs`/`build_src_filter`). Not resolved — internal-flash
  `SAVE`/`RESTORE` works fine without it.

## Open questions (real gaps, not just style choices)

1. External card's ADC readback chip and TWI address — `GDCBV`/`GDCBALLV`
   currently echo the setpoint instead of reading real hardware.
2. `NumChannels` / voltage range defaults are placeholders (4 ch, ±100V).
3. Combined offset math (`OffsetOffset` + `ChannelOffset` + mask) is
   stored/round-trips but not yet applied to the DAC output.
4. CS polarity assumed active-low.
5. `PCA9540_CHAN_LOCAL`/`PCA9540_CHAN_EXT` = 0/1 is a placeholder pairing
   — confirm against the schematic.
6. `GRDSTA`/`GRDPWR`/`INT4`–`INT7` pin direction — all set `INPUT` in
   `setup()`, unconfirmed.
7. How power-enable should actually be wired on this board, given
   `DCbiasData` has no field for it.
8. `Serial1` baud rate to the full MIPS controller — assumed 115200,
   unconfirmed.

## Hardware reference (from the real schematic/PCB, already verified)

Pin table pulled from `.kicad_pcb` pad `pinfunction` properties (not
inferred from footprint geometry):

| ItsyBitsy pin | Net | Notes |
|---|---|---|
| RX (D0) / TX (D1) | RX / TX | RS232 to/from full MIPS controller, via OPTO1/OPTO2 |
| SDA / SCL | → IC1 (mux) | |
| MOSI / MISO / SCK | → EXT2, external card's AD5668 | |
| D5 | LDAC | |
| D10 / D12 | PWMRFCH1 / PWMRFCH2 | independent TCC instances |
| A0 | BRDSEL | **output** |
| A1 / A2 / A3 | GRDSTA / GRDPWR / INT6 | |
| A4 / A5 / D2 | ADDR0 / ADDR1 / ADDR2 | |
| D3 | TRIG_IN | isolated, via OPTO3 |
| D4 | EXT2-14 | plain digital, not PWM |
| D7 / D9 / D11 | INT4 / INT7 / INT5 | |
| D13 | TRIG_OUT | isolated, via OPTO4 |
| SWCLK (**PA30**) | CS | **not in the Arduino pin array** — direct `PORT->Group[PORTA]` register access only (`csPinMode()`/`csWrite()`), no SWD debugging available on this board |
| BAT | 5V | |

## Build

`pio run` from the project root. If something doesn't build, check
`platformio.ini` for the current `lib_deps` first — it may have changed
since this file was written.
