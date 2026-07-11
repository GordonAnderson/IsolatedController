# IsolatedControllerR1.0 firmware — project notes

## Architecture (revised — module split, BRDSEL A/B)

`IsolatedController.cpp` plays the role `MIPS.cpp` plays on a full MIPS
mainframe — it owns the framework (commandProcessor, threads, persistence,
the three I/O streams, `SelectBoard()`/`SelectedBoard()`) and hosts
whatever's attached to its own module bus.

- **Onboard AD5593s (IC4/IC5)** — on I2C mux channel `PCA9540_CHAN_LOCAL`.
  Implement the DIO and Analog modules that used to live on the MIPS
  mainframe. **Not implemented yet** — deferred, per "DCbias first."
- **EXT1/EXT2 "MIPS bus"** — on I2C mux channel `PCA9540_CHAN_EXT`
  (buffered through IC3), plus the SPI bus (MOSI/MISO/SCK/CS). Up to
  **two** real external DCbias cards can share this one connector,
  distinguished by `BRDSEL` (board 0 = A, board 1 = B — same convention as
  MIPS). `DCbias_scan()` toggles both states at each of the 4 known module
  addresses, exactly like `MIPS.cpp`'s `ScanHardware()`.
- **DCbias.cpp/.h** — owns its own `Command[]` table and its own
  `DCbiasData DCbDarray[2]`.

## `DCbias_scan()` vs. a central `ScanHardware()`

Kept it as a function DCbias.cpp owns, not a central host function — a few
reasons, but also a real tradeoff worth knowing about:

- It matches the "each module owns its command table" decision already
  made (mirrors `debug.h`) — a module is self-contained: its data, its
  commands, and now its own discovery.
- Adding a new EEPROM-discoverable module later just means writing
  `NewModule_scan()` and adding one call in `setup()` — no shared
  dispatch function to edit.
- **The real cost**: if more than one module type ever needs to be
  discovered this way, each one re-reads the same 100-byte signature at
  the same addresses independently — N reads instead of 1. MIPS's actual
  `ScanHardware()` avoids that by reading each address once and
  dispatching by name.

For right now, DCbias is the only external module type this board
discovers, so the distinction doesn't cost anything. If a second
EEPROM-discoverable external module type gets added later, it'd be worth
revisiting — a thin host-level function that does the generic "read
signature at address+A/B" once and dispatches to whichever module's name
matches, while each module still owns its own `_init()`/`_commands()`.
Flagging it now rather than silently deciding either way.

## Milestone 1 scope (explicitly deferred)

Profiles, pulse commands, waveform generation, list/segment/table mode,
serial calibration (`CDCBCH` etc.), and the ADC-change-detector/Level-Detect
hooks into offset control (`DCBCADCOF`/`DCBCADCCO`/`DCBCLDOF`/`DCBCLDCH`).

**Implemented now:** the full offset control command set — `SDCB`/`GDCB`/
`GDCBV`, `SDCBOF`/`GDCBOF`/`SDCBOFFENA` (channel-indexed), `SDCBOFOF`/
`GDCBOFOF`/`SDCBCHOF`/`GDCBCHOF`/`SDCBCHMK`/`GDCBCHMK` (board-indexed),
`SDCBONEOFF`/`DCBOFFRBENA` (always board 0) — argument conventions pulled
directly from the real `Serial.cpp`/`DCbias.cpp`. The combined-offset math
(`OffsetOffset` + `ChannelOffset` + mask actually adjusting DAC output) is
stored/round-trips but not yet applied to hardware — see the TODOs in
`DCbiasSetOffOffCmd()`/`DCbiasSetCHOffCmd()`.

## EEPROM byte-compatibility (hard requirement)

Module cards move between systems, so `DCbiasData` in `DCbias.h` is
byte-identical to the real MIPS struct (`src/DCbias.h`) — same field order,
types, and names, including fields this milestone doesn't act on yet
(`Offsetable`, `DACspi`, `DCoffset`, `UseOneOffset`, `OffsetReadback`,
`OffsetOffset`, `OffsetChanMsk`, `ChannelOffset`, `ADCgainOff`,
`ADCgainCh`, `PolDIO`). Verified by hand against ARM struct alignment
rules (both the Due and SAMD51 compile under arm-none-eabi-gcc via
PlatformIO, so padding matches): the field list works out to exactly
**320 bytes**, matching MIPS's own "Aug 14, 2024" size comment. There's no
separate magic-number signature in this struct — `DCbias_scan()`/
`DCbias_init()` validate a read the same way `MIPS.cpp`'s
`ScanHardware()` does, by checking the `Name` field against `"DCbias"`.

Note: the real struct has **no dedicated power-enable field** — MIPS
handles that elsewhere (a relay/enable line outside `DCbiasData`).
`SDCPWR`/`GDCPWR` currently just hold a local static, not persisted — see
the TODO in `DCbiasPowerSetCmd()`.

## Key finding from the real MIPS DCbias.cpp (worth remembering)

MIPS's AD5593-based DCbias board (rev 4.0, "50V bias board") uses its
AD5593 **only** for the offset DAC/readback and supply-voltage monitoring
(fixed channel roles: CH0=offset DAC, CH1=offset readback, CH2/CH3=+/-HV
monitor, CH4=3.3V monitor) — the actual per-channel bias voltages come from
a separate 8-channel SPI DAC, the **AD5668**. That's why this board's own
onboard AD5593s (IC4/IC5) were never meant to *be* DCbias channels — they're
DIO/Analog, matching what Gordon confirmed. The external DCbias card(s)
this board discovers over EXT1/EXT2 have their own AD5668 (driven via the
shared SPI bus + BRDSEL) and, likely, their own AD5593 for offset+
monitoring (not yet wired up — see TODOs in `DCbiasGetVCmd()`).

**GAACE_Core gap:** `Devices.h` has AD5592/AD5593/DAC8571/MCP4725 drivers
but no AD5668. MIPS's own `AD5668()` is written against the Due's
multi-chip-select SPI+DMA hardware and doesn't port over. `DCbias.cpp`
includes a minimal from-scratch AD5668 driver (`AD5668write`/
`AD5668enableInternalRef`) scoped to this board's single hardware CS pin —
`SelectBoard()` (BRDSEL) is what determines which physical card actually
responds.

## Open questions

1. **Card's ADC readback chip/address** — `DCbiasGetVCmd()`/
   `DCbiasReportAllValuesCmd()` currently echo the setpoint instead of
   reading real hardware. Need to know whether the external card's
   readback is AD7998 (TWI) or an AD5593 (like the rev-4 MIPS board), and
   its address.
2. **`NumChannels` / voltage range defaults** — placeholders (4 ch, ±100V)
   in `DCbD_defaults`.
3. **Combined offset math** — `OffsetOffset`/`ChannelOffset`/
   `OffsetChanMsk` are stored but not yet applied to the DAC output
   alongside the per-channel setpoint.
4. **CS polarity** — assumed active-low in `csWrite()`.
5. **`PCA9540_CHAN_LOCAL`/`PCA9540_CHAN_EXT` numbering** — placeholder
   0/1, confirm against schematic (doesn't affect function, just which
   constant is which).
6. **GRDSTA/GRDPWR/INT4-7 pin direction** — still all `INPUT` pending
   confirmation.

## Pin table (verified from `.kicad_pcb` pad `pinfunction` properties)

| ItsyBitsy pin | Net        | Notes |
|---|---|---|
| RX (D0)  | RX          | RS232 from full MIPS controller, via OPTO1 |
| TX (D1)  | TX          | RS232 to full MIPS controller, via OPTO2 |
| SDA/SCL  | → IC1 (mux) | Hardware I2C |
| MOSI/MISO/SCK | MOSI/MISO/SCLK | External DCbias card's AD5668, EXT2 |
| D5       | LDAC        | TCC compare-match capable |
| D10      | PWMRFCH1    | Independent TCC instance |
| D12      | PWMRFCH2    | Independent TCC instance |
| A0       | BRDSEL      | **Output** — this board drives it |
| A1       | GRDSTA      | |
| A2       | GRDPWR      | |
| A3       | INT6        | |
| A4       | ADDR0       | |
| A5       | ADDR1       | |
| D2       | ADDR2       | |
| D3       | TRIG_IN     | isolated, external → this board, via OPTO3 |
| D4       | EXT2-14     | plain digital, not PWM |
| D7       | INT4        | |
| D9       | INT7        | |
| D11      | INT5        | |
| D13      | TRIG_OUT    | isolated, this board → external, via OPTO4 |
| SWCLK (PA30) | CS      | **not in the Arduino pin array** — direct PORT register access only |
| GND      | GND         | |
| BAT      | 5V          | |

## I2C topology

- IC1 (PCA9540, fixed addr 0x70) — 2-channel mux, exclusive, powers up with
  **no channel selected**. Channel `PCA9540_CHAN_LOCAL` → IC4+IC5 (onboard
  AD5593s, DIO+Analog). Channel `PCA9540_CHAN_EXT` → IC3 (PCA9517D buffer,
  transparent) → EXT1/EXT2 headers → external DCbias card(s), selected via
  `SelectBoard()`/`BRDSEL`.
