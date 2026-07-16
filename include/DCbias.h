// =============================================================================
//  DCBIAS MODULE — IsolatedControllerR1.0
//  Drives up to two real, external DCbias module cards plugged into this
//  board's EXT1/EXT2 "MIPS bus" (I2C mux channel PCA9540_CHAN_EXT + the SPI
//  bus carried on the same header), distinguished by BRDSEL (board 0 = A,
//  board 1 = B) — same convention as MIPS.
//
//  IMPORTANT: DCbiasData below is byte-identical to the real MIPS struct
//  (src/DCbias.h) — same field order, same types, ALL fields present even
//  though milestone 1 only acts on a subset. Cards get physically moved
//  between systems, so the EEPROM blob has to round-trip correctly
//  regardless of which fields this particular host uses. Total size is
//  320 bytes, matching the "Aug 14, 2024" MIPS comment — verified by hand
//  against ARM struct alignment before writing this.
//
//  DELIBERATELY NOT ACTED ON YET (per "no advanced features"): profiles,
//  pulse commands, waveform generation, list/segment/table mode, serial
//  calibration (CDCBCH etc.), and the ADC-change-detector/Level-Detect
//  hooks into the offset control (DCBCADCOF/DCBCADCCO/DCBCLDOF/DCBCLDCH).
//
//  IMPLEMENTED — the offset control command set (matches MIPS's real
//  argument conventions exactly):
//    SDCB/GDCB/GDCBV/SDCBOF/GDCBOF/SDCBOFFENA  — channel-indexed (channel
//      resolves to a board internally, same as MIPS's GetDCbiasDataPtr()).
//    SDCBOFOF/GDCBOFOF/SDCBCHOF/GDCBCHOF/SDCBCHMK/GDCBCHMK — board-indexed
//      directly.
//    SDCBONEOFF/DCBOFFRBENA — always act on board 0, matching MIPS (these
//      flags are conceptually shared across both boards even though stored
//      per-board-struct).
//
//  READBACK MONITOR — DCbias_init() starts a 100 ms thread (DCbias_loop,
//  same name and interval as MIPS's DCbiasThread) that reads each card's
//  AD7998 readback ADC (at DCbiasData.ADCadr), keeps a filtered readback
//  per channel (served by GDCBV/GDCBALLV), and computes the worst
//  setpoint-vs-readback error as a percentage of full scale. The filtered
//  error (StrongFilter, same coefficient as MIPS) trips the supply off
//  when it exceeds the STRPLVL/GTRPLVL threshold (% of FS, 0 disables —
//  MIPS default 1.0). SDCBTEST/GDCBTEST enable/disable the error test.
//  On trip (or SDCPWR,OFF) all channel DACs are driven to zero — the
//  board's real supply-enable wiring is still TBD (see NOTES.md).
// =============================================================================
#pragma once
#include <Arduino.h>
#include <Devices.h>          // DACchan / ADCchan, Value2Counts/Counts2Value
#include <commandProcessor.h> // CommandList

#define MAXDCBCHANNELS  8     // per board — matches the AD5668's 8 DAC channels
#define MAXDCBBOARDS    2     // A and B on this board's one external connector

// Exact match to MIPS's DCbiasChannellData (src/DCbias.h) — name kept as-is
// (including the "Channell" spelling) for easy cross-reference.
typedef struct
{
  float    VoltageSetpoint;
  DACchan  DCctrl;
  ADCchan  DCmon;
} DCbiasChannellData;

// Exact match to MIPS's DCbiasData (src/DCbias.h). Field order/types must
// not change — see the note above.
typedef struct
{
  int16_t Size;
  char    Name[20];
  int8_t  Rev;
  int8_t  NumChannels;
  float   MaxVoltage;
  float   MinVoltage;
  bool    Offsetable;
  int8_t  DACspi;
  uint8_t ADCadr;
  uint8_t DACadr;
  uint8_t EEPROMadr;
  DCbiasChannellData DCCD[MAXDCBCHANNELS];
  DCbiasChannellData DCoffset;
  bool    UseOneOffset;
  bool    OffsetReadback;
  float   OffsetOffset;
  uint8_t OffsetChanMsk;
  float   ChannelOffset;
  float   ADCgainOff;
  float   ADCgainCh;
  char    PolDIO;
} DCbiasData;

// One slot per physical board (A = 0, B = 1). DCbBoardPresent[] tracks
// which slots were actually found during DCbias_scan() — commands check
// this (via GetDCbiasDataPtr()) instead of assuming both are populated.
extern DCbiasData DCbDarray[MAXDCBBOARDS];
extern bool       DCbBoardPresent[MAXDCBBOARDS];

// -- Discovery / lifecycle ----------------------------------------------------
// Called from IsolatedController.cpp's setup(). Selects the external mux
// channel, then for each known module address toggles BRDSEL A and B
// (SelectBoard(), IsolatedController.h) and reads a signature block — same
// two-pass pattern MIPS.cpp's ScanHardware() uses to find up to 2 physical
// cards sharing one TWI address. Matches on the Name field (no separate
// magic-number signature in this struct). Boards that aren't found are
// simply left absent — commands NAK cleanly rather than touching hardware
// that isn't there.
void DCbias_scan(void);
void DCbias_init(int8_t board, uint8_t addr);

// Resolve a 1-based flat channel number (spanning both boards, up to
// MAXDCBBOARDS * MAXDCBCHANNELS) to a board index, mirroring MIPS's
// DCbiasCH2Brd(). Returns -1 if the channel is out of range or that
// board isn't present.
int DCbiasCH2Brd(int ch);

// Command table — registered by IsolatedController.cpp via
// cp.registerCommands(DCbias_commands()), same pattern as debug.h.
CommandList *DCbias_commands(void);

// -- AD5668 (8-channel SPI DAC) — minimal driver ------------------------------
// GAACE_Core's Devices.h doesn't have an AD5668 driver (only AD5592/AD5593/
// DAC8571/MCP4725). MIPS's own AD5668() is written against the Due's
// multi-chip-select SPI+DMA hardware and doesn't carry over. This is scoped
// to our single hardware CS pin (PA30, via csWrite()) — SelectBoard() must
// be called first so the right physical card is listening.
void AD5668write(int8_t chan, uint16_t val);
void AD5668enableInternalRef(void);

// -- AD7998 (8-channel I2C readback ADC on the DCbias card) -------------------
// Minimal port of MIPS's hardware-TWI AD7998 driver (src/Hardware.cpp,
// AD7998_b) minus the Due-specific AtomicBlock/TWI-queue plumbing. Values
// are left-justified to 16 bits, matching MIPS, so the same DCmon
// calibration constants work unchanged. SelectBoard() must be called first,
// same as the DAC. Single-channel read returns -1 on any bus error.
int AD7998read(uint8_t adr, int8_t chan);
int AD7998readAll(uint8_t adr, uint16_t *vals);
