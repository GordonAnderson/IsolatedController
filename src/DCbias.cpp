// =============================================================================
//  DCBIAS MODULE — IsolatedControllerR1.0
//  See DCbias.h for scope notes and the byte-compatibility requirement.
// =============================================================================
#include <SPI.h>
#include <Wire.h>
#include <string.h>
#include "DCbias.h"
#include "IsolatedController.h"   // pins, PCA9540 mux, csWrite(), ReadEEPROM, SelectBoard
#include <Errors.h>

extern commandProcessor cp;   // defined in IsolatedController.cpp

DCbiasData DCbDarray[MAXDCBBOARDS];
bool       DCbBoardPresent[MAXDCBBOARDS] = {false, false};

// Defaults used only if a card's EEPROM is unreadable/blank on first boot.
// Every field is populated even though several are inert this milestone —
// see the list in DCbias.h.
static DCbiasData DCbD_defaults =
{
  sizeof(DCbiasData),   // Size
  "DCbias",              // Name
  1,                     // Rev
  4,                     // NumChannels     — TODO: confirm real channel count
  100.0,                 // MaxVoltage      — TODO: confirm real range
  -100.0,                // MinVoltage
  false,                 // Offsetable      — TODO: confirm per real card
  0,                      // DACspi          — no per-slot SPI addressing needed
                          //                   (BRDSEL + single CS handle it)
  0x50,                   // ADCadr          — TODO: confirm real readback ADC address
  0x10,                   // DACadr          — TODO: confirm real offset-DAC address
  0,                      // EEPROMadr       — set from the scanned address in DCbias_init()
  {
    {0.0, {0,1.0,0.0}, {0,1.0,0.0}}, {0.0, {1,1.0,0.0}, {1,1.0,0.0}},
    {0.0, {2,1.0,0.0}, {2,1.0,0.0}}, {0.0, {3,1.0,0.0}, {3,1.0,0.0}},
    {0.0, {4,1.0,0.0}, {4,1.0,0.0}}, {0.0, {5,1.0,0.0}, {5,1.0,0.0}},
    {0.0, {6,1.0,0.0}, {6,1.0,0.0}}, {0.0, {7,1.0,0.0}, {7,1.0,0.0}},
  },
  {0.0, {0,1.0,0.0}, {0,1.0,0.0}},  // DCoffset
  false,                  // UseOneOffset
  false,                  // OffsetReadback
  0.0,                    // OffsetOffset
  0,                      // OffsetChanMsk
  0.0,                    // ChannelOffset
  1.0,                    // ADCgainOff
  1.0,                    // ADCgainCh
  0,                      // PolDIO
};

// =============================================================================
//  AD5668 — 8-channel SPI DAC, minimal driver
// =============================================================================
//  32-bit frame: [Cmd(4b) | Addr/Chan(4b) | Data(16b) | don't-care(4b)]
//  Cmd = 3 -> write to and update DAC channel (the common case).
//  Cmd = 8 -> internal reference setup (data = 1 to enable).
//  One hardware CS on this board (SWCLK/PA30, via csWrite()) shared by both
//  A and B — SelectBoard() (BRDSEL) is what determines which physical card
//  actually responds, so callers MUST select the right board first.
static void AD5668frame(uint8_t cmd, uint8_t chan, uint16_t val)
{
  uint8_t buf[4];
  buf[0] = cmd;
  buf[1] = (chan << 4) | (val >> 12);
  buf[2] = (val >> 4) & 0xFF;
  buf[3] = (val << 4) & 0xFF;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  csWrite(true);
  for (int i = 0; i < 4; i++) SPI.transfer(buf[i]);
  csWrite(false);
  SPI.endTransaction();
}

void AD5668write(int8_t chan, uint16_t val)
{
  AD5668frame(3, chan, val);
}

void AD5668enableInternalRef(void)
{
  AD5668frame(8, 0, 1);
}

// =============================================================================
//  Discovery
// =============================================================================
const uint8_t ModuleAddresses[NumModAdd] = {0x50, 0x52, 0x54, 0x56};

// Same check MIPS.cpp's ScanHardware() uses: the struct has no separate
// magic-number signature, so a Name-field match against the known module
// name IS the validity check.
static bool isDCbiasSignature(const char *name)
{
  return strncmp(name, "DCbias", sizeof("DCbias") - 1) == 0;
}

static void tryInitBoard(uint8_t addr)
{
  char signature[100];
  if (ReadEEPROM(signature, addr, 0, 100) != 0) return;
  if (!isDCbiasSignature(&signature[2])) return;   // offset 2 = Name field

  for (int b = 0; b < MAXDCBBOARDS; b++)
  {
    if (!DCbBoardPresent[b]) { DCbias_init(b, addr); return; }
  }
  // Both slots already filled — nothing more to do for milestone 1
  // (matches MIPS's own "Board += 2" overflow handling only up to the
  // slot count this board actually supports).
}

void DCbias_scan(void)
{
  pca9540SelectChannel(PCA9540_CHAN_EXT);

  for (int i = 0; i < NumModAdd; i++)
  {
    uint8_t addr = ModuleAddresses[i];

    SelectBoard(0);   // "A"
    tryInitBoard(addr);

    SelectBoard(1);   // "B"
    tryInitBoard(addr);
  }
  SelectBoard(0);   // leave BRDSEL in a known state
}

void DCbias_init(int8_t board, uint8_t addr)
{
  SelectBoard(board);

  DCbDarray[board] = DCbD_defaults;

  // Try to load the card's own saved configuration — full 320-byte struct,
  // byte-identical to what a real MIPS host would have written.
  DCbiasData fromCard;
  if (ReadEEPROM(&fromCard, addr, 0, sizeof(DCbiasData)) == 0 &&
      isDCbiasSignature(fromCard.Name))
  {
    DCbDarray[board] = fromCard;
  }
  DCbDarray[board].EEPROMadr = addr;

  AD5668enableInternalRef();
  for (int ch = 0; ch < DCbDarray[board].NumChannels; ch++)
  {
    AD5668write(DCbDarray[board].DCCD[ch].DCctrl.Chan,
                Value2Counts(0.0, &DCbDarray[board].DCCD[ch].DCctrl));
  }

  DCbBoardPresent[board] = true;
}

// =============================================================================
//  Channel <-> board resolution (mirrors MIPS's DCbiasCH2Brd/GetDCbiasBoard)
// =============================================================================
int DCbiasCH2Brd(int ch)
{
  if (ch < 1) return -1;
  int board = (ch - 1) / MAXDCBCHANNELS;
  if (board >= MAXDCBBOARDS) return -1;
  if (!DCbBoardPresent[board]) return -1;
  return board;
}

// =============================================================================
//  Command handlers (milestone 1 subset — matches MIPS host command names
//  and argument conventions exactly: SDCB/GDCB/GDCBV/SDCBOF/GDCBOF/
//  SDCBOFFENA are channel-indexed; SDCBOFOF/GDCBOFOF/SDCBCHOF/GDCBCHOF/
//  SDCBCHMK/GDCBCHMK are board-indexed; SDCBONEOFF/DCBOFFRBENA always act
//  on board 0.)
// =============================================================================

// -- Channel-indexed: SDCB / GDCB / GDCBV -------------------------------------
static void DCbiasSetCmd(void)
{
  int   ch, board;
  float value;
  if (!cp.getValue(&ch, 1, MAXDCBBOARDS * MAXDCBCHANNELS)) { cp.sendNAK(ERR_BADARG); return; }
  if ((board = DCbiasCH2Brd(ch)) < 0) { cp.sendNAK(ERR_INVALIDCHAN); return; }
  int localCh = (ch - 1) % MAXDCBCHANNELS;
  DCbiasData &d = DCbDarray[board];
  if (!cp.getValue(&value, d.MinVoltage, d.MaxVoltage)) { cp.sendNAK(ERR_BADARG); return; }

  d.DCCD[localCh].VoltageSetpoint = value;
  SelectBoard(board);
  AD5668write(d.DCCD[localCh].DCctrl.Chan, Value2Counts(value, &d.DCCD[localCh].DCctrl));
  cp.sendACK();
}

static void DCbiasGetCmd(void)
{
  int ch, board;
  if (!cp.getValue(&ch, 1, MAXDCBBOARDS * MAXDCBCHANNELS)) { cp.sendNAK(ERR_BADARG); return; }
  if ((board = DCbiasCH2Brd(ch)) < 0) { cp.sendNAK(ERR_INVALIDCHAN); return; }
  int localCh = (ch - 1) % MAXDCBCHANNELS;
  cp.sendACK(false);
  cp.println(DCbDarray[board].DCCD[localCh].VoltageSetpoint);
}

static void DCbiasGetVCmd(void)
{
  int ch, board;
  if (!cp.getValue(&ch, 1, MAXDCBBOARDS * MAXDCBCHANNELS)) { cp.sendNAK(ERR_BADARG); return; }
  if ((board = DCbiasCH2Brd(ch)) < 0) { cp.sendNAK(ERR_INVALIDCHAN); return; }
  int localCh = (ch - 1) % MAXDCBCHANNELS;

  // TODO: real ADC readback from DCbDarray[board].ADCadr once that chip is
  // confirmed, e.g.:
  //   SelectBoard(board);
  //   float v = AnalogIn(AD5593readADC, &DCbDarray[board].DCCD[localCh].DCmon, 4);
  float v = DCbDarray[board].DCCD[localCh].VoltageSetpoint;   // stand-in: echo setpoint

  cp.sendACK(false);
  cp.println(v);
}

// -- Offset control: channel-indexed (SDCBOF/GDCBOF/SDCBOFFENA) --------------
static void DCbiasSetOffsetCmd(void)   // SDCBOF, ch, value
{
  int   ch, board;
  float value;
  if (!cp.getValue(&ch, 1, MAXDCBBOARDS * MAXDCBCHANNELS)) { cp.sendNAK(ERR_BADARG); return; }
  if ((board = DCbiasCH2Brd(ch)) < 0) { cp.sendNAK(ERR_INVALIDCHAN); return; }
  DCbiasData &d = DCbDarray[board];
  if (!cp.getValue(&value, d.MinVoltage, d.MaxVoltage)) { cp.sendNAK(ERR_BADARG); return; }
  if (!d.Offsetable) { cp.sendNAK(ERR_NOTOFFSETABLE); return; }

  d.DCoffset.VoltageSetpoint = value;
  // MIPS also propagates to every board when UseOneOffset is set on board 0
  // — matching that here since it's a cheap, direct port:
  if (DCbDarray[0].UseOneOffset)
  {
    for (int b = 0; b < MAXDCBBOARDS; b++)
      if (DCbBoardPresent[b]) DCbDarray[b].DCoffset.VoltageSetpoint = value;
  }
  SelectBoard(board);
  AD5668write(d.DCoffset.DCctrl.Chan, Value2Counts(value, &d.DCoffset.DCctrl));
  cp.sendACK();
}

static void DCbiasGetOffsetCmd(void)   // GDCBOF, ch
{
  int ch, board;
  if (!cp.getValue(&ch, 1, MAXDCBBOARDS * MAXDCBCHANNELS)) { cp.sendNAK(ERR_BADARG); return; }
  if ((board = DCbiasCH2Brd(ch)) < 0) { cp.sendNAK(ERR_INVALIDCHAN); return; }
  cp.sendACK(false);
  cp.println(DCbDarray[board].DCoffset.VoltageSetpoint);
}

static void DCbiasOffsetEnableCmd(void)   // SDCBOFFENA, ch, TRUE|FALSE
{
  int  ch, board;
  char *state;
  if (!cp.getValue(&ch, 1, MAXDCBBOARDS * MAXDCBCHANNELS)) { cp.sendNAK(ERR_BADARG); return; }
  if ((board = DCbiasCH2Brd(ch)) < 0) { cp.sendNAK(ERR_INVALIDCHAN); return; }
  if (!cp.getValue(&state,"TRUE,FALSE")) { cp.sendNAK(ERR_BADARG); return; }
  if(strcmp(state,"TRUE") == 0) DCbDarray[board].Offsetable = true;
  else DCbDarray[board].Offsetable = false;
  cp.sendACK();
}

// -- Offset control: always board 0 (SDCBONEOFF/DCBOFFRBENA) ----------------
static void DCbiasUseOneOffsetCmd(void)   // SDCBONEOFF, TRUE|FALSE
{
  char *state;
  if (!DCbBoardPresent[0]) { cp.sendNAK(ERR_BRDLOWORBRD); return; }
  if (!cp.getValue(&state,"TRUE,FALSE")) { cp.sendNAK(ERR_BADARG); return; }
  if(strcmp(state,"TRUE") == 0) DCbDarray[0].UseOneOffset = true;
  else DCbDarray[0].UseOneOffset = false;
  cp.sendACK();
}

static void DCbiasOffsetReadbackCmd(void)   // DCBOFFRBENA, TRUE|FALSE
{
  char *state;
  if (!DCbBoardPresent[0]) { cp.sendNAK(ERR_BRDLOWORBRD); return; }
  if (!cp.getValue(&state,"TRUE,FALSE")) { cp.sendNAK(ERR_BADARG); return; }
  if(strcmp(state,"TRUE") == 0) DCbDarray[0].OffsetReadback = true;
  else DCbDarray[0].OffsetReadback = false;
  cp.sendACK();
}

// -- Offset control: board-indexed directly ----------------------------------
static bool checkBoard(int board)
{
  if (board < 0 || board >= MAXDCBBOARDS || !DCbBoardPresent[board])
  {
    cp.sendNAK(ERR_BRDHIORBRD);
    return false;
  }
  return true;
}

static void DCbiasSetOffOffCmd(void)   // SDCBOFOF, board, value
{
  int   board;
  float value;
  if (!cp.getValue(&board, 0, MAXDCBBOARDS - 1)) { cp.sendNAK(ERR_BADARG); return; }
  if (!checkBoard(board)) return;
  if (!cp.getValue(&value)) { cp.sendNAK(ERR_BADARG); return; }
  DCbDarray[board].OffsetOffset = value;
  // TODO: apply to the offset DAC output alongside DCoffset.VoltageSetpoint
  // once the combined offset math is worked out — echoed to the struct for
  // now so GDCBOFOF/round-tripping works.
  cp.sendACK();
}

static void DCbiasGetOffOffCmd(void)   // GDCBOFOF, board
{
  int board;
  if (!cp.getValue(&board, 0, MAXDCBBOARDS - 1)) { cp.sendNAK(ERR_BADARG); return; }
  if (!checkBoard(board)) return;
  cp.sendACK(false);
  cp.println(DCbDarray[board].OffsetOffset);
}

static void DCbiasSetCHOffCmd(void)   // SDCBCHOF, board, value
{
  int   board;
  float value;
  if (!cp.getValue(&board, 0, MAXDCBBOARDS - 1)) { cp.sendNAK(ERR_BADARG); return; }
  if (!checkBoard(board)) return;
  if (!cp.getValue(&value)) { cp.sendNAK(ERR_BADARG); return; }
  DCbDarray[board].ChannelOffset = value;
  // TODO: apply to channels enabled via OffsetChanMsk — see the TODO above.
  cp.sendACK();
}

static void DCbiasGetCHOffCmd(void)   // GDCBCHOF, board
{
  int board;
  if (!cp.getValue(&board, 0, MAXDCBBOARDS - 1)) { cp.sendNAK(ERR_BADARG); return; }
  if (!checkBoard(board)) return;
  cp.sendACK(false);
  cp.println(DCbDarray[board].ChannelOffset);
}

static void DCbiasSetCHMaskCmd(void)   // SDCBCHMK, board, hexmask
{
  int board, mask;
  if (!cp.getValue(&board, 0, MAXDCBBOARDS - 1)) { cp.sendNAK(ERR_BADARG); return; }
  if (!checkBoard(board)) return;
  if (!cp.getValue(&mask, 0, 0xFF, HEX)) { cp.sendNAK(ERR_BADARG); return; }
  DCbDarray[board].OffsetChanMsk = (uint8_t)mask;
  cp.sendACK();
}

static void DCbiasGetCHMaskCmd(void)   // GDCBCHMK, board
{
  int board;
  if (!cp.getValue(&board, 0, MAXDCBBOARDS - 1)) { cp.sendNAK(ERR_BADARG); return; }
  if (!checkBoard(board)) return;
  cp.sendACK(false);
  cp.println(DCbDarray[board].OffsetChanMsk, HEX);
}

// -- Power (no dedicated field in DCbiasData — see NOTES.md) -----------------
static void DCbiasPowerSetCmd(void)
{
  char *state;
  if (!cp.getValue(&state,"TRUE,FALSE")) { cp.sendNAK(ERR_BADARG); return; }
  static bool powerEnable = false;   // TODO: confirm how power enable should
  if(strcmp(state,"TRUE") == 0) powerEnable = true;               //       actually be wired on this board.
  else powerEnable = false;
  cp.sendACK();
}

static void DCbiasPowerGetCmd(void)
{
  cp.sendACK(false);
  cp.println("FALSE");   // TODO: wire to the real power-enable state
}

// -- Bulk commands (channel-indexed, spans whichever boards are present) ----
static void DCbiasSetAllCmd(void)   // SDCBALL, v1, v2, ...
{
  int totalCh = 0;
  for (int b = 0; b < MAXDCBBOARDS; b++) if (DCbBoardPresent[b]) totalCh += DCbDarray[b].NumChannels;
  if (totalCh == 0) { cp.sendNAK(ERR_BRDLOWORBRD); return; }

  float value;
  for (int ch = 1; ch <= totalCh; ch++)
  {
    int board = DCbiasCH2Brd(ch);
    if (board < 0) { cp.sendNAK(ERR_INVALIDCHAN); return; }
    int localCh = (ch - 1) % MAXDCBCHANNELS;
    DCbiasData &d = DCbDarray[board];
    if (!cp.getValue(&value, d.MinVoltage, d.MaxVoltage)) { cp.sendNAK(ERR_BADARG); return; }
    d.DCCD[localCh].VoltageSetpoint = value;
    SelectBoard(board);
    AD5668write(d.DCCD[localCh].DCctrl.Chan, Value2Counts(value, &d.DCCD[localCh].DCctrl));
  }
  cp.sendACK();
}

static void DCbiasReportAllSetpointsCmd(void)   // GDCBALL
{
  int totalCh = 0;
  for (int b = 0; b < MAXDCBBOARDS; b++) if (DCbBoardPresent[b]) totalCh += DCbDarray[b].NumChannels;
  cp.sendACK(false);
  for (int ch = 1; ch <= totalCh; ch++)
  {
    int board = DCbiasCH2Brd(ch);
    int localCh = (ch - 1) % MAXDCBCHANNELS;
    cp.print(DCbDarray[board].DCCD[localCh].VoltageSetpoint);
    if (ch < totalCh) cp.print(",");
  }
  cp.println("");
}

static void DCbiasReportAllValuesCmd(void)   // GDCBALLV
{
  int totalCh = 0;
  for (int b = 0; b < MAXDCBBOARDS; b++) if (DCbBoardPresent[b]) totalCh += DCbDarray[b].NumChannels;
  cp.sendACK(false);
  for (int ch = 1; ch <= totalCh; ch++)
  {
    int board = DCbiasCH2Brd(ch);
    int localCh = (ch - 1) % MAXDCBCHANNELS;
    // TODO: real ADC readback — see DCbiasGetVCmd() above.
    cp.print(DCbDarray[board].DCCD[localCh].VoltageSetpoint);
    if (ch < totalCh) cp.print(",");
  }
  cp.println("");
}

static void DCbiasSetNumChannelsCmd(void)   // SDCBCHNS, board, num — setup only
{
  int board, num;
  if (!cp.getValue(&board, 0, MAXDCBBOARDS - 1)) { cp.sendNAK(ERR_BADARG); return; }
  if (!checkBoard(board)) return;
  if (!cp.getValue(&num, 1, MAXDCBCHANNELS)) { cp.sendNAK(ERR_BADARG); return; }
  DCbDarray[board].NumChannels = num;
  cp.sendACK();
}

// =============================================================================
//  Command table
// =============================================================================
static Command DCbiasCmds[] =
{
  {"SDCB",       CMDfunction, 2, (void *)DCbiasSetCmd,             NULL, "Set DC bias channel setpoint: ch, volts"},
  {"GDCB",       CMDfunction, 1, (void *)DCbiasGetCmd,              NULL, "Get DC bias channel setpoint: ch"},
  {"GDCBV",      CMDfunction, 1, (void *)DCbiasGetVCmd,              NULL, "Get DC bias channel readback voltage: ch"},

  {"SDCBOF",     CMDfunction, 2, (void *)DCbiasSetOffsetCmd,        NULL, "Set DC bias channel offset voltage: ch, volts"},
  {"GDCBOF",     CMDfunction, 1, (void *)DCbiasGetOffsetCmd,         NULL, "Get DC bias channel offset voltage: ch"},
  {"SDCBOFFENA", CMDfunction, 2, (void *)DCbiasOffsetEnableCmd,      NULL, "Set the channel's board offsetable flag: ch, TRUE|FALSE"},
  {"SDCBONEOFF", CMDfunction, 1, (void *)DCbiasUseOneOffsetCmd,      NULL, "Use one offset for both boards: TRUE|FALSE"},
  {"DCBOFFRBENA",CMDfunction, 1, (void *)DCbiasOffsetReadbackCmd,    NULL, "Enable offset readback: TRUE|FALSE"},
  {"SDCBOFOF",   CMDfunction, 2, (void *)DCbiasSetOffOffCmd,         NULL, "Set the board's global offset: board, volts"},
  {"GDCBOFOF",   CMDfunction, 1, (void *)DCbiasGetOffOffCmd,         NULL, "Get the board's global offset: board"},
  {"SDCBCHOF",   CMDfunction, 2, (void *)DCbiasSetCHOffCmd,          NULL, "Set the board's channel offset: board, volts"},
  {"GDCBCHOF",   CMDfunction, 1, (void *)DCbiasGetCHOffCmd,          NULL, "Get the board's channel offset: board"},
  {"SDCBCHMK",   CMDfunction, 2, (void *)DCbiasSetCHMaskCmd,         NULL, "Set the board's channel offset mask (hex): board, mask"},
  {"GDCBCHMK",   CMDfunction, 1, (void *)DCbiasGetCHMaskCmd,         NULL, "Get the board's channel offset mask (hex): board"},

  {"SDCPWR",     CMDfunction, 1, (void *)DCbiasPowerSetCmd,          NULL, "Set DC bias power, TRUE or FALSE"},
  {"GDCPWR",     CMDfunction, 0, (void *)DCbiasPowerGetCmd,          NULL, "Get DC bias power state"},

  {"SDCBALL",    CMDfunction, -1,(void *)DCbiasSetAllCmd,            NULL, "Set all DC bias channel setpoints: v1,v2,..."},
  {"GDCBALL",    CMDfunction, 0, (void *)DCbiasReportAllSetpointsCmd,NULL, "Report all DC bias setpoints"},
  {"GDCBALLV",   CMDfunction, 0, (void *)DCbiasReportAllValuesCmd,   NULL, "Report all DC bias readback values"},
  {"SDCBCHNS",   CMDfunction, 2, (void *)DCbiasSetNumChannelsCmd,    NULL, "Set number of active DC bias channels (setup only): board, num"},
  {NULL}
};
static CommandList DCbiasCmdList = {DCbiasCmds, NULL};

CommandList *DCbias_commands(void)
{
  return &DCbiasCmdList;
}
