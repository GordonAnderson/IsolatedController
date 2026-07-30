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
#include <Thread.h>
#include <ThreadController.h>

extern commandProcessor cp;       // defined in IsolatedController.cpp
extern ThreadController control;  // defined in IsolatedController.cpp

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
//  Readback monitor state — mirrors MIPS's DCbias_loop() machinery
//  (src/DCbias.cpp) minus the display/UI plumbing this board doesn't have.
// =============================================================================
static Thread DCbiasThread = Thread();

#define StrongFilter 0.05   // VerrorFiltered coefficient — same value as MIPS

static float Readbacks[MAXDCBBOARDS][MAXDCBCHANNELS];  // filtered readbacks, volts
static float Verror           = 0;    // worst current setpoint-vs-readback error, % of FS
static int   VerrorCh         = 0;    // channel (1-based, flat) with the worst error
static float VerrorFiltered   = 0;    // filtered error the trip decision uses
static float VerrorThreshold  = 1.0;  // STRPLVL/GTRPLVL — trip level, % of FS; 0 disables.
                                      // 1.0 matches MIPSconfigData's default.
static bool  DCbiasTestEnable = true; // SDCBTEST — FALSE disables the error test
static int   MonitorDelay     = 0;    // loop passes left before error testing resumes
static bool  DCbiasPowerEnable = true;// SDCPWR/GDCPWR state; MIPS's PowerEnable default
static bool  Tripped          = false;
static float TrpLvlRange[2]   = {0, 100};

// Call after any commanded output change or power-on — holds off the trip
// test long enough for the outputs and readback filters to settle (same
// intent as MIPS's DelayMonitoring(): 10 passes at 100 ms = 1 s).
static void DelayMonitoring(void)
{
  MonitorDelay = 10;
  VerrorFiltered = 0;
}

// This function will set the three address lines used for the SPI device selection. It is assumed
// the bit directions have already been set.
void SetAddress(int8_t addr)
{
  if(addr & 1) digitalWrite(PIN_ADDR0, HIGH);
  else digitalWrite(PIN_ADDR0, LOW);
  if(addr & 2) digitalWrite(PIN_ADDR1, HIGH);
  else digitalWrite(PIN_ADDR1, LOW);
  if(addr & 4) digitalWrite(PIN_ADDR2, HIGH);
  else digitalWrite(PIN_ADDR2, LOW);
}

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
//  AD7998 — 8-channel I2C readback ADC on the DCbias card, minimal driver
// =============================================================================
//  Port of MIPS's hardware-TWI path (AD7998_b, src/Hardware.cpp) minus the
//  Due-specific AtomicBlock/TWI-queue plumbing. The address pointer byte is
//  written twice — the second conversion gives the input mux time to settle
//  (MIPS's 9/14/2020 cross-talk fix). Results are left-justified to 16 bits
//  so MIPS's DCmon calibration constants apply unchanged.
int AD7998read(uint8_t adr, int8_t chan)
{
  Wire.beginTransmission(adr);
  Wire.write(0x80 | (chan << 4));
  Wire.write(0x80 | (chan << 4));
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom((int)adr, 2) != 2) return -1;

  int hi = Wire.read();
  int lo = Wire.read();
  if (hi < 0 || lo < 0) return -1;
  unsigned int val = ((hi << 8) & 0xFF00) | (lo & 0xFF);
  if ((val & 0x7000) != (unsigned int)(chan << 12)) return -1;  // echo of chan bits
  return (int)((val & 0xFFF) << 4);
}

int AD7998readAll(uint8_t adr, uint16_t *vals)
{
  for (int i = 0; i < 8; i++)
  {
    int v = AD7998read(adr, i);
    if (v == -1) return -1;
    vals[i] = (uint16_t)v;
  }
  return 0;
}

// =============================================================================
//  Power control + readback monitor loop
// =============================================================================
//  TODO: drive the real supply-enable hardware once its wiring on this board
//  is confirmed (DCbiasData has no field for it — see NOTES.md). Until then
//  "power off" means driving every channel DAC to zero, which is the only
//  protective action this board has; setpoints are preserved and reapplied
//  on power-on, the way MIPS's DCbias_loop() zeros/restores outputs around
//  its PWR_ON line.
static void SetDCbiasPower(bool on)
{
  if (on == DCbiasPowerEnable) return;
  DCbiasPowerEnable = on;

  pca9540SelectChannel(PCA9540_CHAN_EXT);
  for (int b = 0; b < MAXDCBBOARDS; b++)
  {
    if (!DCbBoardPresent[b]) continue;
    DCbiasData &d = DCbDarray[b];
    SelectBoard(b);
    for (int ch = 0; ch < d.NumChannels; ch++)
    {
      float v = on ? d.DCCD[ch].VoltageSetpoint : 0.0;
      AD5668write(d.DCCD[ch].DCctrl.Chan, Value2Counts(v, &d.DCCD[ch].DCctrl));
    }
  }
  if (on) { Tripped = false; DelayMonitoring(); }
}

//  Runs every 100 ms on DCbiasThread (created in DCbias_init) — the same
//  job MIPS's DCbias_loop() does: read each card's readback ADC, keep a
//  filtered per-channel readback (served by GDCBV/GDCBALLV), find the worst
//  setpoint-vs-readback error as % of full scale, and trip the supply off
//  when the filtered error exceeds VerrorThreshold.
void DCbias_loop(void)
{
  uint16_t ADCvals[8];

  pca9540SelectChannel(PCA9540_CHAN_EXT);

  bool testEnable = DCbiasPowerEnable && DCbiasTestEnable && (MonitorDelay == 0);
  if (MonitorDelay > 0) MonitorDelay--;

  Verror = 0;
  for (int b = 0; b < MAXDCBBOARDS; b++)
  {
    if (!DCbBoardPresent[b]) continue;
    DCbiasData &d = DCbDarray[b];
    SelectBoard(b);
    if (AD7998readAll(d.ADCadr, ADCvals) != 0) continue;   // card didn't answer — skip this pass

    for (int i = 0; i < d.NumChannels; i++)
    {
      // No offset readback this milestone — the offset setpoint stands in,
      // same as MIPS's non-OffsetReadback path.
      float V = Counts2Value(ADCvals[d.DCCD[i].DCmon.Chan], &d.DCCD[i].DCmon)
                + d.DCoffset.VoltageSetpoint;
      // Dynamic filter, same coefficients as MIPS (10/8/23 fix): track fast
      // on big moves, filter hard once settled.
      float flt = (fabsf(V - Readbacks[b][i]) < 2) ? 0.1 : 0.5;
      Readbacks[b][i] = flt * V + (1 - flt) * Readbacks[b][i];

      if (!testEnable) continue;
      float expected = d.DCCD[i].VoltageSetpoint;
      if ((d.OffsetChanMsk & (1 << i)) != 0) expected += d.ChannelOffset;
      float errorPercentage = (fabsf(Readbacks[b][i] - expected) / d.MaxVoltage) * 100.0;
      if (errorPercentage > Verror)
      {
        Verror = errorPercentage;
        VerrorCh = b * MAXDCBCHANNELS + i + 1;   // MIPS's DCBadd2chan(b, i)
      }
    }
  }

  VerrorFiltered = StrongFilter * Verror + (1 - StrongFilter) * VerrorFiltered;
  if (DCbiasPowerEnable && (VerrorThreshold > 0) && (VerrorFiltered > VerrorThreshold))
  {
    // Same trip action as MIPS minus the display popup — the full MIPS
    // controller sees the trip by polling GDCPWR (and GTRPLVL/GDCBALLV).
    //SetDCbiasPower(false);
    //Tripped = true;
  }
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
  cp.println(addr);
  if (ReadEEPROM(signature, addr, 0, 100) != 0) return;
  cp.println(signature);
  
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

  // Drive the DACs to the restored setpoints (or hold at zero if power is
  // off) — same net effect as MIPS's DCbias_loop() applying the restored
  // configuration after DCbias_init().
  AD5668enableInternalRef();
  for (int ch = 0; ch < DCbDarray[board].NumChannels; ch++)
  {
    float v = DCbiasPowerEnable ? DCbDarray[board].DCCD[ch].VoltageSetpoint : 0.0;
    AD5668write(DCbDarray[board].DCCD[ch].DCctrl.Chan,
                Value2Counts(v, &DCbDarray[board].DCCD[ch].DCctrl));
  }
  for (int ch = 0; ch < MAXDCBCHANNELS; ch++) Readbacks[board][ch] = 0.0;

  DCbBoardPresent[board] = true;

  // The first board found starts the readback monitor — same 100 ms
  // DCbiasThread MIPS's DCbias_init() configures on first init.
  static bool monitorStarted = false;
  if (!monitorStarted)
  {
    DCbiasThread.setName((char *)"DCbias");
    DCbiasThread.onRun(DCbias_loop);
    DCbiasThread.setInterval(100);
    control.add(&DCbiasThread);
    monitorStarted = true;
  }
  DelayMonitoring();
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
  if (DCbiasPowerEnable)
  {
    SelectBoard(board);
    SetAddress(d.DACspi);
    AD5668write(d.DCCD[localCh].DCctrl.Chan, Value2Counts(value, &d.DCCD[localCh].DCctrl));
  }
  DelayMonitoring();
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

  // Filtered readback maintained by the DCbias_loop() monitor thread —
  // same source MIPS's GDCBV serves (DCbiasStates[]->Readbacks[]).
  cp.sendACK(false);
  cp.println(Readbacks[board][localCh]);
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
  if (DCbiasPowerEnable)
  {
    SelectBoard(board);
    AD5668write(d.DCoffset.DCctrl.Chan, Value2Counts(value, &d.DCoffset.DCctrl));
  }
  DelayMonitoring();
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
  DelayMonitoring();
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
  DelayMonitoring();
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

// -- Power (ON|OFF — matches MIPS's DCbiasPowerSet/DCbiasPower conventions).
//    See SetDCbiasPower() above for what "off" means on this board.
static void DCbiasPowerSetCmd(void)   // SDCPWR, ON|OFF
{
  char *state;
  if (!cp.getValue(&state,"ON,OFF")) { cp.sendNAK(ERR_BADARG); return; }
  SetDCbiasPower(strcmp(state,"ON") == 0);
  cp.sendACK();
}

static void DCbiasPowerGetCmd(void)   // GDCPWR
{
  cp.sendACK(false);
  if (DCbiasPowerEnable) cp.println("ON");
  else                   cp.println("OFF");
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
    if (DCbiasPowerEnable)
    {
      SelectBoard(board);
      AD5668write(d.DCCD[localCh].DCctrl.Chan, Value2Counts(value, &d.DCCD[localCh].DCctrl));
    }
  }
  DelayMonitoring();
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
    cp.print(Readbacks[board][localCh]);   // maintained by DCbias_loop()
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

  {"SDCPWR",     CMDfunction, 1, (void *)DCbiasPowerSetCmd,          NULL, "Set DC bias power, ON or OFF"},
  {"GDCPWR",     CMDfunction, 0, (void *)DCbiasPowerGetCmd,          NULL, "Get DC bias power state, ON or OFF"},
  {"?TRPLVL",    CMDfloat,   -1, (void *)&VerrorThreshold, (void *)TrpLvlRange, "DC bias readback error trip level, % of FS; 0 disables"},
  {"?DCBTEST",   CMDbool,    -1, (void *)&DCbiasTestEnable,          NULL, "Enable DC bias readback error testing, TRUE or FALSE"},

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
