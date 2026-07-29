// =============================================================================
//  ISOLATEDCONTROLLER — HOST APPLICATION
//  GAA Custom Electronics — IsolatedControllerR1.0
//  Milestone 1: framework + DCbias module discovery/hosting.
//  See IsolatedController.h and DCbias.h for the architecture notes.
// =============================================================================

#define AppName        "IsolatedController"
#define UseThreads     true    // 25 ms periodic Update() thread
#define UseSPIflash    false   // deferred — see platformio.ini note

#include "IsolatedController.h"
#include "DCbias.h"

#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include <SPI.h>

#include <RingBuffer.h>
#include <commandProcessor.h>
#include <charAllocate.h>
#include <debug.h>
#include <Errors.h>
#include <Devices.h>
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>

#if UseThreads
#include <Thread.h>
#include <ThreadController.h>
ThreadController control = ThreadController();
Thread SystemThread = Thread();
#endif

// =============================================================================
//  Host-level persistent data — just this board's own identity now that
//  DCbias owns its own Data struct (DCbiasData, in DCbias.cpp).
// =============================================================================
typedef struct
{
  int16_t   Size;
  char      Name[20];
  int8_t    Rev;
  unsigned int Signature;
} Data;

Data data;
Data Rev_1_data =
{
  sizeof(Data),
  AppName,
  1,
  SIGNATURE
};
FlashStorage(flash_data, Data);

const char *Version = AppName ", version 0.2";

// =============================================================================
//  Command processor
// =============================================================================
commandProcessor cp;
debug dbg(&cp);

void SaveSettings(void);
void RestoreSettings(void);
void bootloader(void);

Command cmds[] =
{
  {"GVER",    CMDstr,      -1, (void *)Version,        NULL, "Firmware version"},
  {"?NAME",   CMDstr,      -1, (void *)&data.Name,     NULL, "Device name"},
  {"SAVE",    CMDfunction,  0, (void *)SaveSettings,   NULL, "Save system settings to internal flash"},
  {"RESTORE", CMDfunction,  0, (void *)RestoreSettings,NULL, "Restore system settings from internal flash"},
  {"BLOAD",   CMDfunction,  0, (void *)bootloader,     NULL, "Jump to bootloader for firmware update"},
  {NULL}
};
static CommandList cmdList = {cmds, NULL};

void Debug(void) { /* ad-hoc bring-up tests go here */ }

#if UseThreads
void Update(void)
{
  // TODO: DIO/Analog module polling goes here once those modules exist.
  // DCbias readback monitoring runs on its own thread (DCbias_loop, 100 ms,
  // created in DCbias_init()) — see DCbias.cpp.
}
#endif

// =============================================================================
//  Board select (BRDSEL) — host-level, shared by every module that talks to
//  the external MIPS bus. Matches MIPS exactly: even board number -> A
//  (BRDSEL high), odd -> B (BRDSEL low). Tracks current state to skip
//  redundant digitalWrite() calls, same optimization MIPS's own
//  SelectBoard()/SelectedBoard() (src/Hardware.cpp) makes.
// =============================================================================
static int currentBoardSel = -1;   // -1 forces the first call to actually write

void SelectBoard(int8_t board)
{
  int wantA = ((board & 1) == 0) ? 1 : 0;
  if (currentBoardSel == wantA) return;
  if (wantA) ENA_BRD_A;
  else       ENA_BRD_B;
  currentBoardSel = wantA;
}

int SelectedBoard(void)
{
  // 1 = A (even board numbers), 0 = B (odd) — matches SelectBoard() above.
  return currentBoardSel;
}

// =============================================================================
//  EEPROM helpers — small single-byte-address I2C EEPROM (24LCxx-style),
//  used by DCbias_scan()/DCbias_init() to read a module card's signature
//  and saved configuration. Mirrors MIPS's own ReadEEPROM/WriteEEPROM
//  (src/Hardware.cpp) minus the Due-specific AtomicBlock/TWI-queue plumbing.
// =============================================================================
int ReadEEPROM(void *dst, uint8_t dadr, uint16_t address, uint16_t count)
{
  uint8_t *bval = (uint8_t *)dst;
  uint16_t remaining = count;
  uint16_t addr = address;

  while (remaining > 0)
  {
    uint16_t chunk = (remaining > 32) ? 32 : remaining;

    Wire.beginTransmission(dadr | ((addr >> 8) & 1));
    Wire.write(addr & 0xFF);
    if (Wire.endTransmission(true) != 0) return -1;

    Wire.requestFrom((int)(dadr | ((addr >> 8) & 1)), (int)chunk);
    uint16_t got = 0;
    while (Wire.available() && got < chunk) { *(bval++) = Wire.read(); got++; }
    if (got != chunk) return -1;

    addr += chunk;
    remaining -= chunk;
  }
  return 0;
}

int WriteEEPROM(void *src, uint8_t dadr, uint16_t address, uint16_t count)
{
  uint8_t *bval = (uint8_t *)src;
  uint16_t remaining = count;
  uint16_t addr = address;

  while (remaining > 0)
  {
    uint16_t chunk = (remaining > 16) ? 16 : remaining;  // small page size, conservative

    Wire.beginTransmission(dadr | ((addr >> 8) & 1));
    Wire.write(addr & 0xFF);
    for (uint16_t i = 0; i < chunk; i++) Wire.write(bval[i]);
    if (Wire.endTransmission(true) != 0) return -1;
    delay(5);   // EEPROM write cycle time

    bval += chunk;
    addr += chunk;
    remaining -= chunk;
  }
  return 0;
}

// =============================================================================
//  Setup / loop
// =============================================================================
void setup()
{
  data = flash_data.read();
  if (data.Signature != SIGNATURE) data = Rev_1_data;

  Serial.begin(0);
  cp.registerStream(&Serial);

  // RS232 to the isolated full MIPS controller — same ASCII protocol, just
  // another registered stream. TODO: confirm baud rate against that
  // controller's Serial1 configuration.
  Serial1.begin(115200);
  cp.registerStream(&Serial1);

  cp.registerCommands(&cmdList);
  cp.registerCommands(DCbias_commands());   // module owns its own table
  cp.registerCommands(dbg.debugCommands());
  dbg.registerDebugFunction(Debug);

  #if UseThreads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  control.add(&SystemThread);
  #endif

  // -- Hardware peripheral initialisation --------------------------------
  pinMode(PIN_BRDSEL, OUTPUT);
  SelectBoard(0);   // default to board 0 ("A") at boot

  pinMode(PIN_ADDR0, INPUT);
  pinMode(PIN_ADDR1, INPUT);
  pinMode(PIN_ADDR2, INPUT);

  // TODO: confirm direction — placeholder as inputs.
  pinMode(PIN_GRDSTA, INPUT);
  pinMode(PIN_GRDPWR, INPUT);
  pinMode(PIN_INT4, INPUT);
  pinMode(PIN_INT5, INPUT);
  pinMode(PIN_INT6, INPUT);
  pinMode(PIN_INT7, INPUT);

  pinMode(PIN_TRIG_IN,  INPUT);
  pinMode(PIN_TRIG_OUT, OUTPUT);
  digitalWrite(PIN_TRIG_OUT, LOW);

  csPinMode();
  SPI.begin();

  Wire.begin();
  // DIO/Analog modules (onboard AD5593s, PCA9540_CHAN_LOCAL) — not
  // implemented this milestone. When added, they'll init unconditionally
  // here (or with a lightweight I2C presence-check), same as this:
  DCbias_scan();
}

void loop()
{
  cp.processStreams();
  cp.processCommands();
  #if UseThreads
  control.run();
  #endif
}

// =============================================================================
//  System commands
// =============================================================================
void SaveSettings(void)
{
  data.Signature = SIGNATURE;
  flash_data.write(data);
  cp.sendACK();
}

void RestoreSettings(void)
{
  static Data td;
  td = flash_data.read();
  if (td.Signature == SIGNATURE) { data = td; cp.sendACK(); }
  else cp.sendNAK(ERR_NOSDCARD);
}


void bootloader(void)
{
  cp.sendACK();
  #if SAMD51_SERIES
  __disable_irq();
  static volatile uint32_t * const bootAddr51 = (volatile uint32_t *)(HSRAM_ADDR + HSRAM_SIZE - 4);
  static const    uint32_t         tapMagic51 = 0xf01669efUL;
  *bootAddr51 = tapMagic51;
  NVIC_SystemReset();
  while (true);
  #endif
}
