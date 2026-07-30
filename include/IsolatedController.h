// =============================================================================
//  ISOLATEDCONTROLLER — HOST HEADER
//  GAA Custom Electronics — IsolatedControllerR1.0 firmware
// =============================================================================
//
//  IsolatedController.cpp plays the role MIPS.cpp plays on a full MIPS
//  mainframe: it owns the framework (commandProcessor, threads, persistence)
//  and hosts whatever's attached to its own module bus. Concretely:
//
//    - The two onboard AD5593R chips (DIO + Analog IO) sit on I2C mux
//      channel PCA9540_CHAN_LOCAL. They implement functionality that used
//      to live on the MIPS mainframe itself (the DIO and Analog modules) —
//      not covered in this milestone; see the placeholder comment in
//      IsolatedController.cpp.
//    - The EXT1/EXT2 headers (buffered through IC3) are this board's own
//      "MIPS bus" on I2C mux channel PCA9540_CHAN_EXT — this is what a
//      real external DCbias module card plugs into. IsolatedController
//      discovers it the same way MIPS.cpp's ScanHardware() does: read a
//      signature block from the card's ID EEPROM, and if it matches, call
//      that module's _init(addr).
//
//  Each module (DCbias.cpp/.h now, DIO/Analog later) owns its own Command
//  table and exposes a <Module>_commands() accessor — mirrors GAACE_Core's
//  own debug.h pattern rather than MIPS's single centralized Serial.cpp
//  table.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>

#define SIGNATURE  0xAA55A5A7

// =============================================================================
//  PIN DEFINITIONS — IsolatedControllerR1.0
// =============================================================================
//  See NOTES.md for the extraction method (pulled from .kicad_pcb pad
//  `pinfunction` properties, not inferred).

#define PIN_RX          0    // RS232 from isolated full MIPS controller (OPTO1)
#define PIN_TX          1    // RS232 to isolated full MIPS controller (OPTO2)
// SDA/SCL: hardware I2C (Wire) to IC1 (PCA9540 mux)
// MOSI/MISO/SCK: hardware SPI — this is the DCbias card's DAC bus (EXT2)

#define PIN_LDAC        5    // TCC compare-match capable
#define PIN_PWMRFCH1    10   // independent TCC instance
#define PIN_PWMRFCH2    12   // independent TCC instance

#define PIN_BRDSEL      A0   // OUTPUT — this board drives it (see below)
#define PIN_GRDSTA      A1
#define PIN_GRDPWR      A2
#define PIN_INT6        A3
#define PIN_ADDR0       A4
#define PIN_ADDR1       A5
#define PIN_ADDR2       2
#define PIN_TRIG_IN     3    // isolated, external -> this board (OPTO3)
#define PIN_EXT2_14     4    // plain digital, not PWM
#define PIN_INT4        7
#define PIN_INT7        9
#define PIN_INT5        11
#define PIN_TRIG_OUT    13   // isolated, this board -> external (OPTO4)
// CS (SWCLK/PA30) — not in the Arduino pin array, direct PORT access only.

// -- BRDSEL — output, mirrors MIPS's ENA_BRD_A/ENA_BRD_B pattern -------------
// Every module that talks to the external MIPS bus (DCbias now, others
// later) depends on this — it's how two physical cards can share one TWI
// address / one SPI CS. Board numbering matches MIPS exactly: even board
// numbers -> A (BRDSEL high), odd -> B (BRDSEL low).
#define ENA_BRD_A   digitalWrite(PIN_BRDSEL, HIGH)
#define ENA_BRD_B   digitalWrite(PIN_BRDSEL, LOW)

// SelectBoard()/SelectedBoard() — host-level (declared here, not per-module)
// since any module using the external bus needs the same board-select
// state. Implemented in IsolatedController.cpp.
void SelectBoard(int8_t board);
int  SelectedBoard(void);

// -- Direct PORT access for CS (SWCLK / PA30) --------------------------------
// TODO: confirm CS polarity — assumed active-low (typical for SPI CS).
static inline void csPinMode(void)
{
  // 1. Disable the peripheral multiplexer on PA30 so it acts as standard GPIO
  PORT->Group[0].PINCFG[30].bit.PMUXEN = 0;

  // 2. Set PA30 as a Digital Output
  PORT->Group[0].DIRSET.reg = PORT_PA30;

  //PORT->Group[0].DIRSET.reg = PORT_PA30;
}
static inline void csWrite(bool active)
{
  if (active) PORT->Group[0].OUTCLR.reg = PORT_PA30;
  else PORT->Group[0].OUTSET.reg = PORT_PA30;
}

// =============================================================================
//  I2C MUX (IC1, PCA9540) — exclusive 2-channel, fixed address, no A0 pin.
//  Powers up with NO channel selected — always select before talking to
//  either side.
// =============================================================================
#define PCA9540_ADDR        0x70
#define PCA9540_CHAN_LOCAL  0x05   // -> IC4/IC5 (onboard AD5593s: DIO + Analog)
#define PCA9540_CHAN_EXT    0x04   // -> IC3 (buffer) -> EXT1/EXT2 "MIPS bus"
// TODO: confirm these two channel numbers against the schematic — the
// numbering itself doesn't affect function, just which constant is which.

inline int pca9540SelectChannel(uint8_t channel)
{
  Wire.beginTransmission(PCA9540_ADDR);
  Wire.write(0x04 | (channel & 0x01));
  return Wire.endTransmission();
}

// =============================================================================
//  MODULE DISCOVERY — EEPROM read/write for the external MIPS bus (EXT1/EXT2)
// =============================================================================
//
//  Mirrors MIPS's own ReadEEPROM/WriteEEPROM (src/Hardware.cpp): small
//  single-byte-address I2C EEPROM (24LCxx-style), read/written in <=32 byte
//  chunks. A module's signature block is its own persistent Data struct
//  written starting at EEPROM address 0 — byte layout {int16_t Size;
//  char Name[20]; int8_t Rev; ...}, so `&signature[2]` is the Name string
//  and `signature[22]` is Rev, exactly like MIPS.cpp's ScanHardware().
int ReadEEPROM(void *dst, uint8_t dadr, uint16_t address, uint16_t count);
int WriteEEPROM(void *src, uint8_t dadr, uint16_t address, uint16_t count);

// Known module ID EEPROM addresses on the external MIPS bus. Trimmed from
// MIPS's own 6-address list to the 4 standard slots — extend later if
// this board ever needs to stack multiple external cards.
#define NumModAdd 4
extern const uint8_t ModuleAddresses[NumModAdd];
