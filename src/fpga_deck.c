/**
 * FPGA Deck Driver
 * 
 * This is a minimal deck driver that reserves the hardware resources
 * (SPI, IO1, IO2) for the FPGA controller. This prevents conflicts
 * with other decks.
 *
 * Since the FPGA deck has no OneWire memory, you must force-load it
 * by adding to app-config:
 *   CONFIG_DECK_FORCE="bcFPGA"
 */

#include "deck.h"

#define DEBUG_MODULE "FPGA_DECK"
#include "debug.h"

static bool isInit = false;

static void fpgaDeckInit(DeckInfo *info) {
    if (isInit) return;
    
    DEBUG_PRINT("FPGA deck initialized (reserving IO1, IO2, SPI)\n");
    isInit = true;
}

static bool fpgaDeckTest(void) {
    return isInit;
}

static const DeckDriver fpga_deck = {
    .vid = 0x00,          // Custom deck (no official VID)
    .pid = 0x00,          // Custom deck (no official PID)
    .name = "bcFPGA",     // Name used for CONFIG_DECK_FORCE
    
    // Declare used resources - prevents conflicts with other decks
    .usedPeriph = DECK_USING_SPI,
    .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2,
    
    .init = fpgaDeckInit,
    .test = fpgaDeckTest,
};

DECK_DRIVER(fpga_deck);
