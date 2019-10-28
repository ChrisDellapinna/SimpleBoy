
/**/

#ifndef HEADER_GAMEBOY_BUS_H
#define HEADER_GAMEBOY_BUS_H

#include "gameboy_cart.h"


class gameboy_bus
{
public:
    gameboy_bus() {};

    u8 read8CPU(u16 addr);
    u8 read8PPU(u16 addr);
    u8 read8DMA(u16 addr);
    void write8CPU(u16 addr, u8 n);
    void write8PPU(u16 addr, u8 n);
    void write8DMA(u16 addr, u8 n);
    
    void updateDiv(u8 div);
    bool isDivReset();
    
    void irq(u8 interruptPos);
    gameboy_cart* cartridge();

    u8 ime = 1;

private:
    // all the addressable memories go here
    gameboy_cart cart;

    // Assorted emulated memory
    u8 vram[0x2000];
    u8 extram[0x2000];
    u8 wram[0x2000];
    u8 oam[0xA0];
    u8 hram[0x7F];
    u8 io[0x80];
    u8 ier;
    //u8 ime = 1;

    bool divReset = false;
};


#endif

