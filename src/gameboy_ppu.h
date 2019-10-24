
/**/


#ifndef HEADER_GAMEBOY_PPU_H
#define HEADER_GAMEBOY_PPU_H

#include <array>
#include <queue>

#include "gameboy_common.h"
#include "gameboy_bus.h"


/**
 *
 */
struct pxdata
{
    u8 r, g, b;
};


/**
 *
 */
class gameboy_ppu
{
public:
    void (*drawScanlineCallback)() = nullptr;

    void setBus(gameboy_bus* b);
    void clock();

private:
    gameboy_bus* bus;
    //u8 io[0x20]; // PPU IO covers $FF40 - $FF6F
    u8 vram[0x2000], oam[0xA0];

    u32 clks = 0;
    u8 fetcherStep = 0, fetcherClksLeft = 0;
    u8 spriteList[10];
    u8 pxcount = 0;
    std::array<u8, 8> fetchedTile, fetchedSprite;
    std::queue<u8> pxfifo;
    std::array<u8, GB_LCD_XPIXELS> scanline;

    u16 x, y, bgMapCol, bgMapRow, bgMapAddr, bgDataAddr, bgMapOffset, bgDataOffset;

    u8 read8(u16 addr);
    void write8(u16 addr, u8 n);

    void fetchTile();
    void fetchSprite();
};



#endif
