
/**/


#ifndef HEADER_GAMEBOY_PPU_H
#define HEADER_GAMEBOY_PPU_H

#include <array>
#include <queue>
#include <SDL.h>

#include "gameboy_common.h"
#include "gameboy_bus.h"


/**
 *
 */
class gameboy_ppu
{
public:
    void setRender(SDL_Renderer* r, SDL_Window *w, SDL_Surface *s);

    void setBus(gameboy_bus* b);
    void clock();

private:
    gameboy_bus* bus;

    u32 clks = 0;
    u8 fetcherStep = 0, fetcherClksLeft = 0;
    u8 spriteList[10];
    u8 pxcount = 0;
    std::array<u8, 8> fetchedTile, fetchedSprite;
    std::queue<u8> pxfifo;
    std::array<u8, GB_LCD_XPIXELS> scanline;

    SDL_Renderer* renderer;
    SDL_Window* window;
    SDL_Surface* screen;

    s32 x, y;
    u16 bgMapCol, bgMapRow, bgMapAddr, bgDataAddr, bgMapOffset, bgDataOffset;
    u8 pxToDiscardX = 0, bgTileVerticalOffset = 0, pxUntilRenderWindow;

    u8 read8(u16 addr);
    void write8(u16 addr, u8 n);

    void fetchTile(bool windowFetch);
    void fetchSprite();

    void renderScanline();
    void renderScreen();
};



#endif
