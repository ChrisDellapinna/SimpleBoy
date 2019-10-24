
/**
 * SimpleBoy is a work-in-progress Nintendo GameBoy emulator currently
 * usable on Windows. A GUI, simple debugger and cross-platform compatibility
 * are currently planned.
 *
 *
 * @author Chris Dellapinna
 *
 */


#include <SDL.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <queue>
#include <array>

#include "gameboy_common.h"
#include "gameboy_bus.h"
#include "gameboy_cpu.h"
#include "gameboy_ppu.h"
#include "gameboy_cart.h"







/**
* Class respresenting the GameBoy emulator.
*/
class gameboy
{
    public:
        gameboy();
        
        bool init(const char* romFname);
        void step();

    //private:
        gameboy_bus gb_bus;
        gameboy_cpu gb_cpu{ &gb_bus };
        gameboy_ppu gb_ppu;

};


/**
 * A simple constructor that assigns a gameboy_bus* to the CPU and PPU.
 */
gameboy::gameboy()
{
    gb_cpu.setBus(&gb_bus);
    gb_cpu.setPpu(&gb_ppu);
    gb_ppu.setBus(&gb_bus);
}


/**
 * Initializes the gameboy class. Initializes its members as well. Loads the
 * ROM file located at romFname. Returns true on success, false otherwise.
 *
 * @param[in] The name of the ROM file to load.
 * @returns true on success, false on otherwise.
 */
bool gameboy::init(const char *romFname)
{
    gb_cpu.init();
    //gb_ppu.init();
    if (!gb_bus.cartridge()->load(romFname))
    {
        printf("\nFailed to load ROM. \n\n");
        return false;
    }

    return true;
}


/**
 * Performs one 'step' of execution of the GameBoy CPU. The number of clock
 * cycles this consumes will vary by instruction. The PPU and other devices
 * are all clocked the same amount from with the gameboy_cpu::execute()
 * function (see gameboy_cpu::execute, gameboy_cpu::clock and gameboy_ppu::clock)
 */
void gameboy::step()
{
    gb_cpu.execute();
}


int main(int argc, char *argv[])
{
    // Just some sanity testing.. blah blah..
    gameboy gb;
    if (!gb.init("test.gb"))
    {
        printf("\n\nFailed to init gameboy class. Quitting...");
        std::cin.ignore(80, '\n');
        return 1;
    }


    SDL_Window *window = NULL;
    SDL_Surface *screen = NULL;
    SDL_Renderer* renderer = NULL;
    //SDL_Rect r;

    /*if (SDL_Init(SDL_INIT_VIDEO))
    {
        printf("\nUnable to start SDL!");
        return -1;
    }

    window = SDL_CreateWindow("SimpleBoy", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 256, 256, SDL_WINDOW_SHOWN);

    if (window == NULL)
    {
        printf("\nUnable to create SDL windows:  %s", SDL_GetError());
        return -1;
    }

    //screen = SDL_GetWindowSurface(window);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);*/
    

    for (int i = 0; i < 1500000; i++)
    {
        gb.step();
        printf("\nPC: %X, AF: %X, BC: %X, DE: %X, HL: %X", gb.gb_cpu.pc, gb.gb_cpu.af.r, gb.gb_cpu.bc.r, gb.gb_cpu.de.r, gb.gb_cpu.hl.r);
        //std::cin.ignore(80, '\n');
    }

    //SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 255, 255, 255));

    /*for (int x = 0; x < 32; x++) // tile's x coord ( * 8) in the background map
    {
        for (int y = 0; y < 4; y++) // tile's y coord ( * 8) ...
        {
            for (int py = 0; py < 8; py++) // p from 0 ... 7 for each of the two bytes making up a ln of px data
            {
                u8 pxlsb = gb.vram[0x0000 + y * 32 * 16 + x * 16 + py * 2],
                    pxmsb = gb.vram[0x0000 + y * 32 * 16 + x * 16 + py * 2 + 1];

                for (int px = 0; px < 8; px++) // relative x coord of the tile
                {
                    //SDL_RenderClear(renderer);

                    
                    //r.x = 4;//(x * 8 + px);
                    //r.y = 4;//(y * 8 + py);
                    //r.w = 5;
                    //r.h = 5;

                    u8 col = (pxlsb >> (7 - px) & 1) | ((pxmsb >> (7 - px) & 1) << 1);
                    printf("%X ", col);
                    col <<= 6; // close enough..

                    SDL_SetRenderDrawColor(renderer, col, col, col, 255);
                    if (SDL_RenderDrawPoint(renderer, (x * 8 + px), (y * 8 + py)) < 0)
                        printf("\n\nSDL Error: %s", SDL_GetError());
                    //SDL_RenderFillRect(renderer, &r);
                    
                }
                printf("\n");
            }
            printf("\n");
        }
    }
    SDL_RenderPresent(renderer);  // update screen

    for (int i = 0; i < 0x1800; i++)
    {
        printf("%X ", gb.vram[i]);
    }*/

    //SDL_UpdateWindowSurface(window);
    /*SDL_RenderPresent(renderer);

    SDL_Delay(20000);
    SDL_Quit();*/

    

    /*for (int i = 0; i < 600000; i++)
    {
        if (gb.pc == 0x63b)
        {
            while (true)
            {
                printf("\n%X @ %X\tSP: %X  AF: %X  BC: %X  DE: %X  HL: %X \tClks elasped: %i, PPU clks: %i, LY: %i, STAT mode: %i, STAT: %X, LCDC: %X, LYC: %X, IF: %X, IE: %X",
                    gb.read8(gb.pc), gb.pc, gb.sp, gb.af.r, gb.bc.r, gb.de.r, gb.hl.r, gb.clks, gb.ppu_clks, gb.io[IO_PPU_LY], (gb.io[IO_PPU_STAT] & 3),
                    gb.io[IO_PPU_STAT], gb.io[IO_PPU_LCDC], gb.io[IO_PPU_LYC], gb.io[IO_INT_IF], gb.ier);
                gb.execute();
                std::cin.ignore(80, '\n');
            }
        }
        gb.execute();
    }

    printf("\n\nEmulation finished. Hit Enter (Return) key to exit.\n");
    std::cin.ignore(80, '\n');*/

    return 0;
}
