
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
    gb_bus.cartridge() = make_gameboy_cart(romFname);

    if (gb_bus.cartridge() == nullptr)
        return false;

    return true;
}


/**
 * Performs one 'step' of execution of the GameBoy CPU. The number of clock
 * cycles this consumes will vary by instruction. The PPU and other devices
 * are all clocked the same amount from within the gameboy_cpu::execute()
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
    SDL_Event e;
    bool running = true;
    //SDL_Rect r;

    if (SDL_Init(SDL_INIT_VIDEO))
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
    SDL_RenderClear(renderer);
    
    gb.gb_ppu.setRender(renderer, window, screen);
    bool display = false;

    while (running)
    {
        while (SDL_PollEvent(&e) != 0)
        {
            if (e.type == SDL_QUIT)  // Window X'ed out
                running = false;
        }

        for (int i = 0; i < (CLOCK_GB_SCREENREFRESH/4); i++)
        {
            gb.step();
        }
    }

    SDL_Quit();

    /*for (int i = 0; i < 900000000; i++)
    {
        gb.step();
        //printf("\nIF = %X", gb.gb_cpu.read8(0xFF00 + IO_INT_IF));
        //if (gb.gb_cpu.pc == 0xc2a6)// 0x2a5)
        //    display = true;

        if (display)//gb.gb_bus.display)
        {
            printf("\nPC: %X, @PC: %X, AF: %X, BC: %X, DE: %X, HL: %X, SP: %X, TIMA: %X", gb.gb_cpu.pc, gb.gb_cpu.read8(gb.gb_cpu.pc), gb.gb_cpu.af.r, gb.gb_cpu.bc.r, gb.gb_cpu.de.r, gb.gb_cpu.hl.r, gb.gb_cpu.sp, gb.gb_cpu.read8(0xFF00 + IO_TIM_TIMA));
            std::cin.ignore(80, '\n');
        }
    }*/


    return 0;
}
