
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
    bool upPressed = false, downPressed = false, leftPressed = false, rightPressed = false, 
        cPressed = false, xPressed = false, zPressed = false, spacePressed = false;
    bool running = true;
    //SDL_Rect r;

    if (SDL_Init(SDL_INIT_VIDEO))
    {
        printf("\nUnable to start SDL!");
        return -1;
    }

    window = SDL_CreateWindow("SimpleBoy", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, GB_LCD_XPIXELS, GB_LCD_YPIXELS, SDL_WINDOW_SHOWN);

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
            else if (e.type == SDL_KEYDOWN) // keyboard button pressed
            {
                switch (e.key.keysym.sym)
                {
                    case SDLK_UP:
                    {
                        if (!upPressed)
                        {
                            gb.gb_bus.updateJoypadPressed(JOYPAD_UP, true);
                            upPressed = true;
                        }                        
                        break;
                    }
                    case SDLK_DOWN:
                    {
                        if (!downPressed)
                        {
                            gb.gb_bus.updateJoypadPressed(JOYPAD_DOWN, true);
                            downPressed = true;
                        }
                        break;
                    }
                    case SDLK_LEFT:
                    {
                        if (!leftPressed)
                        {
                            gb.gb_bus.updateJoypadPressed(JOYPAD_LEFT, true);
                            leftPressed = true;
                        }
                        break;
                    }
                    case SDLK_RIGHT:
                    {
                        if (!rightPressed)
                        {
                            gb.gb_bus.updateJoypadPressed(JOYPAD_RIGHT, true);
                            rightPressed = true;
                        }
                        break;
                    }
                    case SDLK_c:
                    {
                        if (!cPressed)
                        {
                            gb.gb_bus.updateJoypadPressed(JOYPAD_A, false);
                            cPressed = true;
                        }
                        break;
                    }
                    case SDLK_x:
                    {
                        if (!xPressed)
                        {
                            gb.gb_bus.updateJoypadPressed(JOYPAD_B, false);
                            xPressed = true;
                        }
                        break;
                    }
                    case SDLK_z:
                    {
                        if (!zPressed)
                        {
                            gb.gb_bus.updateJoypadPressed(JOYPAD_SELECT, false);
                            zPressed = true;
                        }
                        break;
                    }
                    case SDLK_SPACE:
                    {
                        if (!spacePressed)
                        {
                            gb.gb_bus.updateJoypadPressed(JOYPAD_START, false);
                            spacePressed = true;
                        }
                        break;
                    }
                }
            }
            else if (e.type == SDL_KEYUP) // keyboard button released
            {
                switch (e.key.keysym.sym)
                {
                    case SDLK_UP:
                    {
                        printf("\nUP PRESSED");
                        gb.gb_bus.updateJoypadReleased(JOYPAD_UP, true);
                        upPressed = false;
                        break;
                    }
                    case SDLK_DOWN:
                    {
                        gb.gb_bus.updateJoypadReleased(JOYPAD_DOWN, true);
                        downPressed = false;
                        break;
                    }
                    case SDLK_LEFT:
                    {
                        gb.gb_bus.updateJoypadReleased(JOYPAD_LEFT, true);
                        leftPressed = false;
                        break;
                    }
                    case SDLK_RIGHT:
                    {
                        gb.gb_bus.updateJoypadReleased(JOYPAD_RIGHT, true);
                        rightPressed = false;
                        break;
                    }
                    case SDLK_c:
                    {
                        gb.gb_bus.updateJoypadReleased(JOYPAD_A, false);
                        cPressed = false;
                        break;
                    }
                    case SDLK_x:
                    {
                        gb.gb_bus.updateJoypadReleased(JOYPAD_B, false);
                        xPressed = false;
                        break;
                    }
                    case SDLK_z:
                    {
                        gb.gb_bus.updateJoypadReleased(JOYPAD_SELECT, false);
                        zPressed = false;
                        break;
                    }
                    case SDLK_SPACE:
                    {
                        gb.gb_bus.updateJoypadReleased(JOYPAD_START, false);
                        spacePressed = false;
                        break;
                    }
                }
            }
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
