
/**/

#include <SDL.h>

#include "gameboy_common.h"
#include "gameboy_bus.h"
#include "gameboy_ppu.h"


/**
 * Sets up several rendering related variables so the PPU class can later render images.
 *
 * @param[in] The SDL_Renderer * to be used, the SDL_Window * to be used, the SDL_Surface * to be used.
 */
void gameboy_ppu::setRender(SDL_Renderer* r, SDL_Window* w, SDL_Surface* s)
{
    renderer = r;
    window = w;
    screen = s;
}


/**
 * Simply sets bus to b.
 */
void gameboy_ppu::setBus(gameboy_bus* b)
{
    bus = b;
}


/**
 * Clocks the emulated PPU 4 cycles off main clock.
 * This should (ideally) push 4 pixels while rendering LCD, perform two IO operations on VRAM/OAM
 */
void gameboy_ppu::clock()
{
    // four clocks
    for (int i = 0; i < 4; i++)
    {
        // check lcd/stat timing
        if ((bus->read8PPU(0xFF00 + IO_PPU_LCDC) & 0x80) == 0) // Display is off
        {
            clks = 0;
            bus->write8PPU(0xFF00 + IO_PPU_STAT, bus->read8PPU(0xFF00 + IO_PPU_STAT) & 0xFC);
            //io[IO_PPU_STAT] &= 0xFC; // h-blank / display off (Cycle Accurate GameBoy Docs state mode = 0)

            // TODO: Still perform LY/LYC comparison even though LCD off?
        }
        else
        {
            // display on, do normal ppu clocking
            if (clks % 456 == 0) // If we just moved onto the next line/LY
            {
                if (bus->read8PPU(0xFF00 + IO_PPU_LY) == 0 && (bus->read8PPU(0xFF00 + IO_PPU_STAT) & 3) == 1)
                    //if (io[IO_PPU_LY] == 0 && (io[IO_PPU_STAT] & 0x03) == 1) // stat=vblank, on line 153
                    bus->write8PPU(0xFF00 + IO_PPU_STAT, bus->read8PPU(0xFF00 + IO_PPU_STAT) & 0xFC);
                //io[IO_PPU_STAT] &= 0xFC; // set stat to h-blank (0), leave ly as-is for ln 0
                // POTENTIALLY TRIGGER STAT IRQ HERE?????
                else
                    bus->write8PPU(0xFF00 + IO_PPU_LY, bus->read8PPU(0xFF00 + IO_PPU_LY) + 1);
                //io[IO_PPU_LY]++;

            // LYC Coincidence bit of STAT is always reset on this clock, except when LY=0
                if (bus->read8PPU(0xFF00 + IO_PPU_LY) != 0)
                    bus->write8PPU(0xFF00 + IO_PPU_STAT, bus->read8PPU(0xFF00 + IO_PPU_STAT) & 0xFB);
                else
                {
                    if (bus->read8PPU(0xFF00 + IO_PPU_LYC) == 0)
                        bus->write8PPU(0xFF00 + IO_PPU_STAT, bus->read8PPU(0xFF00 + IO_PPU_STAT) | 4);
                    else
                        bus->write8PPU(0xFF00 + IO_PPU_STAT, bus->read8PPU(0xFF00 + IO_PPU_STAT) & 0xFB);
                }
                /*if (io[IO_PPU_LY] != 0)
                    io[IO_PPU_STAT] &= 0xFB;
                else
                    (io[IO_PPU_LYC] == 0) ? (io[IO_PPU_STAT] |= 4) : (io[IO_PPU_STAT] &= 0xFB);*/
            }
            else
            {
                //
                if (clks % 456 == 4)
                {
                    if (read8(0xFF00 + IO_PPU_LY) == 153) // io[IO_PPU_LY] == 153)  // becomes ly=0, vblank
                        write8(0xFF00 + IO_PPU_LY, 0); //); io[IO_PPU_LY] = 0;
                    else if (read8(0xFF00 + IO_PPU_LY) <= 143) //( io[IO_PPU_LY] <= 143) // standard display line, stat=oam search (2)
                    {
                        write8(0xFF00 + IO_PPU_STAT, (read8(0xFF00 + IO_PPU_STAT) & 0xFC) | 2);

                        //io[IO_PPU_STAT] &= 0xFC; // stat should already =0, probably not needed
                        //io[IO_PPU_STAT] |= 2;

                        // Trigger STAT IRQ if enabled in STAT
                        if ((read8(0xFF00 + IO_PPU_STAT) & 0x20) != 0) //(io[IO_PPU_STAT] & 0x20) != 0)
                            bus->irq(INT_STAT);
                    }
                    else if (read8(0xFF00 + IO_PPU_LY) == 144) //io[IO_PPU_LY] == 144) // start of vblank
                    {
                        write8(0xFF00 + IO_PPU_STAT, (read8(0xFF00 + IO_PPU_STAT) & 0xFC) | 1);
                        //io[IO_PPU_STAT] &= 0xFC;
                        //io[IO_PPU_STAT] |= 1; // vblank
                        bus->irq(INT_VBLANK);

                        // Trigger STAT IRQ if enabled in STAT
                        if ((read8(0xFF00 + IO_PPU_STAT) & 0x10) != 0)
                            bus->irq(INT_STAT);
                        //if ((io[IO_PPU_STAT] & 0x10) != 0)
                        //    bus->irq(INT_STAT);
                    }

                }
                else if (clks % 456 == 84 && (read8(0xFF00 + IO_PPU_STAT) & 3) == 2) // going from oam -> lcd transfer mode
                {
                    write8(0xFF00 + IO_PPU_STAT, read8(0xFF00 + IO_PPU_STAT) + 1);
                    //io[IO_PPU_STAT]++; // stat=3 (lcd transfer)
                }

                // Perform the LY/LYC comparison, special case for line 153 (clk 4 compares LYC to 153, clk 8 is always flag reset)
                if (read8(0xFF00 + IO_PPU_LY) == 0 && (read8(0xFF00 + IO_PPU_STAT) & 3) == 1)
                {
                    if ((clks % 456) == 4) // special case, LYC compared to 153 not LY's value of 0
                    {
                        if (read8(0xFF00 + IO_PPU_LYC) == 153)
                            write8(0xFF00 + IO_PPU_STAT, read8(0xFF00 + IO_PPU_STAT) | 4);

                        // (io[IO_PPU_LYC] == 153) ? (io[IO_PPU_STAT] |= 4) : (io[IO_PPU_STAT] &= 0xFB);
                    }
                    else if ((clks % 456) == 8) // special case, coincidence bit is always reset
                        write8(0xFF00 + IO_PPU_STAT, read8(0xFF00 + IO_PPU_STAT) & 0xFB); //io[IO_PPU_STAT] &= 0xFB;
                    else // otherwise we compare LYC to LY (=0 in this case) as usual
                    {
                        if (read8(0xFF00 + IO_PPU_LYC) == 0)
                            write8(0xFF00 + IO_PPU_STAT, read8(0xFF00 + IO_PPU_STAT) | 4);
                        else
                            write8(0xFF00 + IO_PPU_STAT, read8(0xFF00 + IO_PPU_STAT) & 0xFB);

                        // (io[IO_PPU_LYC] == 0) ? (io[IO_PPU_STAT] |= 4) : (io[IO_PPU_STAT] &= 0xFB);
                    }
                }
                else
                {
                    if (read8(0xFF00 + IO_PPU_LY) == read8(0xFF00 + IO_PPU_LYC))
                        write8(0xFF00 + IO_PPU_STAT, read8(0xFF00 + IO_PPU_STAT) | 4);
                    else
                        write8(0xFF00 + IO_PPU_STAT, read8(0xFF00 + IO_PPU_STAT) & 0xFB);

                    // (io[IO_PPU_LY] == io[IO_PPU_LYC]) ? (io[IO_PPU_STAT] |= 4) : (io[IO_PPU_STAT] &= 0xFB);
                }

                // Perform a STAT IRQ if coincidence bit set, coincidence interrupt enabled in STAT, etc
                if ((clks % 456) == 4)
                {
                    if (!(read8(0xFF00 + IO_PPU_LY) == 0 && (read8(0xFF00 + IO_PPU_STAT) & 3) == 2))
                    {
                        if ((read8(0xFF00 + IO_PPU_STAT) & 4) != 0) // coincidence bit set
                        {
                            if ((read8(0xFF00 + IO_PPU_STAT) & 0x40) != 0) // STAT IRQ on LY=LYC set
                                bus->irq(INT_STAT);
                        }
                    }
                }
                // special case, on line 153, irq possibly triggered at clks=12 if LY=LYC=0
                else if ((clks % 456) == 12 && read8(0xFF00 + IO_PPU_LY) == 0 && (read8(0xFF00 + IO_PPU_STAT) & 3) == 1) // io[IO_PPU_LY] == 0 && (io[IO_PPU_STAT] & 3) == 1)
                {
                    if ((read8(0xFF00 + IO_PPU_STAT) & 4) != 0) // coincidence bit set
                    {
                        if ((read8(0xFF00 + IO_PPU_STAT) & 0x40) != 0) // STAT IRQ on LY=LYC set
                            bus->irq(INT_STAT);
                    }
                }
            }
        }

        //(io[IO_PPU_LCDC - PPU_IO_OFFSET] & 0x80) != 0) // && ()) //LCD is enabled && we're rendering (0 <= LY <= 143, STAT mode != 1, etc)
        u8 ly = read8(0xFF00 + IO_PPU_LY), stat = read8(0xFF00 + IO_PPU_STAT), lcdc = read8(0xFF00 + IO_PPU_LCDC);

        if ((lcdc & 0x80) != 0 && ly >= 0 && ly <= 143 && (stat & 3) != 1)  
        {
            //printf("\nPPU rendering, clks = %i, fetcherClks = %i, pxcount = %i, px fifo size = %i, STAT mode = %i", clks, fetcherClksLeft, pxcount, pxfifo.size(), read8(0xFF00 + IO_PPU_STAT) & 3);
            u32 clksRefresh = clks % CLOCK_GB_SCANLINE;

            // Compile the list of sprites to render
            if (clksRefresh == 4)
            {
                // if sprites enabled in lcdc..
            }
            // Rendering
            else if (clksRefresh == 84)  // Just started rendering process, setup a few variables for tile fetching and reset pxcount
            {
                u8 scx = read8(0xFF00 + IO_PPU_SCX), scy = read8(0xFF00 + IO_PPU_SCY), ly = read8(0xFF00 + IO_PPU_LY);
                bool renderStartedWithWindow = false;

                if ((lcdc & 0x20) != 0)  // window enabled
                {
                    u8 wx = read8(0xFF00 + IO_PPU_WX), wy = read8(0xFF00 + IO_PPU_WY);

                    if (wy <= ly)  // window is visible on y axis
                    {
                        if (wx <= 7)  // window is visible on x axis
                        {
                            // Start rendering the window, this is the only thing that is going to be visible (bg covered if enabled)
                            x = wx - 7, y = ly - wy;  // coords of px to render in the (window) map
                            bgMapAddr = ((lcdc & 0x40) != 0) ? 0x9C00: 0x9800;
                            bgMapCol = (x / 8), bgMapRow = (y / 8);
                            bgMapOffset = bgMapRow * 32 + bgMapCol;

                            pxToDiscardX = (abs(x) % 8);
                            bgTileVerticalOffset = (y % 8) * 2;

                            pxcount = 0;
                            fetcherClksLeft = 0;
                            pxUntilRenderWindow = 0xFF;

                            renderStartedWithWindow = true;

                            fetchTile(true);
                        }
                        else  // determine when we'll (presumably) stop rendering bg and start window rendering
                        {
                            pxUntilRenderWindow = (wx - 7);
                        }
                    }
                    else
                    {
                        // Window enabled but not visible
                        pxUntilRenderWindow = 0xFF;
                    }
                }
                else
                {
                    // Window disabled
                    pxUntilRenderWindow = 0xFF;
                }

                if ((lcdc & 1) != 0)  // bg enabled
                {
                    if (!renderStartedWithWindow)  // Rendering process not started by the window rendering process above
                    {
                        // Start rendering the background. If the window is enabled the time to start rendering the window would be set above
                        x = scx, y = ly + scy; // coords of first px to render
                        bgMapAddr = (((lcdc >> 3) & 1) == 0) ? 0x9800 : 0x9C00;
                        bgMapCol = (x / 8) % 32, bgMapRow = (y / 8) % 32; // col/row of tile in bg map
                        bgMapOffset = bgMapRow * 32 + bgMapCol;

                        pxToDiscardX = x % 8;
                        bgTileVerticalOffset = (y % 8) * 2;
                        pxcount = 0;
                        fetcherClksLeft = 0;

                        fetchTile(false);
                    }                    
                }
                else if ((lcdc & 0x20) == 0)  // window and bg disabled               
                    tileDisplayDisabled = true;
            }
            else if (clksRefresh > 84 && pxcount < GB_LCD_XPIXELS)
            {
                if (tileDisplayDisabled)
                {
                    if (++pxcount >= GB_LCD_XPIXELS)
                    {
                        stat &= 0xFC;
                        write8(0xFF00 + IO_PPU_STAT, stat);  // stat mode = h-blank

                        // Trigger STAT IRQ if enabled
                        if ((stat & 8) != 0)
                            bus->irq(INT_STAT);

                        renderScanline();
                    }
                }
                else
                {
                    // update the tile fetcher and pixel FIFO
                    // we'll do fetcher first so px data is ready for the FIFO before it is updated and renders
                    if (fetcherClksLeft == 0)
                    {
                        // Done fetching tile, place in FIFO (if room) then work on next one
                        if (pxfifo.size() <= 8)
                        {
                            for (int j = 0; j < 8; j++)
                            {
                                pxfifo.push(fetchedTile[j]);
                            }

                            // fetch next tile
                            x += 8; // not needed
                            bgMapCol++;
                            bgMapCol %= 32; // allows for wrap around of tile map
                            bgMapOffset = bgMapRow * 32 + bgMapCol;

                            fetchTile(false);
                        }
                    }

                    // and update the pixel FIFO
                    if (pxfifo.size() > 8)
                    {
                        // More than 8 px in FIFO so we're able to push a px to screen
                        if (pxToDiscardX <= 0)
                        {
                            scanline[pxcount++] = pxfifo.front();
                            pxfifo.pop();

                            if (--pxUntilRenderWindow <= 0)  // time to start rendering window
                            {
                                while (!pxfifo.empty()) // must clear fifo then start fetching/rendering window background
                                    pxfifo.pop();

                                u8 wx = read8(0xFF00 + IO_PPU_WX), wy = read8(0xFF00 + IO_PPU_WY);

                                //x = wx - 7, y = ly + wy;
                                if (wy > ly)
                                    printf("\nWY greater than LY!");

                                x = 0, y = ly - wy;
                                bgMapAddr = ((lcdc & 0x40) != 0) ? 0x9C00 : 0x9800;
                                bgMapCol = x / 8, bgMapRow = y / 8;
                                bgMapOffset = bgMapRow * 32 + bgMapCol;

                                pxToDiscardX = 0;
                                bgTileVerticalOffset = (y % 8) * 2;

                                fetcherClksLeft = 0;
                                pxUntilRenderWindow = 0xFF;

                                fetchTile(true);
                            }
                        }
                        else
                        {
                            pxToDiscardX--;
                            pxfifo.pop();
                        }

                        // If done rendering to the LCD
                        if (pxcount >= GB_LCD_XPIXELS)
                        {
                            while (!pxfifo.empty())
                                pxfifo.pop();

                            fetcherClksLeft = 0;

                            stat &= 0xFC;
                            write8(0xFF00 + IO_PPU_STAT, stat);  // stat mode = h-blank

                            // Trigger STAT IRQ if enabled
                            if ((stat & 8) != 0)
                                bus->irq(INT_STAT);

                            renderScanline();
                            //std::cin.ignore(80, '\n');
                        }
                    }

                    // update timing for fetcher
                    if (fetcherClksLeft != 0)
                        fetcherClksLeft--;
                }
            }
        }

        clks++;
    }
}


/**
 * Simply a wrapper for bus->read8PPU(..)
 */
u8 gameboy_ppu::read8(u16 addr)
{
    return bus->read8PPU(addr);
}


/**
 * Simply a wrapper for bus->write8PPU(..)
 */
void gameboy_ppu::write8(u16 addr, u8 n)
{
    bus->write8PPU(addr, n);
}



/**
 * Fetches the tile data referenced in the background map at (bgMapAddr + bgMapOffset).
 * Six cycles are added to fetcherClksLeft to emulate the three reads from VRAM.
 */
void gameboy_ppu::fetchTile(bool windowFetch)
{
    //printf("\nTile Fetch request at BGMap = %X + BGOffset = %X, clks = %i", bgMapAddr, bgMapOffset, clks);
    u8 tileLSB, tileMSB, bgp = read8(0xFF00 + IO_PPU_BGP), lcdc = read8(0xFF00 + IO_PPU_LCDC);

    if (((lcdc >> 4) & 1) == 0) // bg data at $8800, signed tile number
    {
        //printf("\nPrefetch tile num 1");
        s8 tileNum = bus->read8PPU(bgMapAddr + bgMapOffset);
        //printf("\nPostfetch tile num 1");
        //printf("\nTile # (s): %i", tileNum);
        tileLSB = bus->read8PPU(0x9000 + bgTileVerticalOffset + (tileNum * 16));  // pattern 0 lies at $9000
        tileMSB = bus->read8PPU(0x9000 + bgTileVerticalOffset + (tileNum * 16) + 1);
    }
    else // bg data at $8000, unsigned tile number
    {
        //printf("\nBG Map addr = %X ", bgMapAddr + bgMapOffset);
        //printf("\nPrefetch tile num 2");
        u8 tileNum = bus->read8PPU(bgMapAddr + bgMapOffset);
        //printf("\nPostfetch tile num 2");
        //printf(" TileNum = %X", tileNum);
        //printf("\nTile # (u): %i", tileNum);
        tileLSB = bus->read8PPU(0x8000 + bgTileVerticalOffset + tileNum * 16);
        tileMSB = bus->read8PPU(0x8000 + bgTileVerticalOffset + tileNum * 16 + 1);
    }

    // copy data to fetchedTile
    for (int i = 7; i >= 0; i--)
    {
        u8 paletteSelect = ((tileLSB >> i) & 1) | (((tileMSB >> i) & 1) << 1);
        fetchedTile[7 - i] = ((bgp >> (paletteSelect * 2)) & 3);
    }

    fetcherClksLeft += 6;
}


/**
 * Fetches the sprite data from.... (TODO)
 */
void gameboy_ppu::fetchSprite()
{

}


void gameboy_ppu::renderScanline()
{
    u8 cols[4] = {255, 170, 90, 0};
    u8 ly = read8(0xFF00 + IO_PPU_LY), x = 0;

    if (ly == 0)
    {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);
    }

    if (!tileDisplayDisabled)
    {
        for (auto p : scanline)
        {
            auto col = cols[p];

            SDL_SetRenderDrawColor(renderer, col, col, col, 255);
            if (SDL_RenderDrawPoint(renderer, x++, ly) < 0)
                printf("\n\nSDL Error: %s", SDL_GetError());
        }
    }
    else
    {
        SDL_SetRenderDrawColor(renderer, cols[3], cols[3], cols[3], 255);
        if (SDL_RenderDrawLine(renderer, 0, ly, GB_LCD_XPIXELS - 1, ly) < 0)
            printf("\n\nSDL Error: %s", SDL_GetError());
    }

    tileDisplayDisabled = false;

    //printf("\nLY: %i, BGP: %X", ly, read8(0xFF00 + IO_PPU_BGP));
    
    if (ly == GB_LCD_YPIXELS - 1)
        renderScreen();
}


void gameboy_ppu::renderScreen()
{
    SDL_RenderPresent(renderer);
    //SDL_Delay(20);
}

