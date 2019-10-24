
/**/

#include "gameboy_common.h"
#include "gameboy_bus.h"
#include "gameboy_ppu.h"



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
                /*else if (clks % 456 == 252 && (read8(0xFF00 + IO_PPU_STAT) & 3) == 3) //(io[IO_PPU_STAT] & 3) == 3) // going from lcd transfer -> hblank
                {
                    // THE TIMING FOR THIS IS WRONG (NEED EXACT PPU TIMING)
                    write8(0xFF00 + IO_PPU_STAT, read8(0xFF00 + IO_PPU_STAT) & 0xFC); // io[IO_PPU_STAT] &= 0xFC;

                    // Trigger STAT IRQ if enabled
                    if ((read8(0xFF00 + IO_PPU_STAT) & 8) != 0)
                        bus->irq(INT_STAT);

                    //if ((io[IO_PPU_STAT] & 8) != 0)
                    //    bus->irq(INT_STAT);
                }*/

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
                /*if (!(io[IO_PPU_LY] == 0 && (io[IO_PPU_STAT] & 3) == 2)) // not on line 0
                {
                    if ((io[IO_PPU_STAT] & 4) != 0) // coincidence bit set
                    {
                        if ((io[IO_PPU_STAT] & 0x40) != 0) // STAT IRQ on LY=LYC set
                            bus->irq(INT_STAT);
                    }
                }
            }*/
            // special case, on line 153, irq possibly triggered at clks=12 if LY=LYC=0
                else if ((clks % 456) == 12 && read8(0xFF00 + IO_PPU_LY) == 0 && (read8(0xFF00 + IO_PPU_STAT) & 3) == 1) // io[IO_PPU_LY] == 0 && (io[IO_PPU_STAT] & 3) == 1)
                {
                    if ((read8(0xFF00 + IO_PPU_STAT) & 4) != 0) // coincidence bit set
                    {
                        if ((read8(0xFF00 + IO_PPU_STAT) & 0x40) != 0) // STAT IRQ on LY=LYC set
                            bus->irq(INT_STAT);
                    }

                    /*if ((io[IO_PPU_STAT] & 4) != 0) // coincidence bit set
                    {
                        if ((io[IO_PPU_STAT] & 0x40) != 0) // STAT IRQ on LY=LYC set
                            bus-> irq(INT_STAT);
                    }*/
                }
            }
        }

        //(io[IO_PPU_LCDC - PPU_IO_OFFSET] & 0x80) != 0) // && ()) //LCD is enabled && we're rendering (0 <= LY <= 143, STAT mode != 1, etc)
        u8 ly = read8(0xFF00 + IO_PPU_LY);
        if ((read8(0xFF00 + IO_PPU_LCDC) & 0x80) != 0 && ly >= 0 && ly <= 143 && (read8(0xFF00 + IO_PPU_STAT) & 3) != 1)  
        {
            printf("\nPPU rendering, clks = %i, fetcherClks = %i, pxcount = %i, px fifo size = %i, STAT mode = %i", clks, fetcherClksLeft, pxcount, pxfifo.size(), read8(0xFF00 + IO_PPU_STAT) & 3);
            u32 clksRefresh = clks % CLOCK_GB_SCANLINE;

            // Compile the list of sprites to render
            if (clksRefresh == 4)
            {

            }
            // Rendering
            else if (clksRefresh == 84)  // Just started rendering process, setup a few variables for tile fetching and reset pxcount
            {
                // We'll determine the tile in the bg map to start rendering and begin fetching data
                x = 0, y = read8(0xFF00 + IO_PPU_LY); // coords of first px to render (simplified for now)
                bgMapCol = (x / 8) % 32, bgMapRow = (y / 8) % 32; // col/row of tile in bg map
                bgMapAddr = (((read8(0xFF00 + IO_PPU_LCDC) >> 3) & 1) == 0) ? 0x9800 : 0x9C00;
                bgMapOffset = bgMapRow * 32 + bgMapCol;

                pxcount = 0;
                fetcherClksLeft = 0;

                fetchTile();
            }
            else if (clksRefresh > 84 && pxcount < GB_LCD_XPIXELS)
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

                        fetchTile();
                    }
                }

                // and update the pixel FIFO
                if (pxfifo.size() > 8)
                {
                    // More than 8 px in FIFO so we're able to push a px to screen
                    scanline[pxcount++] = pxfifo.front();
                    pxfifo.pop();

                    // If done rendering to the LCD
                    if (pxcount >= GB_LCD_XPIXELS)
                    {
                        while (!pxfifo.empty())
                            pxfifo.pop();

                        fetcherClksLeft = 0;

                        write8(0xFF00 + IO_PPU_STAT, read8(0xFF00 + IO_PPU_STAT) & 0xFC);  // stat mode = h-blank

                        // Trigger STAT IRQ if enabled
                        if ((read8(0xFF00 + IO_PPU_STAT) & 8) != 0)
                            bus->irq(INT_STAT);
                        
                        //drawScanlineCallback();
                    }
                }

                // update timing for fetcher
                if (fetcherClksLeft != 0)
                    fetcherClksLeft--;
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
void gameboy_ppu::fetchTile()
{
    printf("\nTile Fetch request at BGMap = %X + BGOffset = %X, clks = %i", bgMapAddr, bgMapOffset, clks);

    u8 tileLSB, tileMSB;

    if (((read8(0xFF00 + IO_PPU_LCDC) >> 4) & 1) == 0) // bg data at $8800, signed tile number
    {
        s8 tileNum = bus->read8PPU(bgMapAddr + bgMapOffset);
        tileLSB = bus->read8PPU(0x8800 + tileNum);
        tileMSB = bus->read8PPU(0x8800 + tileNum + 1);

    }
    else // bg data at $8000, unsigned tile number
    {
        u8 tileNum = bus->read8PPU(bgMapAddr + bgMapOffset);
        tileLSB = bus->read8PPU(0x8000 + tileNum);
        tileMSB = bus->read8PPU(0x8800 + tileNum + 1);
    }

    // copy data to fetchedTile
    for (int i = 7; i >= 0; i--)
    {
        fetchedTile[i] = ((tileLSB >> i) & 1) | (((tileMSB >> i) & 1) << 1);
    }

    fetcherClksLeft += 6;
}


/**
 * Fetches the sprite data from.... (TODO)
 */
void gameboy_ppu::fetchSprite()
{

}