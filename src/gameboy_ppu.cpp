
/**/

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
 * This should (ideally) push 4 pixels while rendering LCD, perform one IO operation on VRAM/OAM
 */
void gameboy_ppu::clock()
{
    // four clocks
    for (int i = 0; i < 4; i++)
    {
        /*
        // check lcd/stat timing
    if ((io[IO_PPU_LCDC] & 0x80) == 0) // Display is off
    {
        ppu_clks = 0;
        io[IO_PPU_STAT] &= 0xFC; // h-blank / display off (Cycle Accurate GameBoy Docs state mode = 0)

        // TODO: Still perform LY/LYC comparison even though LCD off?
    }
    else
    {
        // display on, do normal ppu clocking
        if (ppu_clks % 456 == 0) // If we just moved onto the next line/LY
        {
            if (io[IO_PPU_LY] == 0 && (io[IO_PPU_STAT] & 0x03) == 1) // stat=vblank, on line 153
                io[IO_PPU_STAT] &= 0xFC; // set stat to h-blank (0), leave ly as-is for ln 0
                // POTENTIALLY TRIGGER STAT IRQ HERE?????
            else
                io[IO_PPU_LY]++;

            // LYC Coincidence bit of STAT is always reset on this clock, except when LY=0
            if (io[IO_PPU_LY] != 0)
                io[IO_PPU_STAT] &= 0xFB;
            else
                (io[IO_PPU_LYC] == 0) ? (io[IO_PPU_STAT] |= 4) : (io[IO_PPU_STAT] &= 0xFB);
        }
        else
        {
            //
            if (ppu_clks % 456 == 4)
            {
                if (io[IO_PPU_LY] == 153)  // becomes ly=0, vblank
                    io[IO_PPU_LY] = 0;
                else if (io[IO_PPU_LY] <= 143) // standard display line, stat=oam search (2)
                {
                    io[IO_PPU_STAT] &= 0xFC; // stat should already =0, probably not needed
                    io[IO_PPU_STAT] |= 2;

                    // Trigger STAT IRQ if enabled in STAT
                    if ((io[IO_PPU_STAT] & 0x20) != 0)
                        irq(INT_STAT);
                }
                else if (io[IO_PPU_LY] == 144) // start of vblank
                {
                    io[IO_PPU_STAT] &= 0xFC;
                    io[IO_PPU_STAT] |= 1; // vblank
                    irq(INT_VBLANK);

                    // Trigger STAT IRQ if enabled in STAT
                    if ((io[IO_PPU_STAT] & 0x10) != 0)
                        irq(INT_STAT);
                }

            }
            else if (ppu_clks % 456 == 84 && (io[IO_PPU_STAT] & 3) == 2) // going from oam -> lcd transfer mode
            {
                io[IO_PPU_STAT]++; // stat=3 (lcd transfer)
            }
            else if (ppu_clks % 456 == 252 && (io[IO_PPU_STAT] & 3) == 3) // going from lcd transfer -> hblank
            {
                // THE TIMING FOR THIS IS WRONG (NEED EXACT PPU TIMING)
                io[IO_PPU_STAT] &= 0xFC;

                // Trigger STAT IRQ if enabled
                if ((io[IO_PPU_STAT] & 8) != 0)
                    irq(INT_STAT);
            }

            // Perform the LY/LYC comparison, special case for line 153 (clk 4 compares LYC to 153, clk 8 is always flag reset)
            if (io[IO_PPU_LY] == 0 && (io[IO_PPU_STAT] & 3) == 1)
            {
                if ((ppu_clks % 456) == 4) // special case, LYC compared to 153 not LY's value of 0
                    (io[IO_PPU_LYC] == 153) ? (io[IO_PPU_STAT] |= 4) : (io[IO_PPU_STAT] &= 0xFB);
                else if ((ppu_clks % 456) == 8) // special case, coincidence bit is always reset
                    io[IO_PPU_STAT] &= 0xFB;
                else // otherwise we compare LYC to LY (=0 in this case) as usual
                    (io[IO_PPU_LYC] == 0) ? (io[IO_PPU_STAT] |= 4) : (io[IO_PPU_STAT] &= 0xFB);
            }
            else
                (io[IO_PPU_LY] == io[IO_PPU_LYC]) ? (io[IO_PPU_STAT] |= 4) : (io[IO_PPU_STAT] &= 0xFB);

            // Perform a STAT IRQ if coincidence bit set, coincidence interrupt enabled in STAT, etc
            if ((ppu_clks % 456) == 4)
            {
                if (!(io[IO_PPU_LY] == 0 && (io[IO_PPU_STAT] & 3) == 2)) // not on line 0
                {
                    if ((io[IO_PPU_STAT] & 4) != 0) // coincidence bit set
                    {
                        if ((io[IO_PPU_STAT] & 0x40) != 0) // STAT IRQ on LY=LYC set
                            irq(INT_STAT);
                    }
                }
            }
            // special case, on line 153, irq possibly triggered at clks=12 if LY=LYC=0
            else if ((ppu_clks % 456) == 12 && io[IO_PPU_LY] == 0 && (io[IO_PPU_STAT] & 3) == 1)
            {
                if ((io[IO_PPU_STAT] & 4) != 0) // coincidence bit set
                {
                    if ((io[IO_PPU_STAT] & 0x40) != 0) // STAT IRQ on LY=LYC set
                        irq(INT_STAT);
                }
            }
        }
        */


        if ((io[IO_PPU_LCDC - PPU_IO_OFFSET] & 0x80) != 0) // && ()) //LCD is enabled && LY is in rendering range)
        {
            u32 clksRefresh = clks % CLOCK_GB_SCANLINE;

            // Compile the list of sprites to render
            if (clksRefresh == 4)
            {

            }
            // Rendering
            else if (clksRefresh >= 84 && pxcount < GB_LCD_XPIXELS)
            {
                // update the tile fetcher and pixel FIFO
                // we'll do fetcher first so px data is ready for the FIFO before it is updated and renders
                if (clksRefresh == 84) // Just started rendering process
                {
                    // We'll determine the tile in the bg map to start rendering and begin fetching data
                    x = 0, y = io[IO_PPU_LY - PPU_IO_OFFSET]; // coords of first px to render (simplified for now)
                    bgMapCol = x / 8, bgMapRow = y / 8; // col/row of tile in bg map
                    bgMapAddr = (((io[IO_PPU_LCDC - PPU_IO_OFFSET] >> 3) & 1) == 0) ? 0x9800 : 0x9C00;
                    bgMapOffset = bgMapRow * 32 + bgMapCol;

                    fetchTile();
                }
                else
                {
                    // Continue rendering process as normal
                    if (fetcherClksLeft == 0)
                    {
                        // Done fetching tile, place in FIFO (if room) then work on next one
                        if (pxfifo.size() <= 8)
                        {
                            for (int j = 0; j < 8; j++)
                                pxfifo.push(fetchedTile[j]);

                            // fetch next tile
                            x += 8; // not needed
                            bgMapCol++;
                            bgMapCol /= 32; // allows for wrap around of tile map
                            bgMapOffset = bgMapRow * 32 + bgMapCol;

                            fetchTile();
                        }
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

                        pxcount = 0; // not quite, gotta set the STAT mode to h-blank
                        drawScanlineCallback();
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
 * Fetches the tile data referenced in the background map at (bgMapAddr + bgMapOffset).
 * Six cycles are added to fetcherClksLeft to emulate the three reads from VRAM.
 */
void gameboy_ppu::fetchTile()
{
    u8 tileLSB, tileMSB;

    if (((io[IO_PPU_LCDC - PPU_IO_OFFSET] >> 4) & 1) == 0) // bg data at $8800, signed tile number
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
    for (int i = 7; i < 0; i--)
        fetchedTile[i] = ((tileLSB >> i) & 1) | (((tileMSB >> i) & 1) << 1);

    fetcherClksLeft += 6;
}


/**
 * Fetches the sprite data from.... (TODO)
 */
void gameboy_ppu::fetchSprite()
{

}
