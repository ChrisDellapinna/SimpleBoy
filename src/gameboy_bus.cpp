
/**/


#include <stdio.h>
#include "gameboy_common.h"
#include "gameboy_bus.h"
#include "gameboy_cart.h"


/**
 *
 */
void gameboy_bus::irq(u8 interruptPos)
{
    write8PPU(0xFF0F, read8PPU(0xFF0F) | (1 << interruptPos));
}

/**
 */
gameboy_cart* gameboy_bus::cartridge()
{
    return &cart;
}


/**
 * Reads the byte specified by addr in the CPU's emulated memory map respecting IO restrictions
 * potentially caused by other devices (PPU r/w from OAM/VRAM, OAM DMA from ROM/RAM/etc to OAM).
 * @param The address in the CPU memory map of the byte to be read.
 * @returns The byte read at specified address.
 */
u8 gameboy_bus::read8CPU(u16 addr)
{
    if (addr < 0x8000) // ROM
        return cart.read8(addr);
    else if (addr < 0xA000) // VRAM
        return vram[addr - 0x8000];
    else if (addr < 0xC000) // External (cart) RAM
        return extram[addr - 0xA000];
    else if (addr < 0xE000) // WRAM
        return wram[addr - 0xC000];
    else if (addr < 0xFE00) // WRAM mirror
        return wram[addr - 0xE000];
    else if (addr < 0xFEA0) // OAM
        return oam[addr - 0xFE00];
    else if (addr < 0xFF00) // Unmapped
        return 0xFF;
    else if (addr < 0xFF80) // Various IO ports
    {
        //printf("\nREAD from IO $%X @ %X ", addr, pc);
        return io[addr - 0xFF00];
    }
    else if (addr < 0xFFFF) // HRAM
        return hram[addr - 0xFF80];
    else
        return ier; // Interrupt enable register
}


/**
 *
 *
 */
u8 gameboy_bus::read8PPU(u16 addr)
{
    /*if (addr < 0x8000) // ROM
        return cart.read8(addr);
    else if (addr < 0xA000) // VRAM
        return vram[addr - 0x8000];
    else if (addr < 0xC000) // External (cart) RAM
        return extram[addr - 0xA000];
    else if (addr < 0xE000) // WRAM
        return wram[addr - 0xC000];
    else if (addr < 0xFE00) // WRAM mirror
        return wram[addr - 0xE000];
    else if (addr < 0xFEA0) // OAM
        return oam[addr - 0xFE00];
    else if (addr < 0xFF00) // Unmapped
        return 0xFF;
    else if (addr < 0xFF80) // Various IO ports
    {
        printf("\nREAD from IO $%X @ %X ", addr, pc);
        return io[addr - 0xFF00];
    }
    else if (addr < 0xFFFF) // HRAM
        return hram[addr - 0xFF80];
    else
        return ier; // Interrupt enable register */

    if (addr >= 0x8000 && addr < 0xA000)
        return vram[addr - 0x8000]; // VRAM
    else if (addr >= 0xFE00 && addr < 0xFEA0)
        return oam[addr - 0xFE00]; // OAM
    else if (addr >= 0xFF40 && addr < 0xFF80 || addr == 0xFF00 + IO_INT_IF)
        return io[addr - 0xFF00]; // IO (PPU-related only)
    else if (addr == 0xFFFF)
        return ier;
    else
        printf("\nInvalid read requested by read8PPU(), address = %X\n", addr);

    return 0xFF;
}


/**
 * TODO
 *
 */
u8 gameboy_bus::read8DMA(u16 addr)
{
    return 0;
}


/**
 *
 */
void gameboy_bus::write8CPU(u16 addr, u8 n)
{
    if (addr < 0x8000) // ROM
        cart.write8(addr, n);
    else if (addr < 0xA000) // VRAM
    {
        vram[addr - 0x8000] = n;
    }
    else if (addr < 0xC000) // External (cart) RAM
        return;
    else if (addr < 0xE000) // WRAM
        wram[addr - 0xC000] = n;
    else if (addr < 0xFE00) // WRAM mirror
        wram[addr - 0xE000] = n;
    else if (addr < 0xFEA0) // OAM
        oam[addr - 0xFE00] = n;
    else if (addr < 0xFF00) // Unmapped
        return;
    else if (addr < 0xFF80) // Various IO ports
    {
        //printf("\nWRITE to IO $%X @ %X ", addr, pc);
        switch (addr - 0xFF00)
        {
        case IO_PPU_LCDC:
        {
            if ((io[IO_PPU_LCDC] >> 7) == 1 && (n >> 7) == 0) // turning screen off
            {
                if ((io[IO_PPU_STAT] & 3) != 1) // turning off screen while not in vblank
                    printf("\nWARNING: Screen turned off while not in VBLANK!");
                io[IO_PPU_LY] = 0; // LY fixed at 0 while LCD off
            }
            io[IO_PPU_LCDC] = n;
            break;
        }
        case IO_PPU_STAT: io[IO_PPU_STAT] &= 7; io[IO_PPU_STAT] |= (n & 0xF8); break;
        case IO_PPU_LY: break; // read only
        case IO_PPU_DMA:
        {
            printf("\nOAM DMA occurred! ");
            io[IO_PPU_DMA] = n;

            //oam_dma_active = true;
            //oam_dma_clks = 0;
        }

        // Timer related
        case IO_TIM_DIV: io[IO_TIM_DIV] = 0; divReset = true; break; // all writes reset to 0
        case IO_TIM_TAC: io[IO_TIM_TAC] = (n & 7); break;  // only lower 3 bits r/w

        default: io[addr - 0xFF00] = n; // eh
        }
    }
    else if (addr < 0xFFFF) // HRAM
        hram[addr - 0xFF80] = n;
    else
        ier = n;
}


/**
 *
 */
void gameboy_bus::write8PPU(u16 addr, u8 n)
{
    if (addr >= 0x8000 && addr < 0xA000)
        vram[addr - 0x8000] = n; // VRAM
    else if (addr >= 0xFE00 && addr < 0xFEA0)
        oam[addr - 0xFE00] = n; // OAM
    else if (addr >= 0xFF40 && addr < 0xFF80 || addr == 0xFF00 + IO_INT_IF)
        io[addr - 0xFF00] = n; // IO (PPU-related only)
    else if (addr == 0xFFFF)
        ier = n;
    else
        printf("\nInvalid write requested by write8PPU(), address = %X, n = %X\n", addr, n);
}


/**
 * TODO
 */
void gameboy_bus::write8DMA(u16 addr, u8 n)
{

}


/**
 *
 */
void gameboy_bus::updateDiv(u8 div)
{
    io[IO_TIM_DIV] = div;
}


/**
 *
 */
bool gameboy_bus::isDivReset()
{
    if (divReset)
    {
        divReset = false;  // reset
        return true;
    }
    else
        return false;
}


