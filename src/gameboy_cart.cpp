
/**/

#include <iostream>
#include <fstream>

#include "gameboy_cart.h"
#include "gameboy_common.h"


gameboy_cart::~gameboy_cart()
{
    if (rom != nullptr)
        delete rom;
    if (sram != nullptr)
        delete sram;
}


/*u32 gameboy_cart::getRomSize()
{
    if (rom == nullptr || romSize == 0)
        return 0;

    u8 romSpec = rom[0x0148];

    if (romSpec < 0x08) // Size of the ROM = 32KiB << romSpec
        return (0x8000 << romSpec);
    else if (romSpec == 0x52) // 1.1MiB (32KiB * 72)
        return (0x8000 * 72);
    else if (romSpec == 0x53) // 1.2MiB (32KiB * 80)
        return (0x8000 * 80);
    else if (romSpec == 0x54) // 1.5MiB (32KiB * 96)
        return (0x8000 * 96);
    else
        return 0;
}


u32 gameboy_cart::getSramSize()
{
    u8 sramSpec = rom[0x0149];

    if (sramSpec == 0)
        return 0;
    else
        if (sramSpec == 1)
            return;
}*/


/**
 * Attempts to load the GameBoy ROM located at path fname.
 * Automatically determines cartridge MBC from cart header (TODO).
 *
 * @param The name/path of the ROM file.
 * @returns true on successful load, false otherwise.
 */
/*bool gameboy_cart::load(const char* fname)
{
    // hard coded for 32K ROM carts 
    std::ifstream romfile;
    std::streampos filesize;

    romfile.open(fname, std::ios::in | std::ios::binary | std::ios::ate);
    if (romfile.is_open())
    {
        printf("\nSuccessfully opened %s... Loading contents...", fname);
        filesize = romfile.tellg();
        romfile.seekg(0, std::ios::beg);

        rom = new u8[filesize];
        romSize = filesize;

        if (filesize < 0x8000)  // make sure we dont read invalid position in test rom
            romfile.read((char*)rom, filesize);
        else
            romfile.read((char*)rom, 0x8000);

        romfile.close();

        printf("\n%s loaded successfully.", fname);
        return true;
    }
    else
    {
        printf("\nUnable to open %s... Quitting.", fname);
        return false;
    }
    return false;
}*/


gameboy_cart_romonly::gameboy_cart_romonly(u8* rom_data, std::streamoff rom_size, bool has_sram, bool has_battery)
{
    hasSRAM = has_sram;
    hasBattery = has_battery;
    rom = rom_data;

    if (sramSize != 0)
    {
        sram = new u8[sramSize];
    }
}


gameboy_cart_romonly::~gameboy_cart_romonly()
{
    //
}


u8 gameboy_cart_romonly::read8(u16 addr)
{
    if (addr < 0x8000)
        return rom[addr];
    else if (addr >= 0xA000 && addr < 0xC000)
    {
        if (sram != nullptr)
        {
            u16 index = addr - 0xA000;

            if (sramSize < index)
                return sram[index];
            else
            {
                printf("\nERROR: Read from out of bounds SRAM location.");
            }
        }
        else
            printf("\nERROR: Read from SRAM when none present in cart.");
    }

    printf("\nERROR: Invalid byte read from SRAM. Returned $FF.");
    return 0xFF;
}


void gameboy_cart_romonly::write8(u16 addr, u8 n)
{
    if (sram != nullptr && addr >= 0xA000 && addr < 0xC000 && sramSize < (addr - 0xA000))
        sram[addr - 0xA000] = n;
}


/**
 * MBC1
 */
gameboy_cart_mbc1::gameboy_cart_mbc1(u8* rom_data, std::streamoff rom_size, bool has_sram, bool has_battery)
{
    hasSRAM = has_sram;
    hasBattery = has_battery;
    rom = rom_data;
    romSize = rom_size;

    if (sramSize != 0)
    {
        sram = new u8[sramSize];
    }
}


gameboy_cart_mbc1::~gameboy_cart_mbc1()
{

}


u8 gameboy_cart_mbc1::read8(u16 addr)
{
    if (addr < 0x4000)
        return rom[addr];
    else if (addr < 0x8000)
    {
        u32 index = (0x4000 * romBankSelected) + (addr - 0x4000);
        //printf("\nREAD ROM: $%X", index);
        return rom[index];
    }
    else if (addr >= 0xA000 && addr < 0xC000)
    {
        if (sram != nullptr)
        {
            u32 index = addr - 0xA000;

            if (sramSize < index)
                return sram[index];
            else
            {
                printf("\nERROR: Read from out of bounds SRAM location.");
            }
        }
        else
            printf("\nERROR: Read from SRAM when none present in cart.");
    }

    printf("\nERROR: Invalid byte read from SRAM. Returned $FF.");
    return 0xFF;
}


void gameboy_cart_mbc1::write8(u16 addr, u8 n)
{
    if (addr < 0x2000) // SRAM enable
    {
        if ((n & 0x0F) == 0x0A)
            sramEnabled = true;
        else
            sramEnabled = false;
    }
    else if (addr < 0x4000) // ROM bank number (lower 5b)
    {
        n &= 0x1F; // mask out lower 5b
        if (n == 0)
            n++;

        romBankSelected &= 0x60;
        romBankSelected |= n;
    }
    else if (addr < 0x6000) // RAM bank number / ROM bank number (upper 2b)
    {
        n &= 3;

        if (romMode)
        {
            romBankSelected &= 0x1F;
            romBankSelected |= (n << 5);
        }
        else
            sramBankSelected = n;
    }
    else if (addr < 0x8000) // ROM/RAM mode select
    {
        ((n &= 1) == 0) ? romMode = true: romMode = false;
    }
    else if (addr >= 0xA000 && addr < 0xC000) // Write to SRAM
    {
        if (sramEnabled)
        {

        }
    }
}


/* 
 * MBC3 
*/


gameboy_cart_mbc3::gameboy_cart_mbc3(u8* rom_data, std::streamoff rom_size, bool has_sram, bool has_battery)
{
    hasSRAM = has_sram;
    hasBattery = has_battery;
    rom = rom_data;
    romSize = rom_size;

    sramBankSelected = 0;
    //sramBankMax = ;

    if (hasSRAM)
    {
        sramSize = getSramSize(rom, romSize);
        if (sramSize == 0)
        {
            printf("\nERROR: MBC3 SRAM size data invalid. SRAM specified but ROM header says SRAM size = 0.");
            hasSRAM = false;
        }
        else
        {
            sram = new u8[sramSize];
        }
    }
}


gameboy_cart_mbc3::~gameboy_cart_mbc3()
{
    
}


u8 gameboy_cart_mbc3::read8(u16 addr)
{
    if (addr < 0x4000)
        return rom[addr];
    else if (addr < 0x8000)
    {
        u32 index = (0x4000 * romBankSelected) + (addr - 0x4000);
        //printf("\nREAD ROM: $%X", index);
        return rom[index];
    }
    else if (addr >= 0xA000 && addr < 0xC000)
    {
        if (hasSRAM && sramEnabled && sram != nullptr)
        {
            if ((addr - 0xA000) < sramSize)
                return sram[sramBankSelected * 0x2000 + (addr - 0xA000)];
        }
        else
            printf("\nERROR: Read from SRAM when none present in cart or SRAM IO disabled.");
    }

    printf("\nERROR: Invalid read from cart requested. Returned $FF.");
    return 0xFF;
}


void gameboy_cart_mbc3::write8(u16 addr, u8 n)
{
    if (addr < 0x2000) // SRAM enable
    {
        if ((n & 0x0F) == 0x0A)
            sramEnabled = true; // enable SRAM and RTC registers
        else
            sramEnabled = false;
    }
    else if (addr < 0x4000) // ROM bank number 
    {
        n &= 0x7F; // mask out the 7b bank
        if (n == 0)
            n++;

        romBankSelected = n;
    }
    else if (addr < 0x6000) // RAM bank number / ROM bank number (upper 2b)
    {
        if (n < 4) // Selecting SRAM bank
        {
            if (sramBankSelected >= (sramSize / 0x2000))
                sramBankSelected = 0;
            else
                sramBankSelected = n;
        }
        else // Selecting RTC register to access
        {

        }
    }
    else if (addr < 0x8000) // ROM/RAM mode select
    {
        if (n == 1 && lastLatch == 0)
        {

        }
        else
            lastLatch = n;
    }
    else if (addr >= 0xA000 && addr < 0xC000) // Write to SRAM
    {
        if (sramEnabled)
        {
            sram[sramBankSelected * 0x2000 + (addr - 0xA000)] = n;
        }
    }
}



gameboy_cart* make_gameboy_cart(const char* romFname)
{
    std::ifstream romFile;
    std::streampos romSize;
    u8 *romData;
    gameboy_cart* cart;

    romFile.open(romFname, std::ios::in | std::ios::binary | std::ios::ate);

    if (romFile.is_open())
    {
        printf("\nSuccessfully opened %s... Loading contents...", romFname);
        romSize = romFile.tellg();
        romFile.seekg(0, std::ios::beg);

        romData = new u8[romSize];

        if (romSize < 0x8000)
        {
            printf("\nROM File is less than 32K in size, invalid. Size = %i", (int)romSize);
            return nullptr;
        }

        romFile.read((char*)romData, romSize);
        romFile.close();

        printf("\n%s loaded successfully.", romFname);

        switch (romData[0x147]) // Determine MBC type
        {
            case 0x00: cart = new gameboy_cart_romonly(romData, romSize); break; // ROM only
            case 0x01: cart = new gameboy_cart_mbc1(romData, romSize); break; // MBC1, no SRAM, no battery
            case 0x02: cart = new gameboy_cart_mbc1(romData, romSize, true); break; // MBC1, SRAM, no battery
            case 0x03: cart = new gameboy_cart_mbc1(romData, romSize, true, true); break; // MBC1, SRAM, battery
            //case 0x05: // MBC2
            //case 0x06: // MBC2, battery
            case 0x08: cart = new gameboy_cart_romonly(romData, romSize, true); break; // ROM only, SRAM, no battery
            case 0x09: cart = new gameboy_cart_romonly(romData, romSize, true, true); break; // ROM only, SRAM, battery
            //
            case 0x0F: cart = new gameboy_cart_mbc3(romData, romSize, false, true); break; // MBC3, timer (RTC?), battery
            case 0x10: cart = new gameboy_cart_mbc3(romData, romSize, true, true); break; // MBC3, timer, SRAM, battery
            case 0x11: cart = new gameboy_cart_mbc3(romData, romSize); break; // MBC3, no timer, no SRAM, no battery
            case 0x12: cart = new gameboy_cart_mbc3(romData, romSize, true, false); break; // MBC3, no timer, SRAM, no battery
            case 0x13: cart = new gameboy_cart_mbc3(romData, romSize, true, true); break; // MBC3, no timer, SRAM, battery

            default: printf("\nUnsupported cartridge type: %X", romData[0x147]); return nullptr; break;
        }

        return cart;
    }
    else
    {
        printf("\nUnable to open %s... Quitting.", romFname);
        return nullptr;
    }
}


u16 getSramSize(u8* rom, std::streamoff size)
{
    if (rom == nullptr || size < 0x150)
        return 0;  // invalid rom data
    else
    {
        switch (rom[0x149])
        {
        case 0: return 0; break;        // none
        case 1: return 0x800; break;    // 2K
        case 2: return 0x2000; break;   // 8K
        case 3: return 0x8000; break;   // 32K

        default: return 0; break;  // invalid sram size
        }
    }
}


