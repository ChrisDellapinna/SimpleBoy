
/**/

#include <iostream>
#include <fstream>

#include "gameboy_cart.h"
#include "gameboy_common.h"


/**
 * Attempts to load the GameBoy ROM located at path fname.
 * Automatically determines cartridge MBC from cart header (TODO).
 *
 * @param The name/path of the ROM file.
 * @returns true on successful load, false otherwise.
 */
bool gameboy_cart::load(const char* fname)
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
        size = filesize;

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
}


/**
 *
 */
u8 gameboy_cart::read8(u16 addr)
{
    return rom[addr];
}


/**
 *
 */
void gameboy_cart::write8(u16 addr, u8 n)
{

}


