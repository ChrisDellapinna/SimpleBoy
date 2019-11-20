
/**/

#ifndef HEADER_GAMEBOY_CART_H
#define HEADER_GAMEBOY_CART_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <array>

#include "gameboy_common.h"


/**
 * Union that formats the GameBoy ROM header into a more usable format.
 */
union cartridge_header
{
    struct
    {
        u8 entryPoint[4],   // $100 - 103
            logo[0x30];     // $104 - 133
            /*union
            {
                
            };*/
    };
    u8 data[0x50];
};


/**
 * An abstract base class representing a GameBoy cartridge including IO and ROM/SRAM data.
 */
class gameboy_cart
{
public:
    gameboy_cart() {}
    ~gameboy_cart();

    //bool load(const char* fname);
    //u32 getRomSize();
    //u32 getSramSize();

    virtual u8 read8(u16 addr) = 0;
    virtual void write8(u16 addr, u8 n) = 0;

protected:
    u8* rom = nullptr;
    std::streamoff romSize = 0;
    u8* sram = nullptr;
    std::streamoff sramSize = 0;

    bool hasSRAM = false, hasRTC = false, hasBattery = false;
};


/**
 * Class declaration for ROM-only cartridges
 */
class gameboy_cart_romonly : public gameboy_cart
{
public:
    gameboy_cart_romonly(u8 *rom_data, std::streamoff rom_size, bool has_sram=false, bool has_battery=false);
    ~gameboy_cart_romonly();

    u8 read8(u16 addr);
    void write8(u16 addr, u8 n);
};


/**
 * Class declaration for MBC1 cartridges
 */
class gameboy_cart_mbc1 : public gameboy_cart
{
public:
    gameboy_cart_mbc1(u8* rom_data, std::streamoff rom_size, bool has_sram = false, bool has_battery = false);
    ~gameboy_cart_mbc1();

    u8 read8(u16 addr);
    void write8(u16 addr, u8 n);
protected:
    bool sramEnabled = false, romMode = true;
    u16 romBankMax = 0, romBankSelected = 1, sramBankMax = 0, sramBankSelected = 0;
};


/**
 * Class declaration for MBC3 cartridges
 */
class gameboy_cart_mbc3 : public gameboy_cart
{
public:
    gameboy_cart_mbc3(u8 *rom_data, std::streamoff rom_size, bool has_sram = false, bool has_battery = false);
    ~gameboy_cart_mbc3();

    u8 read8(u16 addr);
    void write8(u16 addr, u8 n);

protected:
    bool sramEnabled = false;
    u16 romBankMax = 0, romBankSelected = 1, sramBankMax = 0, sramBankSelected = 0, lastLatch = 0xFFFF;
};


/**
 * Helper function that loads the GameBoy ROM file located at romFname and then
 * creates an instance of, based on cartridge typed specified in the ROM header, 
 * the respective gameboy_cart derived class and returns a pointer to it.
 *
 * @param[in] The filename of the GameBoy ROM to load.
 * 
 */
gameboy_cart* make_gameboy_cart(const char* romFname);



#endif
