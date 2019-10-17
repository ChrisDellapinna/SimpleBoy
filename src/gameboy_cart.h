
/**/

#ifndef HEADER_GAMEBOY_CART_H
#define HEADER_GAMEBOY_CART_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <array>

#include "gameboy_common.h"


/**
 * Class representing a GameBoy cartridge including the ROM data and I/O interface.
 */
class gameboy_cart
{
public:
    gameboy_cart() {}
    ~gameboy_cart() {}

    bool load(const char* fname);
    u8 read8(u16 addr);
    void write8(u16 addr, u8 n);

private:
    u8* rom = nullptr;
    std::streamoff size = 0;
};



#endif
