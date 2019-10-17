
/**/

#ifndef HEADER_GAMEBOY_COMMON_H
#define HEADER_GAMEBOY_COMMON_H



/**
 * Define several type aliases that will frequently be used.
*/
typedef unsigned char u8;
typedef signed char s8;
typedef unsigned short u16;
typedef signed short s16;
typedef unsigned int u32;
typedef signed int s32;


/**
 * Define number of right shifts needed to access each flag in AF.
 */
#define FLAG_Z 7  //use enum or constexpr instead of preproc?
#define FLAG_N 6
#define FLAG_H 5
#define FLAG_C 4

#define INT_VBLANK 0
#define INT_STAT 1
#define INT_TIMER 2
#define INT_SERIAL 3
#define INT_JOYPAD 4

 /// IO ports (offset from start of IO address)
#define IO_JOY_P1       0x00
#define IO_SER_SB       0x01
#define IO_SER_SC       0x02

#define IO_TIM_DIV      0x04
#define IO_TIM_TIMA     0x05
#define IO_TIM_TMA      0x06
#define IO_TIM_TAC      0x07

#define IO_INT_IF       0x0F
#define IO_SND_NR10     0x10
#define IO_SND_NR11     0x11
#define IO_SND_NR12     0x12
#define IO_SND_NR13     0x13 // write only
#define IO_SND_NR14     0x14

#define IO_SND_NR21     0x16
#define IO_SND_NR22     0x17
#define IO_SND_NR23     0x18 // write only
#define IO_SND_NR24     0x19
#define IO_SND_NR30     0x1A
#define IO_SND_NR31     0x1B
#define IO_SND_NR32     0x1C
#define IO_SND_NR33     0x1D // write only
#define IO_SND_NR34     0x1E

#define IO_SND_NR41     0x20
#define IO_SND_NR42     0x21
#define IO_SND_NR43     0x22
#define IO_SND_NR44     0x23
#define IO_SND_NR50     0x24
#define IO_SND_NR51     0x25
#define IO_SND_NR52     0x26

// 0xff30 -> 3f is SND WAV RAM

#define IO_PPU_LCDC     0x40
#define IO_PPU_STAT     0x41
#define IO_PPU_SCY      0x42
#define IO_PPU_SCX      0x43
#define IO_PPU_LY       0x44
#define IO_PPU_LYC      0x45
#define IO_PPU_DMA      0x46
#define IO_PPU_BGP      0x47
#define IO_PPU_OBP0     0x48
#define IO_PPU_OBP1     0x49
#define IO_PPU_WY       0x4A
#define IO_PPU_WX       0x4B

#define IO_CBG_KEY1     0x4D

#define IO_PPU_VBK      0x4F

#define IO_PPU_HDMA1    0x51
#define IO_PPU_HDMA2    0x52
#define IO_PPU_HDMA3    0x53
#define IO_PPU_HDMA4    0x54
#define IO_PPU_HDMA5    0x55
#define IO_CGB_RP       0x56

#define IO_PPU_BCPS     0x68
#define IO_PPU_BCPD     0x69
#define IO_PPU_OCPS     0x6A
#define IO_PPU_OCPD     0x6B

#define IO_CGB_SVBK     0x70

#define PPU_IO_OFFSET   0x40


#define CLOCK_GB 4194304
#define CLOCK_CGB 8388608
#define CLOCK_GB_SCREENREFRESH 70224
#define CLOCK_GB_SCANLINE 456

#define GB_LCD_XPIXELS 160
#define GB_LCD_YPIXELS 144


/**
 * Define the general register format used by the GameBoy processor.
 * This particular implementation only works on little endian systems.
 */
union reg
{
    u16 r;
    struct
    {
        u8 lo;
        u8 hi;
    };
};


#endif
