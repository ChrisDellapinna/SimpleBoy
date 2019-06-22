
/**
 * SimpleBoy is a work-in-progress Nintendo GameBoy emulator currently
 * usable on Windows. A GUI, simple debugger and cross-platform compatibility
 * are currently planned.
 *
 * Note this project was originally named 'Just Another GameBoy Emulator' or
 * 'JAGBE' for short before I realized this name was used by another project
 * on GitHub. So if there are any references to the old name, that's why.
 *
 * @author Chris Dellapinna
 *
 */


#include <SDL.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

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


#define CLOCK_GB 4194304
#define CLOCK_CGB 8388608
#define CLOCK_GB_SCREENREFRESH 70224


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


/**
 * Class representing a GameBoy cartridge including the ROM data and I/O interface.
 */
class gameboy_cart
{
    public:
        gameboy_cart() {}
        ~gameboy_cart() {}

        bool load(const char *fname);
        u8 read8(u16 addr);
        void write8(u16 addr, u8 n);

    private:
        u8 *rom;
        u32 size = 0;
};


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


/**
 * Class representing the GameBoy emulator
 */
class gameboy
{
    public:
        gameboy() {}
        ~gameboy() {}

        bool init();
        void clock();
        void processInterrupts();
        void execute();
        u8 read8(u16 addr);
        void write8(u16 addr, u8 n);
        //void step();

        void irq(u8 interruptPos);

        bool isSet(u32 flag);
        void set(u32 flag);
        void reset(u32 flag);

    //private:
        gameboy_cart cart;

        reg af, bc, de, hl;
        u16 pc, sp;

        u8 ime = 1;

        bool bootromLoaded = false;

        // Clock tracking related
        u32 clks = 0;
        u32 clks_int_enabled = 0;
        u32 ppu_clks = 0;

        // OAM DMA
        bool oam_dma_active = false;
        u32 oam_dma_clks = 0;
        //u16 oam_dma_src;
        //u8 oam_dma_index;

        // Assorted emulated memory
        //u8 rom[0x8000];
        u8 vram[0x2000];
        u8 extram[0x2000];
        u8 wram[0x2000];
        u8 oam[0xA0];
        u8 hram[0x7F];
        u8 io[0x80];
        u8 ier;

        // Instruction set functions
        //void LD
        void LD(u16& n);
        void LD_SPHL();
        void LDH_nA();
        void LDH_An();
        void LD_Ann();
        void LD_nnA();
        void PUSH(u16& nn);
        void POP(u16& nn);

        void ADD(u8 n);
        void ADC(u8 n);
        void SUB(u8 n);
        //void SBC(u8 n);
        void AND(u8 n);
        void AND_HL8();
        void AND_N();
        void OR(u8 n);
        void OR_HL8();
        void OR_N();
        void XOR(u8 n);
        void CP(u8 n);

        void INC(u8 &n);
        void INC_HL8();
        void INC(u16& nn);
        void DEC(u8& n);
        void DEC_HL8();
        void DEC(u16& nn);
        void ADD_HL(u16 nn);

        void CPL();
        void CCF();
        void SCF();

        void BIT(u8 b, u8 r);
        void SET(u8 b, u8& r);
        void SET(u8 b);
        void RES(u8 b, u8& r);
        void RES(u8 b);
        

        void JP();
        void JP(bool take);
        void JP_HL();
        void JR();
        void JR(bool take);
        void CALL();
        void CALL(bool take);
        void RST(u8 n);
        void RET();
        void RET(bool take);
        void RETI();

        void DI();
        void EI();
};


/**
 * Checks if the specified flag in F is set.
 *
 * @param[in] The position of the desired flag in F (where 0 is least significant bit).
 * @returns true if the flag shifted out is set, false otherwise.
*/
bool gameboy::isSet(u32 flag)
{
    return ((af.lo >> flag) & 1) == 1;
}


/**
 * Sets the specified flag in F.
 *
 * @param[in] The position of the desired flag in F (where 0 is least significant bit).
*/
void gameboy::set(u32 flag)
{
    af.lo |= (1 << flag);
}


/**
 * Resets the specified flag in F.
 *
 * @param[in] The position of the desired flag in F (where 0 is least significant bit).
*/
void gameboy::reset(u32 flag)
{
    af.lo &= (0xFF ^ (1 << flag));
}


/**
 *
*/
bool gameboy::init()
{
    if (!cart.load("test.gb"))
        return false;

    if (bootromLoaded)
    {
        printf("\nBoot ROM specified.");
    }
    else
    {
        pc = 0x0100;
        sp = 0xFFFE;
        af.r = 0x01B0;
        bc.r = 0x0013;
        de.r = 0x00D8;
        hl.r = 0x014D;
        
        ier = 0;

        io[IO_PPU_SCY] = 0;
        io[IO_PPU_SCX] = 0;
        io[IO_PPU_LYC] = 0;
        io[IO_PPU_BGP] = 0xFC;

        io[IO_PPU_LY] = 146;
        io[IO_PPU_STAT] = 1; //vblank
        io[IO_PPU_LCDC] = 0x80; // display on
    }

    return true;
}


/**
 * Clocks the emulated system four system (one machine) cycles.
 * This occurs during instruction emulation (after a memory access, after processor
 * number crunching). The keyword is this function is executed AFTER these operations.
 *
 * Preconditions:
 * OAM DMA: if a DMA has been requested, oam_dma_active must = true, oam_dma_clks must = 0
 * 
 */
void gameboy::clock()
{
    clks += 4;
    ppu_clks += 4;

    // check lcd/stat timing
    ppu_clks %= CLOCK_GB_SCREENREFRESH; // keep clk count within [0, CLOCK_GB_SCREENREFRESH]

    if ((io[IO_PPU_LCDC] & 0x80) == 0) // Display is off
    {
        ppu_clks = 0;
        io[IO_PPU_STAT] &= 0xFC;
        io[IO_PPU_STAT] |= 1; // v-blank / display off (Cycle Accurate GameBoy Docs state mode = 0)
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
    }

    // process any pending OAM DMA
    // TEST THIS! Not super confident in this. Might have an off by one error (timing).
    if (oam_dma_active)
    {
        if (oam_dma_clks == 0) // clock counter = 0 when DMA transfer is first started
        {
            // Currently just implement the DMA as a one time mem copy.
            // Not sure if this is accurate enough, need more info on bus conflicts
            // and what can and cannot be read by CPU during DMA
            
            if (io[IO_PPU_DMA] < 0x80) // ROM
            {
                for (int i = 0; i < 0xA0; i++)
                {
                    oam[i] = cart.read8((io[IO_PPU_DMA] << 8) + i);
                }
            }
            else if (io[IO_PPU_DMA] < 0xA0) // VRAM
                memcpy(oam, (vram + ((io[IO_PPU_DMA] - 0x80) << 8)), 0xA0);
            else if (io[IO_PPU_DMA] < 0xC0) // External RAM
                printf("\nOAM DMA from Ext RAM not implemented!");
            else if (io[IO_PPU_DMA] < 0xE0) // WRAM
                memcpy(oam, (wram + ((io[IO_PPU_DMA] - 0xC0) << 8)), 0xA0);
            else if (io[IO_PPU_DMA] < 0xF2) // WRAM (mirror); confirm this is max valid OAM value
                memcpy(oam, (wram + ((io[IO_PPU_DMA] - 0xE0) << 8)), 0xA0);
            else
            {
                // Bad DMA address specified, cancel the DMA?
                printf("\nOAM DMA req'd is invalid, $FF46: %X", io[IO_PPU_DMA]);
                oam_dma_active = false;
            }
        }

        oam_dma_clks++;

        // Gekkio's docs specify that OAM DMA takes 162 machine cycles.
        if (oam_dma_clks >= 162)
            oam_dma_active = false;
    }
}


/**
 * Checks for any pending interrupts and triggers them if enabled.
 * This "check" is performed after an instruction has finished executing.
 */
void gameboy::processInterrupts()
{
    if (ime == 1) // interrupts enabled
    {
        u8 irqAndEnabled = io[IO_INT_IF] & ier;
        if ((irqAndEnabled) != 0) // an interrupt(s) are enabled and requesting to be triggered
        {
            // Note this is NOT accurate emulation of how the GB handles and processes
            // interrupts, particularly from a timing perspective
            u16 vector = 0;
            ime = 0; // disable any further interrupts

            if (((irqAndEnabled >> INT_VBLANK) & 1) == 1)
            {
                vector = 0x0040;
                io[IO_INT_IF] ^= (1 << INT_VBLANK);
            }
            else if (((irqAndEnabled >> INT_STAT) & 1) == 1)
            {
                vector = 0x0048;
                io[IO_INT_IF] ^= (1 << INT_STAT);
            }
            else if (((irqAndEnabled >> INT_TIMER) & 1) == 1)
            {
                vector = 0x0050;
                io[IO_INT_IF] ^= (1 << INT_TIMER);
            }
            else if (((irqAndEnabled >> INT_SERIAL) & 1) == 1)
            {
                vector = 0x0058;
                io[IO_INT_IF] ^= (1 << INT_SERIAL);
            }
            else // INT_JOYPAD
            {
                vector = 0x0060;
                io[IO_INT_IF] ^= (1 << INT_JOYPAD);
            }

            // Push current pc to stack then jump
            write8(--sp, ((pc & 0xFF00) >> 8));
            write8(--sp, (pc & 0xFF));
            pc = vector;
        }
    }
}


/**
 * Reads the byte addressed by addr in the gameboy's emulated memory map.
 *
 * @param[in] Address in gameboy memory map to be read from.
 * @returns The byte read.
 */
u8 gameboy::read8(u16 addr)
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
        printf("\nREAD from IO $%X @ %X ", addr, pc);
        return io[addr - 0xFF00];
    }
    else if (addr < 0xFFFF) // HRAM
        return hram[addr - 0xFF80];
    else
        return ier; // Interrupt enable register
}


/**
 * Writes a byte n to address addr in gameboy's emulated memory map.
 *
 */
void gameboy::write8(u16 addr, u8 n)
{
    if (addr < 0x8000) // ROM
        cart.write8(addr, n);
    else if (addr < 0xA000) // VRAM
        vram[addr - 0x8000] = n;
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
        printf("\nWRITE to IO $%X @ %X ", addr, pc);
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

                oam_dma_active = true;
                oam_dma_clks = 0;
            }

            default: io[addr - 0xFF00] = n; // eh
        }
    }
    else if (addr < 0xFFFF) // HRAM
        hram[addr - 0xFF80] = n;
    else
        ier = n;
}


/**
 * Requests an interrupt by setting the bit in IF ($FF0F) at position interruptPos.
 *
 * @param[in] Position of the flag to set in IF.
 */
void gameboy::irq(u8 interruptPos)
{
    io[IO_INT_IF] |= (1 << interruptPos);
}


/**
 * Executes the next instruction pointed to by the program counter.
 * It is assumed (and required) that all clock'ing is performed within the emulated
 * instructions.
 *
*/
void gameboy::execute()
{
    // TODO: LD r1, r2; JR nn/cc nn; 
    u8 op = read8(pc++);

    switch (op)
    {
        case 0x00: clock();  break; // NOP
        case 0x01: LD(bc.r); break; // LD BC, nn
        case 0x02: clock(); write8(bc.r, af.hi); break; // LD (BC), A
        case 0x03: INC(bc.r); break; // INC BC
        case 0x04: INC(bc.hi); break; // INC B
        case 0x05: DEC(bc.hi); break; // DEC B
        case 0x06: clock(); bc.hi = read8(pc++); break; // LD B, n

        case 0x09: ADD_HL(bc.r); break; // ADD HL, BC
        case 0x0A: clock(); af.hi = read8(bc.r); break; // LD A, (BC)
        case 0x0B: DEC(bc.r); break; // DEC BC
        case 0x0C: INC(bc.lo); break; // INC C
        case 0x0D: DEC(bc.lo); break; // DEC C
        case 0x0E: clock(); bc.lo = read8(pc++); break; // LD C, n

        case 0x11: LD(de.r); break; // LD DE, nn
        case 0x12: clock(); write8(de.r, af.hi); break; // LD (DE), A
        case 0x13: INC(de.r); break; // INC DE
        case 0x14: INC(de.hi); break; // INC D
        case 0x15: DEC(de.hi); break; // DEC D
        case 0x16: clock(); de.hi = read8(pc++); break; // LD D, n

        case 0x18: JR(); break; // JR n
        case 0x19: ADD_HL(de.r); break; // ADD HL, DE
        case 0x1A: clock(); af.hi = read8(de.r); break; // LD A, (DE)
        case 0x1B: DEC(de.r); break; // DEC DE
        case 0x1C: INC(de.lo); break; // INC E
        case 0x1D: DEC(de.lo); break; // DEC E
        case 0x1E: clock(); de.lo = read8(pc++); break; // LD E, n

        case 0x20: JR(!isSet(FLAG_Z)); break; // JR NZ
        case 0x21: LD(hl.r); break; // LD HL, nn
        case 0x22: clock(); write8(hl.r--, af.hi); break; // LD (HLI), A
        case 0x23: INC(hl.r); break; // INC HL
        case 0x24: INC(hl.hi); break; // INC H
        case 0x25: DEC(hl.hi); break; // DEC H
        case 0x26: clock(); hl.hi = read8(pc++); break; // LD H, n
        case 0x28: JR(isSet(FLAG_Z)); break; // JR Z
        case 0x29: ADD_HL(hl.r); break; // ADD HL, HL
        case 0x2A: clock(); af.hi = read8(hl.r++); break; // LD A, (HLI)
        case 0x2B: DEC(hl.r); break; // DEC HL
        case 0x2C: INC(hl.lo); break; // INC L
        case 0x2D: DEC(hl.lo); break; // DEC L
        case 0x2E: clock(); hl.lo = read8(pc++); break; // LD L, n
        case 0x2F: CPL(); break; // CPL
        case 0x30: JR(!isSet(FLAG_C)); break; // JR NC
        case 0x31: LD(sp); break; // LD SP, nn
        case 0x32: clock(); write8(hl.r--, af.hi); break; // LD (HLD), S
        case 0x33: INC(sp); break; // INC SP
        case 0x34: INC_HL8(); break; // INC (HL)   (clock handled within member func)
        case 0x35: DEC_HL8(); break; // DEC (HL)

        case 0x37: SCF(); break; // SCF
        case 0x38: JR(isSet(FLAG_C)); break; // JR C
        case 0x39: ADD_HL(sp); break; // ADD HL, SP
        case 0x3A: clock(); af.hi = read8(hl.r--); break; // LD A, (HLD)
        case 0x3B: DEC(sp); break; // DEC SP
        case 0x3C: INC(af.hi); break; // INC A
        
        case 0x3E: clock(); af.hi = read8(pc++); break; // LD A, n
        case 0x3F: CCF(); break; // CCF

        case 0x47: bc.hi = af.hi; break; // LD B, A

        case 0x4F: bc.lo = af.hi; break; // LD C, A

        case 0x57: de.hi = af.hi; break; // LD D, A
        case 0x5F: de.lo = af.hi; break; // LD E, A
        case 0x67: hl.hi = af.hi; break; // LD H, A
        case 0x6F: hl.lo = af.hi; break; // LD L, A
        case 0x77: clock(); write8(hl.r, af.hi); break; // LD (HL), A
        case 0x78: af.hi = bc.hi; break; // LD A, B
        case 0x79: af.hi = bc.lo; break; // LD A, C
        case 0x7A: af.hi = de.hi; break; // LD A, D
        case 0x7B: af.hi = de.lo; break; // LD A, E
        case 0x7C: af.hi = hl.hi; break; // LD A, H
        case 0x7D: af.hi = hl.lo; break; // LD A, L
        case 0x7E: clock(); af.hi = read8(hl.r); break; // LD A, (HL)
        case 0x7F: break; // LD A, A (does nothing)
        case 0x80: ADD(bc.hi); break; // ADD A, B
        case 0x81: ADD(bc.lo); break; // ADD A, C
        case 0x82: ADD(de.hi); break; // ADD A, D
        case 0x83: ADD(de.lo); break; // ADD A, E
        case 0x84: ADD(hl.hi); break; // ADD A, H
        case 0x85: ADD(hl.lo); break; // ADD A, L
        case 0x86: clock(); ADD(read8(hl.r)); break; // ADD A, (HL)
        case 0x88: ADC(bc.hi); break; // ADC A, B
        case 0x89: ADC(bc.lo); break; // ADC A, C
        case 0x8A: ADC(de.hi); break; // ADC A, D
        case 0x8B: ADC(de.lo); break; // ADC A, E
        case 0x8C: ADC(hl.hi); break; // ADC A, H
        case 0x8D: ADC(hl.lo); break; // ADC A, L
        case 0x8E: clock();  ADC(read8(hl.r)); break; // ADC A, (HL)
        case 0x8F: ADC(af.hi); break; // ADC A, A
        case 0x90: SUB(bc.hi); break; // SUB A, B
        case 0x91: SUB(bc.lo); break; // SUB A, C
        case 0x92: SUB(de.hi); break; // SUB A, D
        case 0x93: SUB(de.lo); break; // SUB A, E
        case 0x94: SUB(hl.hi); break; // SUB A, H
        case 0x95: SUB(hl.lo); break; // SUB A, L
        case 0x96: clock(); SUB(read8(hl.r)); break; // SUB A, (HL)

        case 0xA0: AND(bc.hi); break; // AND B
        case 0xA1: AND(bc.lo); break; // AND C
        case 0xA2: AND(de.hi); break; // AND D
        case 0xA3: AND(de.lo); break; // AND E
        case 0xA4: AND(hl.hi); break; // AND H
        case 0xA5: AND(hl.lo); break; // AND L
        case 0xA6: AND_HL8(); break; // AND (HL)
        case 0xA8: XOR(bc.hi); break; // XOR B
        case 0xA9: XOR(bc.lo); break; // XOR C
        case 0xAA: XOR(de.hi); break; // XOR D
        case 0xAB: XOR(de.lo); break; // XOR E
        case 0xAC: XOR(hl.hi); break; // XOR H
        case 0xAD: XOR(hl.lo); break; // XOR L
        case 0xAE: clock(); XOR(read8(hl.r)); break; // XOR (HL)
        case 0xAF: XOR(af.hi); break; // XOR A
        case 0xB0: OR(bc.hi); break; // OR B
        case 0xB1: OR(bc.lo); break; // OR C
        case 0xB2: OR(de.hi); break; // OR D
        case 0xB3: OR(de.lo); break; // OR E
        case 0xB4: OR(hl.hi); break; // OR H
        case 0xB5: OR(hl.lo); break; // OR L
        case 0xB6: OR_HL8(); break; // OR (HL)

        case 0xB8: CP(bc.hi); break; // CP B
        case 0xB9: CP(bc.lo); break; // CP C
        case 0xBA: CP(de.hi); break; // CP D
        case 0xBB: CP(de.lo); break; // CP E
        case 0xBC: CP(hl.hi); break; // CP H
        case 0xBD: CP(hl.lo); break; // CP L
        case 0xBE: clock();  CP(read8(hl.r)); break; // CP (HL)
        case 0xBF: CP(af.hi); break; // CP A
        case 0xC0: RET(!isSet(FLAG_Z)); break; // RET NZ
        case 0xC1: POP(bc.r); break; // POP BC
        case 0xC2: JP(!isSet(FLAG_Z)); break; // JP NZ, nn
        case 0xC3: JP(); break; // JP nn
        case 0xC4: CALL(!isSet(FLAG_Z)); break; // CALL NZ, nn
        case 0xC5: PUSH(bc.r); break; // PUSH BC
        case 0xC6: clock(); ADD(read8(pc++)); break; // ADD A, n
        case 0xC7: RST(0x00); break; // RST 00
        case 0xC8: RET(isSet(FLAG_Z)); break; // RET Z
        case 0xC9: RET(); break; // RET
        case 0xCA: JP(isSet(FLAG_Z)); break; // JP Z, nn
        case 0xCB:
        {
            clock(); // clock for first opcode read
            // Out of convenience, we'll breakdown opcodes by encoding rather than as a huge switch
            u8 op = read8(pc++);
            switch (op >> 6)
            {
                case 0: printf("\nUNIMPLEMENTED! -> CB %X", op); break;
                case 1: // BIT
                {
                    u8 b = ((op >> 3) & 7);
                    switch (op & 7) // register
                    {
                        case 0: BIT(b, bc.hi); break; // BIT b, B
                        case 1: BIT(b, bc.lo); break; // BIT b, C
                        case 2: BIT(b, de.hi); break; // BIT b, D
                        case 3: BIT(b, de.lo); break; // BIT b, E
                        case 4: BIT(b, hl.hi); break; // BIT b, H
                        case 5: BIT(b, hl.lo); break; // BIT b, L
                        case 6: clock();  BIT(b, read8(hl.r)); break; // BIT b, (HL)
                        case 7: BIT(b, af.hi); break; // BIT b, A
                    }
                }
                case 2: // RES
                {
                    u8 b = ((op >> 3) & 7);
                    switch (op & 7)
                    {
                        case 0: RES(b, bc.hi); break; // RES b, B
                        case 1: RES(b, bc.lo); break; // RES b, C
                        case 2: RES(b, de.hi); break; // RES b, D
                        case 3: RES(b, de.lo); break; // RES b, E
                        case 4: RES(b, hl.hi); break; // RES b, H
                        case 5: RES(b, hl.lo); break; // RES b, L
                        case 6: RES(b); break; // RES b, (HL)
                    }
                }
                case 3: // SET
                {
                    u8 b = ((op >> 3) & 7);
                    switch (op & 7) // register
                    {
                        case 0: SET(b, bc.hi); break; // SET b, B
                        case 1: SET(b, bc.lo); break; // SET b, C
                        case 2: SET(b, de.hi); break; // RES b, D
                        case 3: SET(b, de.lo); break; // RES b, E
                        case 4: SET(b, hl.hi); break; // RES b, H
                        case 5: SET(b, hl.lo); break; // RES b, L
                        case 6: SET(b); break; // RES b, (HL)
                    }
                }
            }
            //printf("\nUnimplemented CB prefix instruction encountered at %X : CB %X", pc, read8(pc + 1));
            break;
        }
        case 0xCC: CALL(isSet(FLAG_Z)); break; // CALL Z, nn
        case 0xCD: CALL(); break; // CALL nn
        case 0xCE: clock(); ADC(read8(pc++)); break; // ADC A, n
        case 0xCF: RST(0x08); break; // RST 08
        case 0xD0: RET(!isSet(FLAG_C)); break; // RET NC
        case 0xD1: POP(de.r); break; // POP DE
        case 0xD2: JP(!isSet(FLAG_C)); break; // JP NC, nn
        case 0xD4: CALL(!isSet(FLAG_C)); break; // CALL NC, nn
        case 0xD5: PUSH(de.r); break; // PUSH DE
        case 0xD6: clock(); SUB(read8(pc++)); break; // SUB n
        case 0xD7: RST(0x10); break; // RST 10
        case 0xD8: RET(isSet(FLAG_C)); break; // RET C
        case 0xD9: RETI(); break; // RETI
        case 0xDA: JP(isSet(FLAG_C)); break; // JP C, nn
        case 0xDC: CALL(isSet(FLAG_C)); break; // CALL C, nn
        case 0xDF: RST(0x18); break; // RST 18
        case 0xE0: LDH_nA(); break; // LDH (n), A
        case 0xE1: POP(hl.r); break; // POP HL
        case 0xE2: clock(); write8(0xFF00 + (u16)bc.lo, af.hi); break; // LD (C), A
        case 0xE5: PUSH(hl.r); break; // PUSH HL
        case 0xE6: AND_N(); break; // AND n (#)
        case 0xE7: RST(0x20); break; // RST 20
        case 0xE9: JP_HL(); break; // JP (HL)
        case 0xEA: LD_nnA(); break; // LD (nn), A
        case 0xEE: clock(); XOR(read8(pc++)); break; // XOR n
        case 0xEF: RST(0x28); break; // RST 28
        case 0xF0: LDH_An(); break; // LDH A, (n)
        case 0xF1: POP(af.r); break; // POP AF
        case 0xF2: clock(); af.hi = read8(pc++); break; // LD A, (C)
        case 0xF3: DI(); break; // DI
        case 0xF5: PUSH(af.r); break; // PUSH AF
        case 0xF6: OR_N(); break; // OR n (imm)
        case 0xF7: RST(0x30); break; // RST 30
        case 0xF9: LD_SPHL(); break;
        case 0xFA: LD_Ann(); break; // LD A, (nn)
        case 0xFB: EI(); break; // EI
        case 0xFE: clock(); CP(read8(pc++)); break; // CP n
        case 0xFF: RST(0x38); break; // RST 38

        default: printf("\nInvalid instruction encountered at %X : %X", pc-1, op); break;
    }

    processInterrupts();
}


/**
 * LD n,nn
 *
 * @param[out] The 16b register to be loaded with the immediate value.
 */
void gameboy::LD(u16& n)
{
    u16 nn = 0;
    clock();
    nn |= read8(pc++);
    clock();
    nn |= (read8(pc++) << 8);
    n = nn;
    clock();
}


/**
 * LD SP,HL
 */
void gameboy::LD_SPHL()
{
    clock();
    sp = hl.r;
    clock();
}


/**
 * LDH (n),A
 */
void gameboy::LDH_nA()
{
    clock();
    u8 n = read8(pc++);
    clock();
    write8(0xFF00 + (u16)n, af.hi);
    clock();
}


/**
 * LDH A,(n)
 */
void gameboy::LDH_An()
{
    clock();
    u8 n = read8(pc++);
    clock();
    af.hi = read8(0xFF00 + (u16)n);
    clock();
}


/**
 * LD A,(nn)
 */
void gameboy::LD_Ann()
{
    u16 addr = 0;
    clock();
    addr |= read8(pc++);
    clock();
    addr |= (read8(pc++) << 8);
    clock();
    af.hi = read8(addr);
    clock();
}


/**
 * LD (nn),A
 */
void gameboy::LD_nnA()
{
    u16 addr = 0;
    clock();
    addr |= read8(pc++);
    clock();
    addr |= (read8(pc++) << 8);
    clock();
    write8(addr, af.hi);
    clock();
}


/**
 * PUSH nn
 *
 * @param[in] The regsiter nn to be pushed onto the stack.
 */
void gameboy::PUSH(u16& nn)
{
    clock();
    clock(); // delay
    write8(--sp, (nn >> 8));
    clock();
    write8(--sp, (nn & 0xFF));
    clock();
}


/**
 * POP nn
 *
 * @param[out] The register nn to store the value popped off the stack.
 */
void gameboy::POP(u16& nn)
{
    clock();
    nn = 0;
    nn |= read8(sp++);
    clock();
    nn |= ((u16)read8(sp++) >> 8);
    clock();
}


/**
 * ADD A,n
 *
 * @param[in] The value n to be added to register A.
*/
void gameboy::ADD(u8 n)
{
    clock(); 
    reset(FLAG_N);

    if ((u16)af.hi + (u16)n > 0xFF)
        set(FLAG_C);
    else
        reset(FLAG_C);

    if ((af.hi & 0x0F) + (n & 0x0F) > 0x0F)
        set(FLAG_H);
    else
        reset(FLAG_H);

    af.hi += n;

    if (af.hi == 0)
        set(FLAG_Z);
    else
        reset(FLAG_Z);
}


/**
 * ADC A,n
 *
 * @param[in] The value n to be added to register A.
*/
void gameboy::ADC(u8 n)
{
    clock();
    u8 c = (isSet(FLAG_C) ? 1 : 0);

    reset(FLAG_N);

    if ((u16)af.hi + (u16)n + (u16)c > 0xFF)
        set(FLAG_C);
    else
        reset(FLAG_C);

    if ((af.hi & 0x0F) + (n & 0x0F) + c > 0x0F)
        set(FLAG_H);
    else
        reset(FLAG_H);

    af.hi += n + c;

    if (af.hi == 0)
        set(FLAG_Z);
    else
        reset(FLAG_Z);
}


/**
 * SUB A,n
 *
 * @param[in] The value n to be subtracted from register A.
 */
void gameboy::SUB(u8 n)
{
    clock();
    set(FLAG_N);

    if ((af.hi & 0x0F) < (n & 0x0F))
        set(FLAG_H);
    else
        reset(FLAG_H);

    if (af.hi < n)  // double check. I think this is backwards. A + (-n) (in twos complement) results in carry with no borrow, no carry otherwise
        set(FLAG_C);
    else
        reset(FLAG_C);

    af.hi -= n;

    if (af.hi == 0)
        set(FLAG_Z);
    else
        reset(FLAG_Z);
}


/**
 * AND n
 *
 * @param[in] The value n to be AND'd to register A.
 */
void gameboy::AND(u8 n)
{
    clock();
    reset(FLAG_N);
    set(FLAG_H);
    reset(FLAG_C);

    af.hi &= n;

    if (af.hi == 0)
        set(FLAG_Z);
    else
        reset(FLAG_Z);
}


/**
 * AND (HL)
 */
void gameboy::AND_HL8()
{
    clock();
    AND(read8(hl.r));
}


/**
 * AND n (#, imm)
 */
void gameboy::AND_N()
{
    clock();
    AND(read8(pc++));
}


/**
 * OR n
 *
 * @param[in] The value n to be OR'd to register A.
 */
void gameboy::OR(u8 n)
{
    clock();
    reset(FLAG_N);
    reset(FLAG_H);
    reset(FLAG_C);

    af.hi |= n;

    if (af.hi == 0)
        set(FLAG_Z);
    else
        reset(FLAG_Z);
}


/**
 * OR (HL)
 */
void gameboy::OR_HL8()
{
    clock();
    OR(read8(hl.r));
}


/**
 * OR n (#, imm)
 */
void gameboy::OR_N()
{
    clock();
    OR(read8(pc++));
}


/**
 * XOR n
 *
 * @param[in] The value n to be XOR'd to register A.
 */
void gameboy::XOR(u8 n)
{
    clock();
    reset(FLAG_N);
    reset(FLAG_H);
    reset(FLAG_C);

    af.hi ^= n;

    if (af.hi == 0)
        set(FLAG_Z);
    else
        reset(FLAG_Z);
}


/**
 * CP n
 *
 * @param[in] The value n to be compared to register A.
 */
void gameboy::CP(u8 n)
{
    clock();
    set(FLAG_N);

    if ((af.hi & 0x0F) < (n & 0x0F))
        set(FLAG_H);
    else
        reset(FLAG_H);

    if (af.hi < n)  // double check. I think this is backwards. A + (-n) (in twos complement) results in carry with no borrow, no carry otherwise
        set(FLAG_C);
    else
        reset(FLAG_C);

    if (af.hi == n)
        set(FLAG_Z);
    else
        reset(FLAG_Z);
}


/**
 * INC n
 *
 * @param[inout] The register n to be incremented 
 */
void gameboy::INC(u8& n)
{
    clock();
    reset(FLAG_N);

    if ((n & 0x0F) + 1 > 0x0F)
        set(FLAG_H);
    else
        reset(FLAG_H);

    n++;

    if (n == 0)
        set(FLAG_Z);
    else
        reset(FLAG_Z);
}


/**
 * INC (HL)    (NOT SURE HOW THIS SHOULD WORK WITH MEM IO RESTRICTIONS/MM IO REGS)
 * Note as this is implementation is a sort of special case, clock'ing is
 * performed within the member function rather than as part of the execute
 * member function as it is usually performed.
 */
void gameboy::INC_HL8()
{
    clock();  // opcode fetch and processing
    u8 n = read8(hl.r);
    clock();  // n fetch

    reset(FLAG_N);

    if ((n & 0x0F) + 1 > 0x0F)
        set(FLAG_H);
    else
        reset(FLAG_H);

    write8(hl.r, ++n);  // clock for this is performed outside in execute

    if (n == 0)
        set(FLAG_Z);
    else
        reset(FLAG_Z);

    clock();
}


/**
 * INC nn   (16 bit register INC)
 *
 * @param[inout] 16b register to increment.
 */
void gameboy::INC(u16& nn)
{
    nn++;
    clock();  // as no memory is effected, doesn't really matter when we clock
    clock();
}


/**
 * DEC n
 *
 * @param[inout] 8b register to decrement.
 */
void gameboy::DEC(u8& n)
{
    set(FLAG_N);

    // VERIFY THAT BORROW IMPLEMENTATION IS CORRECT
    if ((n & 0x0F) == 0) // If bottom 4b = 0, then must borrow from bit/nibble above
        reset(FLAG_H); // H is reset on borrow
    else
        set(FLAG_H);

    n--;

    if (n == 0)
        set(FLAG_Z);
    else
        reset(FLAG_Z);

    clock();
}


/**
 * DEC (HL)
 */
void gameboy::DEC_HL8()
{
    clock();

    u8 n = read8(hl.r);
    clock();
    set(FLAG_N);

    // VERIFY THAT BORROW IMPLEMENTATION IS CORRECT
    if ((n & 0x0F) == 0) // If bottom 4b = 0, then must borrow from bit/nibble above
        reset(FLAG_H); // H is reset on borrow
    else
        set(FLAG_H);

    n--;
    write8(hl.r, n);

    if (n == 0)
        set(FLAG_Z);
    else
        reset(FLAG_Z);

    clock();
}


/**
 * DEC nn   (16 bit register DEC)
 *
 * @param[inout] 16b register to decrement.
 */
void gameboy::DEC(u16& nn)
{
    nn--;
    clock();
    clock();
}


/**
 * ADD HL, nn
 *
 * @param[in] The register nn to be added to HL.
 */
void gameboy::ADD_HL(u16 n)
{
    clock();
    clock();
    reset(FLAG_N);

    if ((u32)hl.r + (u32)n > 0xFFFF)
        set(FLAG_C);
    else
        reset(FLAG_C);

    if ((hl.r & 0x0FFF) + (n & 0x0FFF) > 0x0FFF)
        set(FLAG_H);
    else
        reset(FLAG_H);

    hl.r += n;
}


/**
 * CPL
 */
void gameboy::CPL()
{
    set(FLAG_N);
    set(FLAG_H);
    af.hi ^= 0xFF;
    clock();
}


/**
 * CCF
 */
void gameboy::CCF()
{
    if (isSet(FLAG_C))
        reset(FLAG_C);
    else
        set(FLAG_C);

    clock();
}


/**
 * SCF
 */
void gameboy::SCF()
{
    set(FLAG_C);
    clock();
}


/**
 * BIT b, r
 *
 * @param[in] b The bit to test.
 * @param[in] r The register whose bit is being tested.
 */
void gameboy::BIT(u8 b, u8 r)
{
    reset(FLAG_N);
    set(FLAG_H);

    if (((r >> b) & 1) == 1)
        reset(FLAG_Z);
    else
        set(FLAG_Z);

    clock();
}


/**
 * SET b, r
 */
void gameboy::SET(u8 b, u8& r)
{
    r |= (1 << b);
    clock();
}


/**
 * SET b, (HL)
 */
void gameboy::SET(u8 b)
{
    clock();
    u8 n = read8(hl.r);
    n |= (1 << b);
    clock();
    write8(hl.r, n);
    clock();
}


/**
 * RES b, r
 */
void gameboy::RES(u8 b, u8& r)
{
    r &= (0xFF ^ (1 << b));
    clock();
}


/**
 * RES b, (HL)
 */
void gameboy::RES(u8 b)
{
    clock();
    u8 n = read8(hl.r);
    n &= (0xFF ^ (1 << b));
    clock();
    write8(hl.r, n);
    clock();
}


/**
 * JP nn
 * This instruction takes 16 clock cycles (as per Gekkio docs)
 */
void gameboy::JP()
{
    u16 addr = 0;
    clock();
    addr |= read8(pc++);
    clock();
    addr |= (read8(pc++) << 8);
    pc = addr;
    clock();
    clock();
}


/**
 * JP cc,nn
 * Assuming same timing as JP nn (Gekkio docs)
 */
void gameboy::JP(bool take)
{
    if (take)
    {
        u16 addr = 0;
        clock();
        addr |= read8(pc++);
        clock();
        addr |= (read8(pc++) << 8);
        pc = addr;
        clock();
        clock();
    }
    else
    {
        clock();
        clock();
        clock();
        clock();
        pc += 2;
    }
}


/**
 * JP (HL)
 */
void gameboy::JP_HL()
{
    pc = hl.r;
    clock();
}


/**
 * JR n
 * Based off timing from Gekkio docs and Pandocs.
 */
void gameboy::JR()
{
    clock();
    pc += (s8)read8(pc++);
    clock();
    clock(); // delay
}


/**
 * JR cc, n
 * Based off timing from Gekkio docs and Pandocs.
 */
void gameboy::JR(bool take)
{
    if (take)
        JR();
    else
    {
        clock();
        clock();
        pc++;
    }
}


/**
 * CALL nn
 */
void gameboy::CALL()
{
    u16 addr = 0;
    clock();

    addr |= read8(pc++);
    clock();
    addr |= (read8(pc++) << 8);
    clock();
    clock(); // delay
    write8(--sp, ((pc & 0xFF00) >> 8));
    clock();
    write8(--sp, (pc & 0xFF));
    clock();
    pc = addr;
}


/**
 * CALL cc, nn
 */
void gameboy::CALL(bool take)
{
    clock();

    if (take)
    {
        u16 addr = 0;
        addr |= read8(pc++);
        clock();
        addr |= (read8(pc++) << 8);
        clock();
        clock(); // delay
        write8(--sp, ((pc & 0xFF00) >> 8));
        clock();
        write8(--sp, (pc & 0xFF));
        clock();
        pc = addr;
    }
    else
    {
        pc += 2;
        clock();
        clock();
    }
}


/**
 * RST n
 * Using timing details from Gekkio's docs.
 *
 * @param[in] The position n to restart to.
 */
void gameboy::RST(u8 n)
{
    clock();
    clock(); // delay
    write8(--sp, (pc >> 8));
    clock();
    write8(--sp, (pc & 0xFF));
    pc = 0x0000 + n;
    clock();
}


/**
 * RET
 * Assuming the same timing as RST (pandocs suggest 16clks)
 */
void gameboy::RET()
{
    u16 addr = 0;
    clock();
    clock(); // delay
    addr |= read8(sp++);
    clock();
    addr |= (read8(sp++) << 8);
    pc = addr;
    clock();
}


/**
 * RET cc
 */
void gameboy::RET(bool take)
{
    if (take)
    {
        RET();
        clock();
    }
    else
    {
        clock();
        clock();
    }
}


/**
 * RETI
 */
void gameboy::RETI()
{
    RET();
    //then enable interrupts
    ime = 1;
    printf("\nRETI encountered @ %X", pc);
}


/**
 * DI
 */
void gameboy::DI()
{
    // timing of interrupt disable is WRONG
    clock();
    ime = 0;
    printf("\nDI encountered @ %X", pc);
}


/**
 * EI
 */
void gameboy::EI()
{
    // timing of interrupt enabled is WRONG
    clock();
    ime = 1;
    printf("\nEI encountered @ %X", pc);
}


int main(int argc, char *argv[])
{
    //
    /*SDL_Window *window = NULL;
    SDL_Surface *screen = NULL;

    if (SDL_Init(SDL_INIT_VIDEO))
    {
        printf("Unable to start SDL!\n");
        return -1;
    }

    window = SDL_CreateWindow("SimpleBoy", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 320, 240, SDL_WINDOW_SHOWN);

    if (window == NULL)
    {
        printf("Unable to create SDL windows:  %s\n", SDL_GetError());
        return -1;
    }

    screen = SDL_GetWindowSurface(window);
    SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 255, 255, 255));
    SDL_UpdateWindowSurface(window);

    SDL_Delay(2000);

    SDL_Quit();*/

    // Just some sanity testing.. blah blah..
    gameboy gb;
    if (!gb.init())
    {
        printf("\n\nFailed to init gameboy class. quitting.");
        std::cin.ignore(80, '\n');
        return 1;
    }

    for (int i = 0; i < 600000; i++)
    {
        if (gb.pc == 0x18B)
        {
            for (;;)
            {
                /*printf("\n%X @ %X\tSP: %X  AF: %X  BC: %X  DE: %X  HL: %X \tClks elasped: %i, PPU clks: %i, LY: %i, STAT mode: %i, STAT: %X, LCDC: %X, LYC: %X, IF: %X, IE: %X",
                    gb.read8(gb.pc), gb.pc, gb.sp, gb.af.r, gb.bc.r, gb.de.r, gb.hl.r, gb.clks, gb.ppu_clks, gb.io[IO_PPU_LY], (gb.io[IO_PPU_STAT] & 3),
                    gb.io[IO_PPU_STAT], gb.io[IO_PPU_LCDC], gb.io[IO_PPU_LYC], gb.io[IO_INT_IF], gb.ier);*/
                gb.execute();
            }
        }
        
        gb.execute();
    }

    printf("\n\nEmulation finished. Hit Enter (Return) key to exit.\n");
    std::cin.ignore(80, '\n');

    return 0;
}
