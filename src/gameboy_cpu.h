
/**/


#ifndef HEADER_GAMEBOY_CPU_H
#define HEADER_GAMEBOY_CPU_H


#include "gameboy_bus.h"
#include "gameboy_ppu.h"


/**
 * Class representing the GameBoy CPU
 */
class gameboy_cpu
{
public:
    //gameboy_cpu() {}
    gameboy_cpu(gameboy_bus* b) : bus(b) {}
    ~gameboy_cpu() {}

    bool init();
    void setBus(gameboy_bus* b);
    void setPpu(gameboy_ppu* p);
    void clock();
    //void clock_ppu_lcd();
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
        //gameboy_cart cart;
        //gameboy_ppu ppu;
    gameboy_bus* bus;
    gameboy_ppu* ppu; // unforunately this dependency is needed because of limitations in the clock counting mechanism

    reg af, bc, de, hl;
    u16 pc, sp;

    //u8 ime = 1;

    bool bootromLoaded = false;

    // Clock tracking related
    u32 clks = 0;
    u32 clks_int_enabled = 0;

    u16 timaTimes[4] = {1024, 16, 64, 256};
    u16 clksDivCounter = 0;
    u16 clksTimaCounter = 0;


    // OAM DMA
    bool oam_dma_active = false;
    u32 oam_dma_clks = 0;
    //u16 oam_dma_src;
    //u8 oam_dma_index;

    // Assorted emulated memory
    //u8 rom[0x8000];
    /*u8 vram[0x2000];
    u8 extram[0x2000];
    u8 wram[0x2000];
    u8 oam[0xA0];
    u8 hram[0x7F];
    u8 io[0x80];
    u8 ier;*/

    // Instruction set functions
    //void LD
    void LD_HL_N();
    void LD(u16& n);
    void LD_SPHL();
    void LD_HL_SP_N();
    void LDH_nA();
    void LDH_An();
    void LD_Ann();
    void LD_nnA();
    void LD_nnSP();
    void PUSH(u16& nn);
    void POP(u16& nn);

    void ADD(u8 n);
    void ADC(u8 n);
    void SUB(u8 n);
    void SBC(u8 n);
    void SBC_HL();
    void SBC_N();
    void AND(u8 n);
    void AND_HL8();
    void AND_N();
    void OR(u8 n);
    void OR_HL8();
    void OR_N();
    void XOR(u8 n);
    void CP(u8 n);

    void INC(u8& n);
    void INC_HL8();
    void INC(u16& nn);
    void DEC(u8& n);
    void DEC_HL8();
    void DEC(u16& nn);
    void ADD_HL(u16 nn);

    void CPL();
    void CCF();
    void SCF();

    void RLCA();
    void RLA();
    void RRCA();
    void RRA();
    void SRL(u8& n);
    void SRL();
    void RLC(u8& n);
    void RLC();
    void RL(u8& n);
    void RL();
    void RRC(u8& n);
    void RRC();
    void RR(u8& n);
    void RR();
    void SLA(u8& n);
    void SLA();
    void SRA(u8& n);
    void SRA();

    void BIT(u8 b, u8 r);
    void BIT(u8 b);
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

    void SWAP(u8 &n);
    void SWAP();
};



#endif
