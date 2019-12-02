
![SimpleBoy screenshots](https://i.ibb.co/2hFThKK/simpleboy-screenshots.png)

SimpleBoy is a Game Boy emulator which aims to be cleanly and simply coded while providing
good emulation accuracy. Its currently very much a work in progress.

Game Boy is a registered trademark of Nintendo Co., Ltd. The SimpleBoy project is in no way
affiliated with Nintendo. The purpose of this project is purely educational.

## Status
Currently the emulator is being actively developed. CPU is currently designed to be M-Cycle
accurate while PPU is T-Cycle accurate. CPU memory timing is primarily based off findings of
Gekkio and other accuracy focused emulator devs. PPU timing (as of writing) is largely based
off of the timing presented in the Ultimate Game Boy Talk (33c3) on YouTube with some changes/
additions from info found online (again, mainly from info supplied but accuracy oriented
emulator devs).

###### CPU
All instructions emulated with the exception of STOP. HALT behaviour isn't entirely accurate
(fails Blargg's test).

###### Timers
DIV, TAC, TIMA all pass basic tests. None of the "odd" behaviour/edge cases described by the
Cycle Accurate Game Boy Docs are currently emulated however.

###### PPU
BG and Window emulated. Timing hasn't been verified but should be reasonable based on what I've
found online. Sprites/OAM are TODO.

###### APU, Serial, etc
Not currently emulated.

## Testing/Accuracy
There are still a few bugs I haven't yet tracked down stopping some tests (and other ROMs) 
from booting, namely Gekkio's test ROMs.

###### Blargg
- **cpu_instrs** - Passes all tests but several unexpected STOP instructions encountered. Appears
 to be an issue with detecting CGB hardware?
- **interrupt time** - Fails.
- **HALT bug** - Fails. Mostly complete but a couple cases tested by ROM fail.
- **instr_timing** - Passes.
- **mem_timing (1)** - Passes.
- **mem_timing (2)** - Crashes with erroneous reads from cartridge RAM (none present in cart).


###### Gekkio
Not currently running.
