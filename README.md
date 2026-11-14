This is a fork of [MiSTer-devel/PCFX_MiSTer](https://github.com/MiSTer-devel/PCFX_MiSTer) that aims to play Team Innocent: The Point of No Return. Other games may become playable as a side effect.
I don't have a PC-FX, so most of the extra work comes from Mednafen, MAME and what little documentation there is online about the chips of this console.

* Not accurate - Many changes are best guess.
* AI used during development.
* Any changes on this fork are free to take upstream, but due to the first two points, I wouldn't do it.


# NEC PC-FX core for MiSTer (PCFX_VVK fork)

Based on the `dev` branch of [MiSTer-devel/PCFX_MiSTer](https://github.com/MiSTer-devel/PCFX_MiSTer) (release 20260912 plus the later KING BG1-BG3 and scrolling work).


## Installation

Same as upstream:

- The core needs the PC-FX build of Main from [ReverendGumby/Main_MiSTer, branch pcfx](https://github.com/ReverendGumby/Main_MiSTer/tree/pcfx) for CD support (a pre-built binary is in `releases/pcfx/`).
- Copy the core RBF to `_Console/` and the PC-FX BIOS to `games/PCFX/boot.rom`. Tested with `pcfx.rom` (MD5 08e36edbea28a017f79f8d4f7ff9b6d7).
- SDRAM of any size is required.
- A `.sav` file with the name of the CD is mounted as backup RAM.


## Changes from upstream

Video output:
- Standard MiSTer video processing: scandoubler and filters, HDMI scaling options, analog and direct video.
- OSD option "Border": hides the border around the 256-dot picture (left, right and the 5 lines above the picture). Default: hidden.
- The screen is black (not green) while all layers are off, for example at reset.

CD:
- CD-DA playback (NEC commands D8-DE and SCSI-2 audio commands), with the SOUNDBOX output mixer and ADPCM volumes.
- ABORT messages during a data read end the command at once, and the bridge FIFO is cleared.

RAINBOW (HuC6271):
- Runs of null macroblocks (DC code 0Fh) and the null color registers.
- Control register SRT (reset / run).
- Faster DCT decode, block header search as Mednafen, and a decoder reset for each 16-line strip.

KING (HuC6272) and HuC6261:
- KING tells the HuC6261 which BG layer each pixel comes from, for the layer enable, priority, palette offset and cellophane setting.
- More BG color formats: 4 and 16 colors (dot and block mode, with the BAT palette bank) and 64K colors (not verified in a game yet).
- HuC6261: R00, R01 and R04-R07 can be read back (games change single layer bits this way); RAINBOW chroma key (R0A-R0C).
- KING RAM is cleared at core load and at each reset, so no old picture shows.

Other:
- PC-FX timer (I/O F00h-FC0h) and its interrupt.
- A debug serial port on the HPS UART (`/dev/ttyS1`) for development: it logs CD commands and other events, and it accepts controller presses.


## Not done

- PSG (wavetable) sound.
- KING: 16M color block mode, BG subscreens and some microprogram details.
- RAINBOW: horizontal scroll and the block noise filter.
- CD: read speed and read-ahead like a real drive (reads are faster than on hardware), subcode.
