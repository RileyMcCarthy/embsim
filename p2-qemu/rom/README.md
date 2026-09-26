# The P2 boot ROM and stage-1 loader

`rom_booter_v33k.spin2` is Parallax's published boot-ROM source,
[`ROM_Booter_v33k.spin2`](https://github.com/parallaxinc/propeller/blob/65f74a117e0470b83f492f888e5c55366c51864f/resources/FPGA%20Examples/ROM_Booter_v33k.spin2)
at parallaxinc/propeller `65f74a117e0470b83f492f888e5c55366c51864f` (git blob
`2a323533af69d1c5554574b36a35e3c4821a7215`). It is under the MIT License,
Copyright (c) 2019 Parallax Inc.; the notice is in
[`LICENSE-PARALLAX`](LICENSE-PARALLAX), the repository's `LICENSE.txt` at that
commit. It is trimmed so flexspin can assemble it, and the file's opening
comment records the same changes:

- truncated after upstream line 989, before the hub-exec `orgh` at line 990 —
  the SD second stage, TAQOZ and the debug monitor, which are written in
  PNut-only dialect and are not part of the boot decision this simulation
  exercises;
- lines 229 and 284: `jmp #@_start_sdcard` becomes `jmp #try_serial`;
- lines 537 (`jmp #@_start_taqoz`) and 540 (`jmp #@_start_monitor`) become
  `jmp #.byte`, so the serial escapes loop back to the serial read.

What remains is the entire boot path proper: the RNG seeding, the cog/LUT
self-load, the pull-up decision tree on P59/P60/P61, the bit-banged SPI-flash
loader with its `"Prop"` checksum, and the serial loader.
`rom_booter_v33k.bin` is the assembled form of that file, under the same
licence.

`stage1.spin2` and `stage1.bin` are embsim's own work, under the workspace's
MIT [`LICENSE`](../../LICENSE): the second-stage flash loader, the 1 KB the ROM
loads, verifies and launches. Its SPI primitives follow the ROM's own idiom.
See [`../src/flashimage.rs`](../src/flashimage.rs) for the flash layout it
consumes.

## Rebuilding both

With FlexSpin 7.5.0 (`Version 7.5.0-HEAD-v7.4.3`, the one MaD's PlatformIO
`toolchain-flexcc` ships), from this directory:

```bash
flexspin -2 -o /tmp/rom_full.bin rom_booter_v33k.spin2
python3 -c "open('rom_booter_v33k.bin','wb').write(open('/tmp/rom_full.bin','rb').read()[0xFC000:])"
flexspin -2 -o stage1.bin stage1.spin2
```

The ROM assembles at `$FC000`, so flexspin warns that the whole image exceeds
512 KB; the ROM is that image from `$FC000` on. Both outputs are
byte-identical to the committed files:

| file | bytes | sha256 |
|---|---|---|
| `rom_booter_v33k.bin` | 1376 | `f57727182f59f3a79fab742544a692842898f97a8461fce374dd722f138e4b3e` |
| `stage1.bin` | 164 | `39c97ed19ff6b448dd2d29a3ef4f32c24f0ca510c7759a8a5a20a462f40d01a4` |
