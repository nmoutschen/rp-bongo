# Ferris bongo

This project reads the volume from an audio probe, and switch Ferris's position when the sound reaches a certain threshold.

## Usage guide

```bash
cargo build --releaase
elf2uf2 target/thumbv6m-none-eabi/release/rp-bongo rp-bongo.uf2
# Then boot your Pico in bootsel mode and copy the uf2 file to the drive.
```
