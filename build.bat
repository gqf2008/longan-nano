cargo build --example epd --release --all-features
@REM riscv-none-embed-objcopy.exe -O binary .\target\riscv32imac-unknown-none-elf\release\examples\ferris ferris.bin
@REM riscv-none-embed-objcopy.exe -O binary .\target\riscv32imac-unknown-none-elf\release\examples\blinky blinky.bin
@REM riscv-none-embed-objcopy.exe -O binary .\target\riscv32imac-unknown-none-elf\release\examples\display display.bin
@REM riscv-none-embed-objcopy.exe -O binary .\target\riscv32imac-unknown-none-elf\release\examples\hello_world hello_world.bin
@REM riscv-none-embed-objcopy.exe -O binary .\target\riscv32imac-unknown-none-elf\release\examples\scan scan.bin
riscv-none-embed-objcopy.exe -O binary .\target\riscv32imac-unknown-none-elf\release\examples\epd epd.bin