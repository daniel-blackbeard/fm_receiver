@echo off
cd /d "%~dp0"
if not exist build mkdir build

arm-none-eabi-gcc -c -mcpu=cortex-a9 -marm -ffreestanding startup.S -o build\startup.o
if errorlevel 1 exit /b 1

arm-none-eabi-gcc -c -mcpu=cortex-a9 -marm -ffreestanding -O0 -Wall -Wextra main.c -o build\main.o
if errorlevel 1 exit /b 1

arm-none-eabi-gcc -c -mcpu=cortex-a9 -marm -ffreestanding -O0 -Wall -Wextra eth0.c -o build\eth0.o
if errorlevel 1 exit /b 1

arm-none-eabi-ld -T linker.ld build\startup.o build\main.o build\eth0.o -o build\fm_axi_test.elf
if errorlevel 1 exit /b 1

arm-none-eabi-objcopy -O binary build\fm_axi_test.elf build\fm_axi_test.bin

echo === Built build\fm_axi_test.elf ===
