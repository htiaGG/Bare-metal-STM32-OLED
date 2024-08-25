ECHO OFF
CLS

rmdir .\bin
mkdir .\bin

REM Compiling
arm-none-eabi-gcc -O0 -Wall -c -g -mcpu=cortex-m4 -mthumb main.c -o bin\main.o
arm-none-eabi-gcc -O0 -Wall -c -g -mcpu=cortex-m4 -mthumb starter.c -o bin\starter.o

REM Linking
arm-none-eabi-ld -o bin\prog.elf -T link.ld bin\starter.o bin\main.o
arm-none-eabi-objcopy bin\prog.elf bin\prog.bin -O binary

REM Disassembling
arm-none-eabi-objdump -D -h bin\starter.o > bin\starter.list
arm-none-eabi-objdump -D -h bin\main.o > bin\main.list
arm-none-eabi-objdump -D -h bin\prog.elf > bin\prog.list
arm-none-eabi-nm --numeric-sort bin\prog.elf

st-flash write %WorkingDir%bin\prog.bin 0x08000000

pause
