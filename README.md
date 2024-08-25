# Games on a STM32 & 132x32 OLED Screen (from scratch!)

### Hardware used:
- MCU: [STM32F401RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- Platform: [Nucleo-64 ](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)

(Note: memory layout is specific to stm32f401re MCU)

### Requirements
- ARM GCC, https://launchpad.net/gcc-arm-embedded - ARM GCC toolchain (Cross compiler/Linker)
- ST link, https://github.com/stlink-org/stlink - ST link tool for command line flashing of MCU

Add these tools to the Path (Windows)

### Building
```
git clone https://github.com/htiaGG/stm32-game-loader-SSD1306.git
cd stm32-game-loader-SSD1306
.\build.bat
```
Currently the batch file builds and flashes the MCU, so make sure the board is connected (through STLINK).
