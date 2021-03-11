# Flux Pump on STM32

## **Hardware**

---
| MCU Architecture | STM32 Cortex-M4 |
| ----------- | ----------- |
| **MCU** | STM32G474RET6 |
| **Dev Board** | ST NUCLEO-G474RE |
| **CPU Speed** | 170 MHz |
| **Flash Memory** | 512 kB |
| **SRAM** | 128 kB |
| **DAC Ouputs** | 3x 12-bit 1MS/s |
| **ADC Inputs** | 42x 12-bit 5MS/s |
| **Timers** | 2x 32-bit, 10x 16-bit, HRTIM|
| **HRTIM Resolution** | 184 ps |
| **Communication Interfaces** | 4x I2C, 4x SPI, 6x USART |

## **Outputs**

---

### **Analog Outputs (DAC)**

**Voltage Range:** 0-3.3V (midpoint of 1.65V)

**Resolution:** 12 bit (0.8mV)

**Max Sample Rate:** 1MSPS

Samples for signal waveforms are held in Look Up Tables stored in SRAM

| Signal | Source | Pin | Parameters |
| ----------- | ----------- | ----------- | ----------- |
| Transformer | DAC2 OUT1 | PA6 | Shape, Frequency, Amplitude |
| Switch 1 | DAC1 OUT1 | PA4 | Shape, Frequency, Amplitude, Burst Width, Burst Centerpoint |
| Switch 2 | DAC1 OUT2 | PA5 | Shape, Frequency, Amplitude, Burst Width, Burst Centerpoint |

### Waveform Shapes

| Shape | Descirption |
| ----------- | ----------- |
| Constant | Sets all of LUT to same value |
| Sine | Uses standard (slow) sin function to calculate sine |
| Cosine | Uses standard (slow) cos function to calculate cosine |
| Triangle | Uses a continuous method using the standard (slow) trig functions |
| Square | Uses a discrete method to produce a square wave |
| Sine_Fast | Uses CMSIS DSP arm_sin_q15 function to calculate sine |
| Cosine_Fast | Uses CMSIS DSP arm_cos_q15 function to calculate cosine |
| Triangle_Fast | Uses a discrete method to produce a triangle wave |

## **Communication**

---

PuTTY is used to connect to the STM32 board via a Serial COM port connected to the LPUART peripheral and is used to communicate commands and information with the MCU.

**Download:** [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html)

Identify which COM port is used by the STLink Virtual COM Port device and connect to it at a baud rate of 4000000 (4MBaud). Force on *Local Echo* and *Local Line Editing* in the terminal emulation settings for CLI style command input.

### Implemented Commands

| Command | Function |
| ----------- | ----------- |
| #echo | STM32 will return "#echo echo" to confirm there is a correctly configured connection |
| #T On | Turns Transformer DAC waveform on |
| #T Off | Turns Transformer DAC waveform off |
| ?T conf | Queries clock configuration for Transformer waveform |
| #S1 On | Turns Switch 1 DAC Waveform on |
| #S1 Off | Turns Switch 1 DAC Waveform off |
| #S2 On | Turns Switch 2 DAC Waveform on |
| #S2 Off | Turns Switch 2 DAC Waveform off |
| #I read | Outputs a cycle's worth of ADC readings from the current sensor |
| ?I zero | Queries average ADC reading from the current sensor for zero current |

## **Development Environment**

---

### **STM32CubeMX**

*STM32CubeMX is a graphical tool that allows a very easy configuration of STM32 microcontrollers and microprocessors, as well as the generation of the corresponding initialization C code for the Arm® Cortex®-M core or a partial Linux® Device Tree for Arm® Cortex®-A core, through a step-by-step process.*

**Download:** [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)

**Required:** [Java SE Runtime Environment 8u45](https://www.oracle.com/java/technologies/javase/javase8-archive-downloads.html) - There is currently a bug which means newer versions of Java than this may cause the program not to open.

Opening the file *FluxPump_G474RE.ioc* in STM32CubeMX gives the configuration of hardware such as Clocks, Timers, GPIO, DACs and ADCs. The MX project is set to create a basic application structure for a Makefile based toolchain. Click "GENERATE CODE" to regenerate the source code if any changes to the hardware configuration are made.

### **Visual Studio Code**

This is used for editing the source code, compiling, programming the MCU and debugging. Various extensions are available to facilitate developing for STM32.

**Download:** [VSCode](https://code.visualstudio.com/)

**Extensions:**

- C/C++ (Microsoft)
- C++ Intellisense (austin)
- Stm32-for-vscode (Bureau Moeilijke Dingen)
- Cortex-Debug (marus25)

**Required:** [GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads), [Make](http://gnuwin32.sourceforge.net/packages/make.htm), [OpenOCD](https://gnutoolchains.com/arm-eabi/openocd/)

Make sure the paths to each of these executables are included in the PATH environment variables to ensure they can be accessed by VSCode.

### Custom Tasks

Static libraries (such as arm_cortexM4lf_math used for fast trig functions) are not currently supported by the *Stm32-for-vscode* extension. The current workaround is the modified makefile *STM32MakeWithLibs.make* which has the neccessary linker commands manually added to include the libraries required. To build using this modified makefile, run the tasks "Build STM32 project with Libraries" or "Build and flash to an STM32 platform with Libraries", listed in ~/.vscode/tasks.json .
