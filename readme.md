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

| Signal | Source | Pin | Connector | Parameters |
| ----------- | ----------- | ----------- | ----------- | ----------- |
| Transformer | DAC2 OUT1 | PA6 | CN10,P13 | Shape, Frequency, Amplitude |
| Switch 1 | DAC1 OUT1 | PA4 | CN7,P32 | Shape, Frequency, Amplitude, Burst Width, Burst Center |
| Switch 2 | DAC1 OUT2 | PA5 | CN10,P11 | Shape, Frequency, Amplitude, Burst Width, Burst Center |

### Waveform Shapes

| Shape | Descirption |
| ----------- | ----------- |
| Constant | Sets all of LUT to same value |
| Sine | Uses CMSIS DSP arm_sin_q15 function to calculate sine |
| Cosine | Uses CMSIS DSP arm_cos_q15 function to calculate sine |
| Triangle | Uses a discrete method to produce a triangle wave |
| Square | Uses a discrete method to produce a square wave |
| Sine_Slow | Uses C standard (slow) sin function to calculate cosine |

## **Communication**

---

PuTTY can used to connect to the STM32 board via a Serial COM port connected to the LPUART peripheral and provides a text based terminal for sending commands.

**Download:** [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html)

Identify which COM port is used by the STLink Virtual COM Port device and connect to it at a baud rate of 3953488 (~4MBaud). Force on *Local Echo* and *Local Line Editing* in the terminal emulation settings for CLI style command input.

### Implemented Commands

| Command | Function |
| ----------- | ----------- |
| #echo | STM32 will return "#echo echo" to confirm there is a correctly configured connection |
| #T On | Turns Transformer DAC waveform on |
| #T Off | Turns Transformer DAC waveform off |
| #T s *shape_enum* | Changes Transformer waveform shape to given enum |
| #T a *amplitude* | Changes Transformer waveform amplitude to given value |
| ?T div | Queries Transformer clock divider configuration |
| ?T spc | Queries number of 1MSPS samples per cycle |
| #S1 On | Turns Switch 1 DAC Waveform on |
| #S1 Off | Turns Switch 1 DAC Waveform off |
| #S1 s *shape_enum* | Changes Switch 1 waveform shape to given enum |
| #S1 a *amplitude* | Changes Switch 1 waveform amplitude to given value |
| #S1 f *frequency* | Changes Switch 1 waveform frequency to given value |
| #S1 w *width* | Changes Switch 1 burst width to given width in milliseconds |
| #S1 c *center* | Changes Switch 1 burst center to given point in cycle |
| #S2 On | Turns Switch 2 DAC Waveform on |
| #S2 Off | Turns Switch 2 DAC Waveform off |
| #S2 s *shape_enum* | Changes Switch 2 waveform shape to given enum |
| #S2 a *amplitude* | Changes Switch 2 waveform amplitude to given value |
| #S2 f *frequency* | Changes Switch 2 waveform frequency to given value |
| #S2 w *width* | Changes Switch 2 burst width to given width in milliseconds |
| #S2 c *center* | Changes Switch 2 burst center to given point in cycle |
| #I read | Outputs a cycle's worth of ADC readings from the current sensor |
| ?I offset | Queries current sensor DC offset value |
| ?I conv | Queries current sensor conversion factor between ADC reading and amps |
| ?I pk | Queries current sensor peak amplitude over last cycle |
| ?I rms | Queries current sensor RMS measurement over last cycle |
| ?P duty | Queries the current duty cycle of the PWM signal |

Also provided in the folder *Matlab UART* are functions to interface with the MCU in a scripted manner. Raw integer data from the MCU are stored in matlab arrays and can be graphed as neccessary.

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

### Static Libraries

The static library arm_cortexM4lf_math is used for fast trig functions as part of the CMSIS DSP library. And must be added to the STM32-for-VSCode.config.yaml file to ensure proper compilation without errors.
