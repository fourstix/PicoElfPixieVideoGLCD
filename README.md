# PicoElfPixieVideoGLCD
Teensy 3.2 based Pixie Video simulator for the 1802 Pico/Elf v2 microcomputer using a GLCD with SPI

This code simulates a cdp1861 Pixie Video chip, using a [Teensy 3.2.](https://www.pjrc.com/store/teensy32.html)
This [simulator](https://github.com/fourstix/PicoElfPixieVideoGLCD/blob/master/docs/PicoELfPixieVideoGLCD.pdf)
uses a video ram buffer with a 128 x 64 graphics display supported by the
[U8G2 graphics library](https://github.com/olikraus/u8g2) as a video display.  The code will simulate
the interrupts, external flag 1 signal, and DMA Output requests from the original pixie video.  This
allows programs written for the original Cosmac Elf hardware to run directly PicoElf v2 with the simulator.
This simulator supports 32 x 64 and 64 x 64 video resolutions.

The Teensy 3.2 is available from [PJRC.](https://www.pjrc.com/store/teensy32.html) and is also sold by
[Sparkfun](https://www.sparkfun.com/products/13736) and [Adafruit.](https://www.adafruit.com/product/2756)

This hardware design uses a 128 x 64 ST7920 GLCD display with SPI. U8G2 supports this display as well as
many other kinds of 128 x 64 displays.  A list of supported displays is available
[here.](https://github.com/olikraus/u8g2/wiki/u8g2setupcpp)

Based on the Pico/Elf v2 hardware by Mike Riley. Information about the Pico/Elf v2 is available at
[Elf-Emulation.com](http://www.elf-emulation.com/).

Examples
---------------------
Here are some examples running actual CDP1802 programs modified for the Elf/OS on the Pico/Elf v2 Hardware
with the PicoElfVideoGLCD simulator.  These examples were compiled with the [RcAsm 1802 Assmbler](https://github.com/rileym65/RcAsm).

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><img src="https://github.com/fourstix/PicoElfPixieVideoGLCD/blob/master/pics/spaceship.jpg"></td>
   <td><img src="https://github.com/fourstix/PicoElfPixieVideoGLCD/blob/master/pics/dma_test.jpg"></td>
   <td><img src="https://github.com/fourstix/PicoElfPixieVideoGLCD/blob/master/pics/tvclock.jpg"></td>
  </tr>
  <tr align="center">
    <td>Close up of 128x64 ST7920 GLCD display with 1802 Pico/Elf v2 running Cosmac Elf Spaceship program.</td>
    <td>Close up of 128x64 ST7920 GLCD display with 1802 Pico/Elf v2 running Tom Pittman's DMA Test program.</td>
    <td>Close up of 128x64 ST7920 GLCD display with 1802 Pico/Elf v2 running Tom Pittman's TV Clock program.</td>
  </tr>
  <tr align="center">
     <td colspan="3"><img src="https://github.com/fourstix/PicoElfPixieVideoGLCD/blob/master/pics/schematic.jpg"></td>
  </tr>
  <tr align="center">
     <td colspan="3">Pico/Elf Pixie Video GLCD Hardware Schematic</td>
  </tr>
</table>
Notes
-----
* **Video resolution**  
  * Resolutions of 64 x 64 and 32 x 64 are directly supported.
  * For 128 x 64 and other resolutions, video data will be captured on every other DMA request (as per a 64 x 64 resolution).
* **Data Lines**  
  * Data lines from the Pico/Elf Expansion Connector are latched by a 74LS374 Octal D Flip-flip triggered by TPB.
  * The latched Data lines are connected to the Teensy 3.2 Port D pins (pins 2, 14, 7, 8, 6, 20, 21, 5)
  * The Teensy 3.2 reads data as a byte in a single instruction from Port D during an 1802 DMA Output cycle.
  * Full details about Teensy 3.2 pin connections are in the code comments.
* **Video Control**    
  * The 1802 Instruction INP 1 (Input from Port 1,1802 Opcode 69) will turn video processing ON.
  * The 1802 Instruction OUT 1 (Output to Port 1, 1802 Opcode 61) will turn video processing OFF.
  * The /EF1 line will go LOW four lines before a frame's DMA requests begin, and during the last four DMA request lines of a frame.
  * An Interrupt Request will be asserted (/INT = LOW) exactly 29 instruction cycles before the first DMA request
  * 128 lines of 8 DMA_OUT requests will be asserted per frame. Each DMA_OUT request takes one instruction cycle.
  * Exactly 6 instruction cycles will occur between each line of DMA requests.
  * There are a minimum of 6350 instruction cycles (given a 4MHz clock) between frames to simulate the blanking period.
  * Software for the 1802 that rely on these control timings will work with this simulator. (If not, please open an issue.)
  * Timing details are documented in the code comments.
* **Teensy Port 1 Interrupt**
  * The Port 1, /INP and /Out signals on the Pico/Elf Expansion connector trigger interrupts on the Teensy 3.2 to turn video on or off.
  * Input and Output to Port 1 is used to turn video on or off.
  * INP 1, 1802 opcode 69 (PORT1 = LOW, /INP = LOW) turns video on.
  * OUT 1, 1802 opcode 61 (PORT1 = LOW, /OUT = LOW) turns video off.
* **Teensy TPB Interrupt**  
  * A rising signal on TPB triggers an interrupt on the Teensy 3.2.
  * The Teensy interrupt handler will process one cycle in the video state machine
  * During DMA the interrupt handler will read the video data byte from Port D (pins 2, 14, 7, 8, 6, 20, 21, 5)
  * There are 8 bytes of video data read per frame line.
  * Video data is captured every other DMA line and stored in a Video Buffer
* **Frame rate and GLCD update rate**
  * After one complete frame of data is captured, the GLCD display will be updated.
  * Any SPI 64 x 128 GLCD supported by U8G2 should work as a display.
  * If the captured video data does not change, the display will not be updated for that frame.
  * The GLCD with SPI is usually fast enough to update within the frame, but if not, then during display update, interrupts  and control signals will continue, but data will not be captured for the frame. Programs will run correctly, even when data is not captured.
  * The 1802 will see frames requests at rate of about 61/second.

Repository Contents
-------------------
* **/src/PicoElfPixieVideoGLCD/**  
  * PicoElfPixieVideoGLCD.ino -- Teensy 3.2 based Pixie Video simulator for 1802 Pico/Elf v2 and
  a 128x64 ST7920 SPI graphic LCD display.
* **/src/examples/asm/** -- example programs source files and RcAsm batch files.
  * spaceship.asm -- Joseph Weisbecker's original Elf graphics program modified for the Elf/OS
  * dma_test.asm -- Tom Pittman's DMA Test program modified for the Elf/OS
  * tvclock.asm -- Tom Pittman's DMA Test program modified for the Elf/OS
* **/src/examples/bin/** -- binaries, list files and hex files for example programs  
* **/docs** -- documentation files
  * PicoElfPixieVideoGLCD.pdf -- schematic for Pixie Video simulation logic using a Teensy 3.2
* **/pics/** -- example pictures for readme
* **/brd/** -- Printed Circuit Board layout for PicoELfPixieVideoGLCD
  * PicoElfPixieVideoGLCD-KiCad4.zip -- KiCad4 project files for Pico/Elf Pixie Video GLCD PCB.
  * PicoElfPixieVideoGLCD-gerbers.zip -- Gerber files for Pico/Elf Pixie Video GLCD PCB.

  License Information
  -------------------

  This code is public domain under the MIT License, but please buy me a beer
  if you use this and we meet someday (Beerware).

  References to any products, programs or services do not imply
  that they will be available in all countries in which their respective owner operates.

  Sparkfun, the Sparkfun logo, and other Sparkfun products and services are
  trademarks of the Sparkfun Electronics Corporation, in the United States,
  other countries or both.

  Adafruit, the Adafruit logo, and other Adafruit products and services are
  trademarks of the Adafruit Industries, in the United States,other countries or both.

  PJRC, the PJRC logo, and other PJRC products and services are
  trademarks of the PJRC.com LLC, in the United States,other countries or both.

  Other company, product, or services names may be trademarks or services marks of others.

  All libraries used in this code are copyright their respective authors.

  Universal 8bit Graphics Library
  Copyright (c) 2016-2021, olikraus
  All Rights Reserved

  The Teensy 3.2 hardware and software
  Copyright (c) 2016-2021 by Paul Stoffregen, PJRC.com LLC

  The Pico/Elf v2 1802 microcomputer hardware and software
  Copyright (c) 2004-2021 by Mike Riley.

  Elf/OS and RcAsm Software
  Copyright (c) 2004-2021 by Mike Riley.

  Many thanks to the original authors for making their designs and code available as open source.

  This code, firmware, and software is released under the [MIT License](http://opensource.org/licenses/MIT).

  The MIT License (MIT)

  Copyright (c) 2020 by Gaston Williams

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  **THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.**