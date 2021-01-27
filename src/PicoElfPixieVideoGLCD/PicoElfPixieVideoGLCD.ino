/*
 * A Teensy 3.2 based Pixie Video simulator for 
 * the 1802 Pico/Elf v2 microcomputer using a
 * generic 128X64 ST7920 Graphics LCD with SPI.
 * 
 * Copyright (c) 2021 by Gaston Williams
 * 
 * The Teensy 3.2 is available from PJRC, here:
 * https://www.pjrc.com/store/teensy32.html
 * 
 * The Teensy 3.2 is also sold by Sparkfun and Adafruit:
 * https://www.sparkfun.com/products/13736
 * https://www.adafruit.com/product/2756
 * 
 * The Teensy 3.2 hardware and software
 * Copyright (c) 2016-2021 by Paul Stoffregen, PJRC.com LLC
 * 
 * Based on the Pico/Elf v2 hardware by Mike Riley. 
 * Information about the Pico/Elf v2 is available here: 
 * http://www.elf-emulation.com/
 * 
 * The Pico/Elf v2 1802 microcomputer
 * Copyright (c) 2004-2021 by Mike Riley.
 * 
 * This code simulates a cdp1861 Pixi video chip to load a video
 * ram buffer and then updates a 128 x 64 GLCD display using SPI.  
 * The 1802 EF1 flag, interrupts and dma lines are all manipulated 
 * as in a real cdp1861 chip running in 64 x 64 resolution.
 *
 * The U8G2 graphics library is available at:
 * https://github.com/olikraus/u8g2
 * 
 * This code uses a 128 x 64 graphics display supported by the U8G2 library
 * as a video display.  U8G2 supports many kinds of 128 x 64 displays.  
 * 
 * Please see https://github.com/olikraus/u8g2/wiki/u8g2setupcpp for a list.
 * 
 * All libraries are copyright their respective authors.
 * 
 * Universal 8bit Graphics Library
 * Copyright (c) 2016-2021 by olikraus@gmail.com
 * All Rights Reserved
 * 
 * The Pico/Elf v2 Hardware Design
 * Copyright (c) 2004-2020 by Mike Riley
 * 
 * The Teensy 3.2 Hardware design
 * Copyright (c) 2016-2020 by Paul Stoffregen, PJRC.com LLC
 * 
 * Many thanks to the original authors for making their designs and code avaialble.
 * 
 * Note: Serial communication can cause glitches.  Turn off Debugging and unplug 
 *       the USB cable for best results. for this reason debug comments are 
 *       turned off and commented out by default.
 *       
 * Changes from MCard1802TeensyPixieVideo, the original I2C version:      
 * 
 * Fix: Removed u8g2.clear() before paint that causes flicker
 * Add: Because there is no reset signal on the Pico/Elf Expansion connector,
 *       add RESET_PIN to external push button to provide video reset function.
 * Fix: Adjust END_BUFFER_CYCLES for Pico/Elf v2 clock speed
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>


/* Set DEBUG to 1 to include debug code
 * Debug statements need to be uncommented because
 * they will affect the timing in the video logic.
 */
#define DEBUG 0

#define BYTE_MASK 0x00FF 

//Display Pins
#define CS_PIN  10
#define RST_PIN 23
//Pins 11, 12, 13 are used by Hardware SPI

//Input Pins
#define VIDEO_ON_PIN   3
#define TPB_PIN        4
#define VIDEO_OFF_PIN  9
#define SC0_PIN       18
#define SC1_PIN       19
//Hardware reset button
#define RESET_BTN_PIN 22

//Output Pins
#define INT_PIN 15
#define DMA_PIN 16
#define EF1_PIN 17

/*
 * States for Pixie Video
 */
//Unitialized off state
#define VIDEO_OFF   0
//Begin a Frame of Video
#define VIDEO_BEGIN 1
//Process DMA for a frame
#define VIDEO_DMA   2
//End a frame of Video
#define VIDEO_END   3
//Start or restart Video 
#define VIDEO_START 4
//Halt Video
#define VIDEO_STOP  5
//Debug State
#define VIDEO_DEBUG 6

//Video state cycles
#define VIDEO_STATE_BEGIN  0
#define VIDEO_INT_BEGIN   29
//cycle 30 Interrupt read
//cycle 31 Interrupt response - 29 cycles to DMA

//cycle 56 End interrupt and EF1 signals - 4 cycles to DMA
#define VIDEO_SIGNALS_END 56
#define VIDEO_DMA_REQ     58
//End the video setup - next cycle is the first DMA cycle
#define VIDEO_SETUP_END   60
/*
 * Old TV's used to blank the screen when returning to the top of
 * the video display.  Some programs rely on this extra time at 
 * the end of each video frame. Other programs use the Pixie Video 
 * interrupts for timing.  We need to wait at the end of the frame 
 * for some amount of time before starting the next video display cycle.   
 * 
 * The Pixie Video interrupt was set to occur 61 times a second on the
 * original hardware, for one interrupt ever 16,393 uSec.  The Pico/Elf v2 
 * clock runs at 4MHz meaning each clock cycle is 0.25 uSec.  Each instruction
 * cycle takes 8 clock cycles, for a time of 8 x 0.25 uSec = 2 uSec.  (If
 * your Pico/Elf v2 runs at a different clock speed than 4MHz, you may need
 * to calculate your own value of END_BUFFER_CYCLES as outlined here.)
 * 
 * The video logic takes 60 cycles at the beginning then has 128 lines 
 * of 14 cycles/line for DMA, for a total of 60 + 128X14 = 1852 cycles, 
 * or 3704 uSec.
 * 
 * Subtracting this from the interrupt interval leaves a wait time of
 * 16,393 - 3704 = 12,689 uSec or 6345 instruction cycles.  We round this
 * value up to 6350 cycles to give a better value that matches the values 
 * obtained by measuring time manually in a clock program.
 * 
 * The wait time for the blank period is rounded up to 6350 instruction cycles 
 * on the Pico/Elf.  For GLCD didplays, we just wait at the end of the frame
 * until that many cycles have passed.  Then we can restart the video logic.
 */
//End of Frame buffer cycles
#define END_BUFFER_CYCLES 6350
//Number of DMA Cycles in a single linesp
#define LINE_DMA_COUNT   8

//Total number of cycles per line
#define MAX_CYCLES_LINE 14

#define LINE_DMA_OFF     6
#define LINE_DMA_ON     12


//Video buffer for 64 x 64 resolution
#define VIDEO_BUFFER_SIZE 512
//Number of rows in a video display
#define DISPLAY_ROWS 64


//end of frame signaled by /EF1 for last 4 lines
#define LINE_SIGNAL_END   123

//Last line in frame 
#define LAST_IN_FRAME 127

//Number of bytes in a display row (64 bits)
#define BYTES_PER_ROW 8

//Bytes in a single raster line
#define RASTER_DATA_SIZE  16

//data bus pins:  D0  D1 D2 D3 D4  D5  D6 D7
byte dataPin[8] = {2, 14, 7, 8, 6, 20, 21, 5}; 

//bytes for data read
byte data_bus = 0x00;

//video flag
boolean video_on = false;

/*
 * variables used both inside and outside of interrupts 
 * must be volatile
 */
 //Buffer for video data
volatile byte video_data[VIDEO_BUFFER_SIZE];
 
//pixie video signal from 1802 Port 1
volatile boolean pixie_video = false;

//flag to redraw display if data has changed
volatile boolean redraw = false;

//Video State machine
volatile byte state = VIDEO_OFF;


/* 
 *  GLCD ST7920 to Arudino Connections for Hardware SPI
 *  Note: Vo is already connected to internal pot on my board.
 *  
 *  GLCD  Pin    Arduino Pin
 *  BLK   20        GND   
 *  BLA   19        +5V
 *  RST   17        23
 *  PSD   15        GND
 *  
 *  E      6        13 MOSI
 *  R/W    5        11 SCK
 *  RS     4        10 CS
 *  
 *  VCC    2        +5V
 *  GND    1        GND
 */

// U8g2 Contructor
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
U8G2_ST7920_128X64_1_HW_SPI u8g2(U8G2_R0, CS_PIN, RST_PIN);

//Lookup table for XBM format bitmap conversion
// In XBM format LSB is the left most bit, 
// so we expand each nibble to a byte and flip bits
const uint8_t lookup_xbm[16] PROGMEM {
  0x00, 0xC0, 0x30, 0xF0, 0x0C, 0xCC, 0x3C, 0xFC,
  0x03, 0xC3, 0x33, 0xF3, 0x0F, 0xCF, 0x3F, 0xFF};

//Clear the dma data buffer
void clearVideoData() {
  for (int i = 0; i < VIDEO_BUFFER_SIZE; i++) {
    video_data[i] = 0x00;
  } //for
} //clearVideoData


//Set up video display
//Must be called in setup routine
void setupVideo() {
  //Slow down the Teensy SPI bus clock
  //Our ST7920 display can only handle an 800kHz clock
  u8g2.setBusClock(800000);

  u8g2.begin();
  u8g2.setBitmapMode(false); //non-transparent
  u8g2.setDrawColor(1); // White
} // setupVideo  

// Paint entire display
void paintDisplay() {
  u8g2.firstPage();  
  do {
    drawDisplay();
  } while( u8g2.nextPage() );
} // paintDisplay

// Clear video buffer and display.
void blankDisplay() {
  clearVideoData();  
  paintDisplay();
} // blankVideo

//Draw the entire display
void drawDisplay() {
  for(int row = 0; row < DISPLAY_ROWS; row++) {
    byte raster_data[16];
 
    //Fill line with expanded data
    for (int i = 0; i < BYTES_PER_ROW; i++) {
      //Get a byte from the video data and transform it for display    
      byte b = video_data[row * BYTES_PER_ROW + i];
      byte hi = (b >> 4) & 0x0F;
      byte lo = b & 0x0F;
  
      // For XBM LSB is the left most, so we expand to byte and flip bits
      raster_data[2*i]   = pgm_read_byte_near(lookup_xbm + hi);
      raster_data[2*i+1] = pgm_read_byte_near(lookup_xbm + lo);  
    } // for i
    
    //Draw line oncce for 64 x 64 resolution
    u8g2.drawXBM(0, row, 128, 1, raster_data);
  } // for row < DISPLAY_ROWS
}  //drawDisplay



//Reset control output pins back to negative logic off states
void resetVideoControls() {
  digitalWrite(INT_PIN, HIGH);
  digitalWrite(DMA_PIN, HIGH);
  digitalWrite(EF1_PIN, HIGH);
} //resetVideoControls

//Start the video
void startVideo() {
  video_on = true;
  //start the state machine  
  state = VIDEO_START;

 u8g2.begin();
  //start watching TPB
  attachInterrupt(TPB_PIN, isrTPB, RISING);  
} //startVideo

//Stop the video
void stopVideo() {
  //Stop watching TPB
  detachInterrupt(TPB_PIN);  

  video_on = false;
 
  //Turn off output pins
  resetVideoControls();

  //clear display and data
  u8g2.clear();
  clearVideoData();

  //stop video state machine
  state = VIDEO_STOP;
} //stopVideo

/*
 * Video state machine
 * 
 * Note that the TPB interrupt occurs near the end of the cycle.
 * Data is held valid for reading, but any changes made in this 
 * cycle will be read in the _NEXT_ cycle.  Then acted upon in
 * the following cycle.  
 * 
 * This is why DMA signals are changed two cycles before. 
 * State codes indicate the next cycle.  So for example. a DMA 
 * acknowledge state code, indicates the next cycle is a DMA cycle.
 */ 
void doVideoState() {
  //counter for instruction cycles
  static unsigned int cycles = 0;
  //flag to reset counter rather than increment
  static boolean reset_counter = false;
  
  //flag to capture a frame of video data
  static boolean capture_frame = false;
  //flag to capture a DMA line of video data
  static boolean capture_line = false;
  //Up to 128 dma lines in a video
  static byte dma_line = 0;
  
  //flag to indicate display data has changed
  static boolean changed = false;

  //static byte new_byte = 0x00; //dma data byte
  
//  #if DEBUG
//    static int refresh_count = 0;
//  #endif

  switch (state) {        
/*
 * Video Begin State
 * 
 * The timing for cycles in this state are critical.  EF1 must be asserted for
 * at least 56 cycles (4 lines of 14 cycles).  The Interrupt request must be
 * raised and repsoned to with exactly 29 cycles before the first DMA cycle.  
 * 
 * Cycle 0(A)
 * 1      2     3     4     5     6     7   8   9  10  11  12  13  14
 * 15     16    17    18    19   20    21  22  23  24  25  26  27  28
 * 29(B)  30(C) 31(D) 32    33   34    35  36  37  38  39  40  41  42
 * 43     44    45    46    47   48    49  50  51  52  53  54  55  56(E)  
 * 57     58(F) 59(G) 60(H) 0(I)
 * 
 * (A) Cycle 0  - /EF1 is on
 * (B) Cycle 29 - Interrupt Begin (/INT is on)
 * (C) Cycle 30 - Interrupt read by 1802
 * (D) Cycle 31 - Interrupt response by 1802 - 29 cycles before first DMA
 * (E) Cycle 56 - Signals End (/EF1 off, /INT off) - 4 cycles before first DMA
 * (F) Cycle 58 - DMA request - /DMA_OUT is on
 * (G) Cycle 59 - DMA read by 1802 (DMA occurs after instruction completes)
 * (H) Cycle 60 - Setup Ends - DMA Acknowledged, counter reset, state = VIDEO_DMA
 * (I) Cycle 0, State VIDEO_DMA - First DMA cycle occurs
 */    
    //Start of video frame
    case VIDEO_BEGIN:  
      if (cycles == VIDEO_STATE_BEGIN) {
//        #if DEBUG
//          Serial.print("Asserting /EF1: ");        
//          Serial.println(cycles);          
//        #endif  
        //raise EF1 4 lines before DMA
        setExternalFlag(true);       
      } else if (cycles == VIDEO_INT_BEGIN) {
        //raise interrupt two lines before DMA
//        #if DEBUG
//          Serial.print("Requesting Interrupt: ");
//          Serial.println(cycles);
//        #endif          
        requestInterrupt(true);
      } else if (cycles == VIDEO_SIGNALS_END) {        
//        #if DEBUG
//          Serial.print("Ending Signals: ");
//          Serial.println(cycles);
//        #endif    
        setExternalFlag(false);
        requestInterrupt(false);        
      } else if (cycles == VIDEO_DMA_REQ) {
        //raise DMA request two cycles before
//        #if DEBUG
//          Serial.print("Requesting DMA: ");
//          Serial.println(cycles);
//        #endif            
        requestDma(true);
      } else if (cycles >= VIDEO_SETUP_END) {
        //wait for dma request to be acknowledged
//        #if DEBUG
//            Serial.print("Video Setup End: ");
//            Serial.println(cycles);
//          #endif  
        if (isDmaAck()) {
          //Next cycle will be a dma cycle so set state
//          #if DEBUG
//            Serial.print("DMA Acknowledged: ");
//            Serial.println(cycles);
//          #endif
        
          state = VIDEO_DMA;        
          reset_counter = true;
          dma_line = 0;
          capture_line = false;         
          //Capture this frame only if finished redrawing display
          capture_frame = !redraw;                 
          changed = false;
          } //if isDmaAck
      } else {          
//            if (isInterrupt()) {
//            #if DEBUG
//                  Serial.print(F("Interrupt: "));
//                  Serial.println(cycles);
//                }
//            #endif
//            } // if isInterrupt
      } // if-else cycles      
    break;
/*
 * Video DMA State
 * 
 * The timing here is also critical.  The logic reads one byte per cycle
 * via DMA requests to the 1802.  Eight bytes are read via DMA per line, 
 * followed by 6 instructions cycles before the start of the next line. 
 * 
 * Only data from the last line of each group of four is captured as video 
 * data for the GLCD display.  But DMA is asserted for eight cycles on every
 * line so that the timing is maintained.  The cycle counter is reset for
 * each line, for 128 lines. The external flag /EF1 is on for the last four
 * lines of the video.
 * 
 * Cycle:
 * 0(A) 1   2   3   4   5   6(B)  7(C)  8(D)  9   10   11   12(E)  13(F)
 * 
 * (A) Cycles 0 to 7 - Eight DMA cycles where one byte is read from the 1802.
 * Video data is captured every fourth line on lines 3, 7, 11,... 119, 123, 127
 * (B) Cycle 6 - DMA request removed
 * (C) Cycle 7 - Last DMA cycle, next 6 cycles are instruction cycles.
 * (D) Cycles 8 to 13 - Six instruction cycles (for three 2-cycle instructions)
 * (E) Cycle 12 - DMA request raised for each line except for line 127.
 * (F) Cycle 13 - End of Line, DMA acknowledged, line counter incremented, cycle
 * counter reset to zero for next line. 
 * 
 * During lines 124 to 127, /EF1 is on.  At the end of line 127, reset cycle 
 * counter and set the state to VIDEO_END 
 * 
 */
    case VIDEO_DMA: 
      //capture data immediately
      //new_byte = GPIOD_PDIR & BYTE_MASK;  
//      #if DEBUG
//        Serial.print("Line:");
//        Serial.print(dma_line);
//        Serial.print(" Cycles: ");
//        Serial.print(cycles);
//        Serial.print(" Capture:"); 
//        Serial.println(capture_line);
//      #endif
      //Record first 8 cycles after DMA request
      if (cycles < LINE_DMA_COUNT) {
        if (capture_line) {
          byte new_byte = GPIOD_PDIR & BYTE_MASK;
          //integer divide
          byte raster = dma_line / 2;  
          //int type because 512 larger than a byte
          int offset = BYTES_PER_ROW * raster + cycles;           
          byte old_byte = video_data[offset];
          //If any byte has changed save it and set flag to update display
          if (new_byte != old_byte) {
            changed = true;
            video_data[offset] = new_byte;
          } // if new_byte != old_byte                    
        } //if capture_line
        
        //on the next to last dma cycle raise /DMA_OUT line high
        if (cycles == LINE_DMA_OFF) {
          //remove DMA request
          requestDma(false);        
        } //if LINE_DMA_OFF
      //After 8 DMA cycles, there are 6 instruction cycles before next line
      } else if (cycles == LINE_DMA_ON) {         
        //assert next dma request on all lines except last one
        if (dma_line < LAST_IN_FRAME) {
          requestDma(true);
        } //if dma_line < LAST_IN_FRAME
      //At the end of the Line the DMA should be acknowledged for the next line
      } else if (cycles > LINE_DMA_ON) {
        /* 
         * At end of each line, check that dma acknowledged for next line.
         * Except there is no dma ack at end of last line, because it's 
         * finished reading all the video data required for the display.
         * If DMA isn't acknowledged either this is the end of the last line 
         * in the frame or otherwise just wait until it is acknowledged.
         */        
        if (isDmaAck()) {
          dma_line++;
          /* 
           * Capture data on last line of each repeated set of lines. 
           * For a 64x64 display resolution, each line of video data is 
           * usually repeated two times.  (For a 32x64 display resolution, 
           * each line of video data is repeated four times.) Only the last 
           * line actually appears on the display, so only the last line 
           * is captured.  To avoid flicker, don't  capture anything, if 
           * the display is still redrawing.
           */
          if (capture_frame) { 
            //Capture last line in set of two repeated lines
            capture_line = (dma_line % 2 == 1);
          } //if capture frame          
          reset_counter = true;          
          if (dma_line > LINE_SIGNAL_END) {
            //set /EF1 low during last 4 lines 
            setExternalFlag(true);
          } //if dma_line > LINE_SIGNAL_END                
        } else if (dma_line >= LAST_IN_FRAME) {            
            //After the end of last line, end the video frame              
            state = VIDEO_END;            
            reset_counter = true; 
            setExternalFlag(false);    
//            #if DEBUG 
//              Serial.println("End of DMA");
//            #endif            
        } //if isDmaAck else if LAST_IN_FRAME
      }//if-else cycles
    break;
    
    //Update the display if needed, then wait and restart
    case VIDEO_END:
      if (cycles == VIDEO_STATE_BEGIN) {   
        //Turn /EF1 flag off
        setExternalFlag(false);
//        #if DEBUG
//          if(!redraw) {
//              if (changed) {
//                Serial.print(F("Changed:"));
//                Serial.println(refresh_count);
//                refresh_count = 0;
//              } else {
//                refresh_count++;
//              } //if-else changed         
//          } //if !redraw
//        #endif
        //Set redraw flag if video data changed
        if (changed) {
          redraw = true;
          changed = false;  
        } //if changed
      } //if VIDEO_STATE_BEGIN
      
//      #if DEBUG
//        Serial.println(F("End of Video Frame."));
//        //Set state to DEBUG
//        state = VIDEO_DEBUG;
//      #endif
                 
    //Wait awhile before starting a new video cycle, some programs depend on this
    if (cycles >= END_BUFFER_CYCLES) {     
      state = VIDEO_BEGIN;
      reset_counter = true;
    } //if cycles >= END_BUFFER_CYCLES
    break;

    //Set up video for first frame to begin
    //on next instruction cycle
    case VIDEO_START:
      //Next cycle should be zero
      reset_counter = true;
      //Clear all variables
      dma_line = 0;
      capture_frame = false;
      capture_line = false;
      //First frame always considered as changed
      changed = false;
      //Begin the first frame
      state = VIDEO_BEGIN;
    break;

//    #if DEBUG
//      case VIDEO_DEBUG:
//        Serial.println(F("DMA data received: "));
//        for (int i = 0; i < VIDEO_BUFFER_SIZE; i++) {
//          if (i % BYTES_PER_ROW == 0) {
//            Serial.println();
//          } //if i % BYTES_PER_ROW
//          //print hex values so they all line up
//          print2hex(video_data[i]);   
//          Serial.print(" ");
//        } //for 
//        Serial.println();          
//        //Only print once
//        state = VIDEO_STOP;
//      break;
//    #endif
     
    //VIDEO_OFF and VIDEO_STOP
    default:
      //do nothing
    break;
  } //switch
  
  //roll over to zero if needed  
  if (reset_counter) {
    cycles = 0;
    reset_counter = false;
  } else {
  cycles++;
  } //if-else reset_counter     
  
  return;
} //doVideoState
 
/*** Debug routines ***/
#if DEBUG
//print on/off status for a boolean variable
void printFlag(boolean flag) {
  if(flag) {
    Serial.print("on");
  } else {
    Serial.print("off");
  }
} //printFlag

//Print video off / on status
void printVideoState () {
    Serial.print(F("Pixie Video is "));
    Serial.println(video_on ? F("on.") : F("off."));
  } //printVideoState
  
// Pretty print two hex digits for a byte value
void print2hex(byte b) {  
  //If single Hex digit
  if (b < 0x10) {
   Serial.print(F("0"));
  } // if b < 0x10
  Serial.print(b, HEX);
}  
#endif

/*** interrupt service routines ***/
//Interrupt routine for TPB
void isrTPB() {
//  //static variables are only initialized once
  static boolean service_busy = false;
  
  //Service interrupt only once per instruction cycle  
  if (!service_busy) {
    //Ignore any repeated interrupts in same cycle
    service_busy = true;    

    //video state machine
    doVideoState();
    
    //Done. So get ready for interrupt in next cycle
    service_busy = false;     
  } //if !service_busy
} //isrTPB

//Interrupt routine for Video On
void isrPort1Input() {
  pixie_video = true;
//  #if DEBUG
//    Serial.println("Port 1 INPUT interrupt");
//  #endif 
} //isrPort1Input

//Interrupt routine for Video Off
void isrPort1Output() {
  pixie_video = false;
//  #if DEBUG
//    Serial.println("Port 1 OUTPUT interrupt");
//  #endif 
} //isrPort1Output

/*** 1861 video control line routines ***/
//Request interrupt from 1802
void requestInterrupt(boolean intReq) {
  //Negative logic: /INT is true when low, false when high
  if (intReq) {
    digitalWriteFast(INT_PIN, LOW);  
  } else {
    digitalWriteFast(INT_PIN, HIGH);  
  } //if intReq
} //requestInterrupt

//Request DMA OUtput from 1802
void requestDma(boolean dmaReq) {
  //Negative logic: /DMA_OUT is true when low, false when high
  if (dmaReq) {
    digitalWriteFast(DMA_PIN, LOW);  
  } else {
    digitalWriteFast(DMA_PIN, HIGH);  
  } //if dmaReq
} //requestDma

//Check to see if interrupt was acknowledged
boolean isInterrupt() {
  boolean sc0 = (digitalReadFast(SC0_PIN) == HIGH);
  boolean sc1 = (digitalReadFast(SC1_PIN) == HIGH);
  boolean interrupt = (sc1 && sc0);
//   #if DEBUG
//        Serial.print(F("SC0:"));
//        Serial.print(sc0);
//        Serial.print(F(" SC1:"));
//        Serial.print(sc1);
//        Serial.print(" IRQ:");
//        Serial.print(interrupt);
//        Serial.println();
//  #endif   
  return interrupt;
}

//check to see if dma request is acknowledged
boolean isDmaAck() {
  boolean sc0 = (digitalReadFast(SC0_PIN) == HIGH);
  boolean sc1 = (digitalReadFast(SC1_PIN) == HIGH);
  boolean dma_status = (sc1 && !sc0);
//   #if DEBUG
//        Serial.print(F("SC0:"));
//        Serial.print(sc0);
//        Serial.print(F(" SC1:"));
//        Serial.print(sc1);
//        Serial.print(" DMA:");
//        Serial.print(dma_status);
//        Serial.println();
//  #endif    
  return dma_status;
}

//set the /EF1 flag for Pixie Video
void setExternalFlag(boolean state) {
  if (state) {
    //Negative logic: On is low, off is high
    digitalWriteFast(EF1_PIN, LOW);
  } else {
    //Negative logic: On is low, off is high
    digitalWriteFast(EF1_PIN, HIGH);
  } //if state
} //setExternalFlag()

/*
 * Pico/Elf  Pico/Elf   Teensy 3.2
 *  Signal   Expansion      Pin
 *    D0        2             2
 *    D1        4            14
 *    D2        6             7 
 *    D3        8             8
 *    D4       10             6
 *    D5       12            20
 *    D6       14            21
 *    D7       16             5
 *  Port1_INP  /(23+19)       3 (VIDEO_ON)
 *    TPB      18             4
 *  PORT1_OUT  /(23+21)       9 (VIDEO_OFF)        
 *    SC0      31            18
 *    SC1      32            19 
 *    INT      28            15
 *  DMA_OUT    27            16
 *    EF1      20            17
 *  
 */
 

 //One-time setup code
void setup(void) {
  #if DEBUG
    Serial.begin(9600);
  #endif

  //Set up all the digital input pins with pullups
  for (int i=0; i < 8; i++) {
    pinMode(dataPin[i], INPUT_PULLUP);
  } //for
  
  //Set up interrupt pins
  pinMode(TPB_PIN, INPUT);
  //interrupt handler managed by start/stop video

  pinMode(VIDEO_ON_PIN, INPUT);
  attachInterrupt(VIDEO_ON_PIN, isrPort1Input, FALLING);

  pinMode(VIDEO_OFF_PIN, INPUT);
  attachInterrupt(VIDEO_OFF_PIN, isrPort1Output, FALLING);

  //Set up State input pins
  pinMode(SC0_PIN, INPUT);
  pinMode(SC1_PIN, INPUT);

  //Set up Hardware reset button pin
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);

  //Set up Output pins with default values
  pinMode(INT_PIN, OUTPUT);
  digitalWrite(INT_PIN, HIGH);
  
  pinMode(DMA_PIN, OUTPUT);
  digitalWrite(DMA_PIN, HIGH);
  
  pinMode(EF1_PIN, OUTPUT);
  pinMode(EF1_PIN, HIGH);
  
  /* U8g2 pins */
  //Set up CS pin and set low for SPI
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, LOW);

  //Set up RST Pin managed by U8g2
  pinMode(RST_PIN, OUTPUT);
  
  setupVideo();
  
  #if DEBUG
    //wait for dust to settle and print msg
    delay(1000);
    Serial.println("Ready!");
  #endif  
} // setup
/*
 * This code simulates a cdp1861 Pixi video chip to load a video
 * ram buffer then update a 128 x 64 GLCD display.  The EF1 flag, 
 * interrupts and dma lines are all manipulated as in a real cdp1861 
 * chip running in 32 x 64 resolution.
 */
void loop(void) {
   if (video_on) {
    //check to see if pixie video is no longer active
    if (!pixie_video) {     
//      #if DEBUG
//        Serial.println("Stopping Video...");
//      #endif   
      stopVideo();
//      #if DEBUG  
//        printVideoState();
//      #endif
    } //if !pixie_video
        //Check to see if we need to redraw the display
    if (redraw) { 
//      #if DEBUG
//        Serial.println("Redrawing");  
//      #endif
      paintDisplay();  
//      #if DEBUG
//        Serial.println("Done");  
//      #endif          
      redraw = false;   
    } //if redraw  
  } else {   
    //turn video on when port1 input received
    if (pixie_video) {            
      startVideo();
//      #if DEBUG
//        printVideoState();
//      #endif             
    }//if pixie_video
  } //if-else video_on
  
  //check hardware reset   
  if (digitalReadFast(RESET_BTN_PIN) == LOW) {     
//    #if DEBUG
//      Serial.println("Reset!");
//    #endif
    //Turn off flag, stop video and reset display
    pixie_video = false;
    stopVideo();
    setupVideo();
//    #if DEBUG
//      Serial.println("Ready!");
//    #endif    
  } //if RESET_BTN_PIN = LOW
} //loop
