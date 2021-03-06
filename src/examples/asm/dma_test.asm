;  -------------------------------------------------------------------
;  DMA Video Test
;  Based on Tom Pittman's original Video DMA Program
;  Published in A Short Course in Computer Programming by Tom Pittman
;  Copyright 1979 Netronics Research & Development Ltd.
;
;  Modified to run under the Elf/OS with Pico/Elf Pixie Video
;  Copyright 2021 by Gaston Williams
;  -------------------------------------------------------------------
; *** Based on Elf/OS software written by Michael H Riley
; *** Thanks to the author for making this code available.
; *** Original author copyright notice:
; *******************************************************************
; *** This software is copyright 2004 by Michael H Riley          ***
; *** You have permission to use, modify, copy, and distribute    ***
; *** this software so long as this copyright notice is retained. ***
; *** This software may not be used in commercial applications    ***
; *** without express written permission from the author.         ***
; *******************************************************************

; Uncomment the line below to assemble a ROM program
; Comment out to assemble a stand alone program
;#DEFINE _ROM_ 1

include    bios.inc
include    kernel.inc

#IFDEF _ROM_
; ***************************************************
; *** This block generates the ROM package header ***
; ***************************************************
           org     8000h                 ;
           lbr     0ff00h                ; Long branch for Pico/Elf
           db      'dma_test',0          ; Program filename
           dw      9000h                 ; Where in Rom image to find program
           dw      endrom-2000h+9000h    ; last address of image
; *******************************************************
; **** Next block is the execution header, this is   ****
; **** part of the package header and written to the ****
; **** actual file when saved to disk                ****
; *******************************************************
           dw      2000h           ; Exec header, where program loads
           dw      endrom-2000h    ; Length of program to load
           dw      2000h           ; Execution address
           db      0               ; Zero marks end of package header
; *********************************
; ***** End of package header *****
; *********************************

  org     2000h
#ENDIF

#IFNDEF _ROM_
; ************************************************************
; This block generates the Execution header for a stand-alone
; program. It begins 6 bytes before the program start.
; ************************************************************

           org     02000h-6        ; Header starts at 01ffah
           dw      2000h
           dw      endrom-2000h
           dw      2000h
#ENDIF

           br      start
; **************************************************
; *** Build information:                         ***
; ***    build date                              ***
; ***    build number                            ***
; ***    information text string                 ***
; **************************************************
; Build date format:
; 80h+month, day, four digit year
; **************************************************
; 80h month offset indicates extended
; build information, with build number and text.
; **************************************************
date:      db      80h+1  ; Month: January
           db      12     ; Day
           dw      2021   ; Year

build:     dw      3      ; build

           db      'Copyright 2021 Gaston Williams',0

        ; stack pointer r2 is already set by OS
start:     ldi 023H ; value for x=2; p=3
           str r2   ; save for disable Int instruction
           dis      ; Keep x=2; p=3 and disable interrupts

           ghi r3   ; P = 3
           phi r0   ; set up DMA pointer
           ldi 080H ; point to video Data
           glo r0   ; set up DMA pointer

Video:     inp 1    ; turn video on
           ;------------ DMA occurs here ------------
loop:      ldi 080H ; fix r0
           plo r0
           bn4 loop ; continue until input pressed
           ; Leave interrupts disabled
           out 1    ; turn off Video
           lbr     o_wrmboot       ; return to Elf/OS


  org 2080H
; data for video dma
buffer: db 080H, 081H, 082H, 083H, 084H, 085H, 086H, 087H
        db 088H, 089H, 08AH, 08BH, 08CH, 08DH, 08EH, 08FH

endrom:    equ     $               ; End of code
