;*****************************************************************
;* COE538 FINAL PROJECT: EEBOT GUIDANCE PROGRAM                  *
;* GROUP: JORDAN, NATHAN, JOAQUIN                                *
;*****************************************************************

; export symbols

              XDEF Entry, _Startup ; export ’Entry’ symbol

              ABSENTRY Entry ; for absolute assembly: mark this as application entry point
; Include derivative-specific definitions
              INCLUDE 'derivative.inc'
;
; Liquid Crystal Display Equates
;-------------------------------
CLEAR_HOME      EQU $01       ; Clear the display and home the cursor
INTERFACE       EQU $38       ; 8 bit interface, two line display
CURSOR_OFF      EQU $0C       ; Display on, cursor off
SHIFT_OFF       EQU $06       ; Address increments, no character shift
LCD_SEC_LINE    EQU 64        ; Starting addr. of 2nd line of LCD (note decimal value!)
; LCD Addresses
LCD_CNTR        EQU PTJ       ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT         EQU PORTB     ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E           EQU $80       ; LCD E-signal pin
LCD_RS          EQU $40       ; LCD RS-signal pin
; Other codes
NULL            EQU 00        ; The string ’null terminator’
CR              EQU $0D       ; ’Carriage Return’ character
SPACE           EQU ' '       ; The ’space’ character

; ADC Thresholds
MIN_A           EQU $B4       ; Anything HIGHER than this values means A senses the tape
MIN_B           EQU $B4       ; Anything HIGHER than this values means B senses the tape
MIN_C           EQU $B4       ; Anything HIGHER than this values means C senses the tape
MIN_D           EQU $CC       ; Anything HIGHER than this values means D senses the tape
MIN_EF          EQU $85       ; Anything LOWER than this values means E senses the tape
MAX_EF          EQU $90       ; Anything HIGHER than this values means F senses the tape

; Turn Thresholds
;ACCY x 50us
;Software timers were used because difficulties were found while implementing hardware timer
;Remnants of the code remains.
FWD_INT         EQU 4000      ; Used for forward movement while line tracking
RIGHT_INT       EQU 54000;13500     ; Time needed to make the bot turn right.
LEFT_INT        EQU 100       ; Time needed to make the bot turn left.
U_TRN_INT       EQU 54000     ; Time needed to make a full U-Turn
TURN_ADJ        EQU 10000       ; Used to bring the EEBot's wheels into the intersection.
NUDGE           EQU 3000       ; Used to adjust the bot LFT and RHT while line tracking.
T_REV           EQU 30      ; Used for U-Turn reverse timing


; State defintions
START           EQU 0
FWD             EQU 1
U_TRN           EQU 2
ALL_STP         EQU 3
RIGHT_TRN       EQU 4
LEFT_TRN        EQU 5

; variable/data section
                ORG $3800
;---------------------------------------------------------------------------
; Storage for the Timer Readings and Current State Register
                ;ORG   $3850 ; Where our TOF counter register lives
TOF_COUNTER     dc.b  50 ; The timer, incremented at 23Hz
CRNT_STATE      dc.b  3 ; Current state register
T_RHT_TRN       ds.b  1 ; Right turn time
T_LFT_TRN       ds.b  1 ; Left turn time
T_U_TRN         ds.b  1 ; U-Turn time

; Storage for the Sensor readings
SENSOR_LINE     FCB $01 ; Storage for EF Line Sensor Reading
SENSOR_BOW      FCB $23 ; Storage for A Bow Sensor Reading (Bow = front)
SENSOR_PORT     FCB $45 ; Storage for D Port Sensor Reading (Port = Left)
SENSOR_MID      FCB $67 ; Storage for C Mid Sensor Reading
SENSOR_STBD     FCB $89 ; Storage for B Starboard Sensor Reading (Startboard = Right) 
SENSOR_NUM      RMB 1   ; The currently selected sensor (0 = line, 1 = bow, etc...)

TOF_TEMP        RMB 2

TOP_LINE        RMB 20 ; Top line of display
                FCB NULL ; terminated by null
                
BOT_LINE        RMB 20 ; Bottom line of display
                FCB NULL ; terminated by null
                
CLEAR_LINE      FCC '            '
                FCB NULL ; terminated by null
TEMP            RMB 1 ; Temporary location


;****************************************************************************
                ORG $4000 ;Start of Code Section                            *
;****************************************************************************-
Entry:
_Startup:

;****************************************************************************
;* Initialization                                                           *
;**************************************************************************
                LDS #$4000      ; Initialize the stack pointer
                CLI             ; Enable interrupts
                JSR INIT        ; Initialize ports
                JSR openADC     ; Initialize the ATD
                JSR openLCD     ; Initialize the LCD
                JSR CLR_LCD_BUF ; Write ’space’ characters to the LCD buffer
                JSR ENABLE_TOF  ; Enable for hardware timer
                ;Initializes the motors
                BSET  DDRA,%00000011  ; STAR_DIR, PORT_DIR 
                BSET  DDRT,%00110000  ; STAR_SPEED, PORT_SPEED

;****************************************************************************
;* Main Loop                                                                *
;****************************************************************************
MAIN            JSR   G_LEDS_ON       ; Enable the guider LEDs
                JSR   READ_SENSORS    ; Read the 5 guider sensors
                JSR   G_LEDS_OFF      ; Disable the guider LEDs
                JSR   DISPLAY_SENSORS ; Updates the display
                LDY   #1000
                JSR   del_50us
                LDAA  CRNT_STATE      ; Loads AccA with Current State
                JSR   DISPATCHER      ; Jumps to Dispatcher Routine
                BRA   MAIN
                
;****************************************************************************
;* Data Section                                                             *
;****************************************************************************
;msg1            dc.b  "Battery volt ",0 
;msg2            dc.b  "State  ",0
tab             dc.b  "START  ",0      
                dc.b  "FWD    ",0        
                dc.b  "U_TRN  ",0
                dc.b  "ALL_STP",0
                dc.b  "RHT_TRN",0
                dc.b  "LFT_TRN",0
                
;****************************************************************************
;* Dispatcher
; Works similarly to lab5, jumps and checks thru each state. If equal, do nothing and 
;go to next JSR to start the routine. If not, check for next state, etc...          *
;**************************************************************************** 
DISPATCHER      CMPA  #START      ; If it’s the START state
                BNE   NOT_START   ; else Jump to Not Start State
                JSR   START_ST    ; then call START_ST routine
                BRA   DISP_EXIT   ; and exit 
                
NOT_START       CMPA  #FWD        ;                                                     
                BNE   NOT_FWD
                JSR   FWD_ST      ; then call it
                JMP   DISP_EXIT   ; and exit
                
NOT_FWD         CMPA  #U_TRN
                BNE   NOT_U_TRN   ;
                JSR   U_TRN_ST    ;JUMP TO U_TRN
                JMP   DISP_EXIT

NOT_U_TRN       CMPA  #ALL_STP
                BNE   NOT_ALL_STP ;
                JSR   ALL_STP_ST  ;JMP TO ALL_STP
                JMP   DISP_EXIT
                
NOT_ALL_STP     CMPA  #RIGHT_TRN
                BNE   NOT_RHT_TRN
                JSR   RHT_TRN_ST            ;JMP TO RIGHT TURN
                JMP   DISP_EXIT

;This is dead code for the time being, Bot is right hand rule wall follower.
;NOT_RHT_TRN     CMPA  #LEFT_TRN                
;                BNE   NOT_LFT_TRN
;                NOP               ;JMP TO LEFT TURN
;                JMP   DISP_EXIT
                                
; Not Reverse Turn: Terminates Program.                               
NOT_RHT_TRN     SWI ; Else the CRNT_ST is not defined, so stop

; Returns to Main Loop. 
DISP_EXIT       RTS ; Exit from the state dispatcher

;****************************************************************************
;* States                                                                   *
;****************************************************************************
;----------------------------------------------------------------------------
; START STATE
;----------------------------------------------------------------------------
START_ST        BRCLR PORTAD0,$04,NO_FORWARD  ; If FWD_BUMP Is Pressed Do nothing and return to main
                JSR   INIT_FWD                ; Initialize the FORWARD state
                MOVB  #FWD,CRNT_STATE         ; Go into the FORWARD state
                BRA   START_EXIT
NO_FORWARD      NOP                           ; Else
START_EXIT      RTS                           ; return to the MAIN routine

;----------------------------------------------------------------------------
; FORWARD STATE
; if Front_Bump = Pressed -> Set state to U_TRN
; If Back_Bump = Pressed -> Set state to ALL_STP
; if D detects tape -> set state to RHT_TRN
; else 
;   if EF > ~EFmid -> Make Right Adjust
;   if EF < ~EFmid -> Make Left Adjust
;   else
;   Continue Forward.
;----------------------------------------------------------------------------
FWD_ST          BRSET PORTAD0,$04,NO_FWD_BUMP   ; If FWD_BUMP isn't pressed jmp to NO_FWD_BUMP
                JSR   INIT_U_TRN                ; initialize the U-TURN routine
                MOVB  #U_TRN,CRNT_STATE         ; set the state to U_TRN state 
                JMP   FWD_EXIT                  ; and return
                
NO_FWD_BUMP     BRSET PORTAD0,$08,NO_REAR_BUMP  ;  If REAR_BUMP isn't pressed jmp to NO_REAR_BUMP
                JSR   INIT_ALL_STP              ; else, initialize the ALL_STOP state
                MOVB  #ALL_STP,CRNT_STATE       ; and change state to ALL_STOP
                JMP   FWD_EXIT                  ; and return


;!!!
                
NO_REAR_BUMP    LDAA  SENSOR_STBD                ; Load AccA with the right pattern sensor               
                CMPA  #MIN_D                     ; Compare sensor value with threshold
                BLO   NOT_D                      ; Branch if D doesn't detect tape
                JSR   INIT_RHT                   ; Otherwise intialize right turn
                MOVB  #RIGHT_TRN,CRNT_STATE      ; Set state to right Turn
                JMP   FWD_EXIT
; Dead code for until we get the bot to complete the maze first, This will be needed for
; only for maze retrace.                
;NOT_B           LDAA  SENSOR_PORT               ; Load AccA with the left pattern sensor               
;                CMPA  MIN_D                     ; Compare sensor value with threshold
;                BLT   NOT_D                     ; Branch if D doesn't detect tape
;                JMP   INIT_LFT                  ; Otherwise intialize left turn
;                MOVB  #LEFT_TRN,CRNT_STATE      ; Set state to left Turn
;                JMP   FWD_EXIT

NOT_D           LDAA  SENSOR_LINE                ; Load Acca with the line sensor
                CMPA  #MIN_EF                    ; Compare with Lower Bound Value (Left tilted)
                BHI   NO_ADJ_LFT                 ; If more than lower bound, not adj needed
                BSET  PTT,%00100000              ; Turn on only right motor
                LDY   #NUDGE                     ; Wait for a bit to adjust
                JSR   del_50us
                BCLR  PTT,%00100000              ; Turn off the right motor
                JMP   FWD_EXIT
                
                
NO_ADJ_LFT      LDAA  SENSOR_LINE                ; This is for redundency, load Acca with line sensor value
                CMPA  #MAX_EF                    ; Compare with Upper Bound Value,
                BLO   NO_ADJ_RHT                 ; If less than upper bound, no adj needed
                BSET  PTT,%00010000              ; Turn on only left motor
                LDY   #NUDGE                     ; Wait for a bit to adjust
                JSR   del_50us
                BCLR  PTT,%00010000              ; Turn off the left motor
                JMP   FWD_EXIT

NO_ADJ_RHT      BSET  PTT,%00110000              ; Turn ON the drive motors
                LDY   #FWD_INT                     ; wait a bit to nudge forwards
                JSR   del_50us        
                BCLR  PTT,%00110000              ; Turn OFF the drive motors
                JMP   FWD_EXIT                                           
                
FWD_EXIT        RTS

;----------------------------------------------------------------------------
; RIGHT TURN STATE NOTE: If time permits, change to general turn state.
;----------------------------------------------------------------------------
RHT_TRN_ST      ;LDAA   TOF_COUNTER           ; If Tc>Tfwdturn then
                ;CMPA   T_RHT_TRN
                ;CPD   #RIGHT_INT             ; the robot should go FWD
                ;BNE   NO_RHT_FT             ; so
                ;BRA   RHT_TRN_ST
                LDY   #RIGHT_INT            ;do a delay with a specific time
                JSR   del_50us
                JSR   INIT_FWD              ; initialize the FWD state
                MOVB  #FWD,CRNT_STATE       ; set state to FWD
                BRA   RHT_TRN_EXIT          ; and return
NO_RHT_FT       NOP
                ;RTS                         ; Else
                
RHT_TRN_EXIT    RTS                         ; return to the MAIN routine

;----------------------------------------------------------------------------
; U-TURN STATE : NOTE: I realized after designing that U-turn state doesn't need to exist.
;----------------------------------------------------------------------------
U_TRN_ST        LDY   #U_TRN_INT
                JSR   del_50us              ; so
                JSR   INIT_FWD              ; initialize the FWD state
                MOVB  #FWD,CRNT_STATE       ; set state to FWD
                BRA   U_TRN_EXIT            ; and return
NO_U_FT         NOP                         ; Else
U_TRN_EXIT      RTS                         ; return to the MAIN routine
;----------------------------------------------------------------------------
; ALL STOP STATE
;----------------------------------------------------------------------------
ALL_STP_ST      BRSET PORTAD0,$04,NO_START  ; If FWD_BUMP
                BCLR  PTT,%00110000         ; initialize the START state (both motors off)
                MOVB  #START,CRNT_STATE     ; set the state to START
                BRA   ALL_STP_EXIT          ; and return
NO_START        NOP                         ; Else
ALL_STP_EXIT    RTS                         ; return to the MAIN routine

;****************************************************************************
;* State Initializations                                                    *
;****************************************************************************
;---------------------------------------------------------------------------
; Initialize Forward State.
; Sets motors to forward direction, and turns both motors off.
;---------------------------------------------------------------------------
INIT_FWD        BCLR  PORTA,%00000011   ; Set FWD direction for both motors
                BCLR  PTT,%00110000     ; Turn OFF the drive motors
                RTS

;---------------------------------------------------------------------------
; Initialize Right Turn State.
; Sets motors to forward direction.
; Turns on both drive motors to bring the EEBot's wheels into the intersection
; to make the turn.
; Turns off both wheels and then turns on port wheel to make turn.
; Uses hardware timer to know when to stop turning.
;---------------------------------------------------------------------------                
INIT_RHT        BCLR  PORTA, %00000011  ; Redundent. Set motors to fwd   
                BSET  PTT,%00110000     ; turn on drive motors
                LDY   #TURN_ADJ         ; Bring wheels to intersection
                
                JSR   del_50us        
                BCLR  PTT,%00110000     ; turn off both wheels.
                BSET  PTT,%00010000     ; turn ON only port wheel to start turning right.
                
                ;;;;;
                ;LDD   #0
                ;STD   TOF_COUNTER
                ;;;;
                ;changed from D to A??
                ;LDAA   TOF_COUNTER       ; Mark the current time
                ;ADDA   #RIGHT_INT        ; Add the interval needed for right turn
                ;STAA   T_RHT_TRN         ; Store result to right turn interval register
                RTS                

;---------------------------------------------------------------------------
; Initialize All Stop State.
; Turns off both drive motors
;---------------------------------------------------------------------------
INIT_ALL_STP    BCLR  PTT,%00110000     ; Turn off the drive motors
                RTS
;---------------------------------------------------------------------------
; Initialize U Turn State.
; Set both motors to reverse direction and reverse a little bit to move
; away from the wall.
; After reversing set the right motor to forward direction to start turing
; on the spot.
; Uses hardware timer to know when to stop turning.
;---------------------------------------------------------------------------
INIT_U_TRN      BSET  PORTA,%00000011   ; Set REV direction to both motors
                BSET  PTT,%00110000     ; Reverse the bot for a bit
                LDY   T_REV            
                JSR   del_50us
                
                BCLR  PTT,%00110000      ; TURN OFF BOTH MOTORS        
                BCLR  PORTA,%00000010   ; Set FWD direction to Right motor
                BSET  PTT,%00110000     ; turn ON BOTH wheelS to start turning right.
                
                ;LDD   TOF_COUNTER       ; Mark the current time
                ;ADDD  #U_TRN_INT        ; Add the interval needed for right turn
                ;STD   T_U_TRN           ; Store result to U turn interval register
                RTS               
               
; subrotine section
;---------------------------------------------------------------------------
; Initialize ports
INIT            BCLR DDRAD,$FF ; Make PORTAD an input (DDRAD @ $0272)
                BSET DDRA,$FF ; Make PORTA an output (DDRA @ $0002)
                BSET DDRB,$FF ; Make PORTB an output (DDRB @ $0003)
                BSET DDRJ,$C0 ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                RTS
;---------------------------------------------------------------------------
; Initialize the ADC
openADC         MOVB #$80,ATDCTL2 ; Turn on ADC (ATDCTL2 @ $0082)
                LDY #1 ; Wait for 50 us for ADC to be ready
                JSR del_50us ; - " -
                MOVB #$20,ATDCTL3 ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                MOVB #$97,ATDCTL4 ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                BSET  ATDDIEN,$0C        ;configure pins AN03,AN02 as digital inputs
                RTS
;---------------------------------------------------------------------------
; Clear LCD Buffer
; This routine writes ’space’ characters (ascii 20) into the LCD display
; buffer in order to prepare it for the building of a new display buffer.
; This needs only to be done once at the start of the program. Thereafter the
; display routine should maintain the buffer properly.
CLR_LCD_BUF     LDX #CLEAR_LINE
                LDY #TOP_LINE
                JSR STRCPY
CLB_SECOND      LDX #CLEAR_LINE
                LDY #BOT_LINE
                JSR STRCPY
                
CLB_EXIT        RTS

;---------------------------------------------------------------------------
; String Copy
; Copies a null-terminated string (including the null) from one location to
; another
; Passed: X contains starting address of null-terminated string
; Y contains first address of destination

STRCPY          PSHX ; Protect the registers used
                PSHY
                PSHA
STRCPY_LOOP     LDAA 0,X ; Get a source character
                STAA 0,Y ; Copy it to the destination
                BEQ STRCPY_EXIT ; If it was the null, then exit
                INX ; Else increment the pointers
                INY
                BRA STRCPY_LOOP ; and do it again
STRCPY_EXIT     PULA ; Restore the registers
                PULY
                PULX
                RTS

;---------------------------------------------------------------------------
; Guider LEDs ON
G_LEDS_ON       BSET PORTA,%00100000 ; Set bit 5
                RTS
;
; Guider LEDs OFF
G_LEDS_OFF      BCLR PORTA,%00100000 ; Clear bit 5
                RTS
;---------------------------------------------------------------------------
; Read Sensors
;.
READ_SENSORS      CLR SENSOR_NUM ; Select sensor number 0
                  LDX #SENSOR_LINE ; Point at the start of the sensor array
RS_MAIN_LOOP      LDAA SENSOR_NUM ; Select the correct sensor input
                  JSR SELECT_SENSOR ; on the hardware
                  LDY #400 ; 20 ms delay to allow the
                  JSR del_50us ; sensor to stabilize
                  LDAA #%10000001 ; Start A/D conversion on AN1
                  STAA ATDCTL5
                  BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
                  
                  LDAA ATDDR0L ; A/D conversion is complete in ATDDR0L
                  STAA 0,X ; so copy it to the sensor register
                  CPX #SENSOR_STBD ; If this is the last reading
                  BEQ RS_EXIT ; Then exit

                  INC SENSOR_NUM ; Else, increment the sensor number
                  INX ; and the pointer into the sensor array
                  BRA RS_MAIN_LOOP ; and do it again

RS_EXIT           RTS
;---------------------------------------------------------------------------
; Select Sensor

; Finally, save the TEMP to the hardware.

SELECT_SENSOR     PSHA ; Save the sensor number for the moment
                  
                  LDAA PORTA ; Clear the sensor selection bits to zeros
                  ANDA #%11100011 ;
                  STAA TEMP ; and save it into TEMP

                  PULA ; Get the sensor number
                  ASLA ; Shift the selection number left, twice
                  ASLA ;
                  ANDA #%00011100 ; Clear irrelevant bit positions

                  ORAA TEMP ; OR it into the sensor bit positions
                  STAA PORTA ; Update the hardware
                  RTS

;---------------------------------------------------------------------------
; Display Sensor Readings

DP_FRONT_SENSOR   EQU TOP_LINE+3
DP_PORT_SENSOR    EQU BOT_LINE+0
DP_MID_SENSOR     EQU BOT_LINE+3
DP_STBD_SENSOR    EQU BOT_LINE+6
DP_LINE_SENSOR    EQU BOT_LINE+9
DP_TOF_COUNTER    EQU TOP_LINE+6

DISPLAY_SENSORS   LDAA SENSOR_BOW ; Get the FRONT sensor value
                  JSR BIN2ASC ; Convert to ascii string in D
                  LDX #DP_FRONT_SENSOR ; Point to the LCD buffer position
                  STD 0,X ; and write the 2 ascii digits there

                  LDAA SENSOR_PORT ; Repeat for the PORT value
                  JSR BIN2ASC
                  LDX #DP_PORT_SENSOR
                  STD 0,X

                  LDAA SENSOR_MID ; Repeat for the MID value
                  JSR BIN2ASC
                  LDX #DP_MID_SENSOR
                  STD 0,X
                  
                  LDAA SENSOR_STBD ; Repeat for the STARBOARD value
                  JSR BIN2ASC
                  LDX #DP_STBD_SENSOR
                  STD 0,X
                  
                  LDAA SENSOR_LINE ; Repeat for the LINE value
                  JSR BIN2ASC
                  LDX #DP_LINE_SENSOR
                  STD 0,X
                  
                  LDAA  TOF_COUNTER
                  JSR   BIN2ASC
                  LDX   #DP_TOF_COUNTER
                  STD   0,X  
                  
                  LDAA #CLEAR_HOME ; Clear the display and home the cursor
                  JSR cmd2LCD ; "
                  
                  LDY #40 ; Wait 2 ms until "clear display" command is complete
                  JSR del_50us

                  LDX #TOP_LINE ; Now copy the buffer top line to the LCD
                  JSR putsLCD

                  LDAA #LCD_SEC_LINE ; Position the LCD cursor on the second line
                  JSR LCD_POS_CRSR

                  LDX #BOT_LINE ; Copy the buffer bottom line to the LCD
                  JSR putsLCD
                  
                 ; LDAA  TOF_COUNTER
                 ; JSR   BIN2ASC
                 ; LDX   #$89
                  ;LDAA  TOF_COUNTER
                  ;JSR   BIN2ASC
                  ;STD   TOF_TEMP
                  ;LDAA  #$89
                  ;JSR   cmd2LCD
                  ;LDX   #TOF_TEMP
                  ;JSR   putsLCD  
                  
                  LDAA  #$C9 ; Move LCD cursor to the 2nd row, end of msg2
                  JSR   cmd2LCD ;
                  LDAB  CRNT_STATE ; Display current state
                  LSLB ; "
                  LSLB ; "
                  LSLB ; "
                  LDX   #tab ; "
                  ABX ; "
                  JSR   putsLCD ; "
                  RTS

;---------------------------------------------------------------------------
; Binary to ASCII
; Converts an 8 bit binary value in ACCA to the equivalent ASCII character 2
; character string in accumulator D
; Uses a table-driven method rather than various tricks.
; Passed: Binary value in ACCA
; Returns: ASCII Character string in D
; Side Fx: ACCB is destroyed

HEX_TABLE         FCC '0123456789ABCDEF' ; Table for converting values

BIN2ASC           PSHA ; Save a copy of the input number on the stack
                  TAB ; and copy it into ACCB
                  ANDB #%00001111 ; Strip off the upper nibble of ACCB
                  CLRA ; D now contains 000n where n is the LSnibble
                  ADDD #HEX_TABLE ; Set up for indexed load
                  XGDX
                  LDAA 0,X ; Get the LSnibble character

                  PULB ; Retrieve the input number into ACCB
                  PSHA ; and push the LSnibble character in its place
                  RORB ; Move the upper nibble of the input number
                  RORB ; into the lower nibble position.
                  RORB
                  RORB
                  ANDB #%00001111 ; Strip off the upper nibble
                  CLRA ; D now contains 000n where n is the MSnibble
                  ADDD #HEX_TABLE ; Set up for indexed load
                  XGDX
                  LDAA 0,X ; Get the MSnibble character into ACCA
                  PULB ; Retrieve the LSnibble character into ACCB
                  RTS
;---------------------------------------------------------------------------
; Routines to control the Liquid Crystal Display
;---------------------------------------------------------------------------
; Initialize the LCD
openLCD           LDY #2000 ; Wait 100 ms for LCD to be ready
                  JSR del_50us ; "
                  LDAA #INTERFACE ; Set 8-bit data, 2-line display, 5x8 font
                  JSR cmd2LCD ; "
                  LDAA #CURSOR_OFF ; Display on, cursor off, blinking off
                  JSR cmd2LCD ; "
                  LDAA #SHIFT_OFF ; Move cursor right (address increments, no char. shift)
                  JSR cmd2LCD ; "
                  LDAA #CLEAR_HOME ; Clear the display and home the cursor
                  JSR cmd2LCD ; "
                  LDY #40 ; Wait 2 ms until "clear display" command is complete
                  JSR del_50us ; "
                  RTS
;---------------------------------------------------------------------------
; Send a command in accumulator A to the LCD
cmd2LCD           BCLR LCD_CNTR,LCD_RS ; Select the LCD Instruction register
                  JSR dataMov ; Send data to IR or DR of the LCD
                  RTS
;---------------------------------------------------------------------------
; Send a character in accumulator in A to LCD
putcLCD           BSET LCD_CNTR,LCD_RS ; select the LCD Data register
                  JSR dataMov ; send data to IR or DR of the LCD
                  RTS
;---------------------------------------------------------------------------
; Send a NULL-terminated string pointed to by X
putsLCD           LDAA 1,X+ ; get one character from the string
                  BEQ donePS ; reach NULL character?
                  JSR putcLCD
                  BRA putsLCD
donePS            RTS

;---------------------------------------------------------------------------
; Send data to the LCD IR or DR depending on the RS signal
dataMov           BSET LCD_CNTR,LCD_E ; pull the LCD E-sigal high
                  STAA LCD_DAT ; send the 8 bits of data to LCD
                  NOP
                  NOP
                  NOP
                  BCLR LCD_CNTR,LCD_E ; pull the E signal low to complete the write operation
                  
                  LDY #1 ; adding this delay will complete the internal
                  JSR del_50us ; operation for most instructions
                  RTS
;---------------------------------------------------------------------------
; Position the Cursor
LCD_POS_CRSR      ORAA #%10000000 ; Set the high bit of the control word
                  JSR cmd2LCD ; and set the cursor address
                  RTS
;---------------------------------------------------------------------------
; 50 Microsecond Delay
del_50us          PSHX ; (2 E-clk) Protect the X register
eloop             LDX #300 ; (2 E-clk) Initialize the inner loop counter
iloop             NOP ; (1 E-clk) No operation
                  DBNE X,iloop ; (3 E-clk) If the inner cntr not 0, loop again
                  DBNE Y,eloop ; (3 E-clk) If the outer cntr not 0, loop again
                  PULX ; (3 E-clk) Restore the X register
                  RTS ; (5 E-clk) Else return
;----------------------------------------------------------------------------
; Enable the Timer Overflow
ENABLE_TOF        LDAA  #%10000000
                  STAA  TSCR1           ; Enable TCNT
                  STAA  TFLG2           ; Clear TOF
                  LDAA  #%10000100      ; Enable TOI and select prescale factor equal to 16
                  STAA  TSCR2
                  RTS                   ; same as in Appendix B of this lab (Lab 4)
;----------------------------------------------------------------------------  
;****************************************************************************
;* Interupt Routines                                                        *
;**************************************************************************** 
;----------------------------------------------------------------------------  
; Timer Overflow Interupt Service Routine
TOF_ISR           INC   TOF_COUNTER
                  LDAA  #%10000000      ; Clear (Writes 1 to the flag bit of the inerupt)
                  STAA  TFLG2           ; TOF 
                  RTI
;----------------------------------------------------------------------------  



;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
                    ORG   $FFFE
                    DC.W  Entry ; Reset Vector
                    ORG   $FFDE
                    DC.W  TOF_ISR ; Timer Overflow Interrupt Vecto
