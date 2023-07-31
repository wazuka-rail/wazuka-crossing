#ifdef  SIMPLE
    PROCESSOR 16F15213
#else
    PROCESSOR 16F15223
#endif

#include <xc.inc>

#include "macros.inc"

; CONFIG1
  CONFIG  FEXTOSC = OFF         ; External Oscillator Mode Selection bits (Oscillator not enabled)
  CONFIG  RSTOSC = HFINTOSC_32MHZ; Power-up Default Value for COSC bits (HFINTOSC (32 MHz))
  CONFIG  CLKOUTEN = OFF        ; Clock Out Enable bit (CLKOUT function is disabled; I/O function on RA4)
  CONFIG  VDDAR = LO            ; VDD Range Analog Calibration Selection bit (Internal analog systems are calibrated for operation between VDD = 1.8V - 3.6V)

; CONFIG2
  CONFIG  MCLRE = INTMCLR       ; Master Clear Enable bit (If LVP = 0, MCLR pin is port defined function; If LVP = 1, RA3 pin function is MCLR)
  CONFIG  PWRTS = PWRT_1        ; Power-up Timer Selection bits (PWRT set at 1 ms)
  CONFIG  WDTE = NSLEEP         ; WDT Operating Mode bits (WDT enabled while Sleep = 0, suspended when Sleep = 1; SEN is ignored)
  CONFIG  BOREN = NSLEEP        ; Brown-out Reset Enable bits (Brown-out Reset enabled while running, disabled in Sleep; SBOREN is ignored)
  CONFIG  BORV = LO             ; Brown-out Reset Voltage Selection bit (Brown-out Reset Voltage (VBOR) set to 1.9V)
  CONFIG  PPS1WAY = ON          ; PPSLOCKED One-Way Set Enable bit (The PPSLOCKED bit can be set once after an unlocking sequence is executed; once PPSLOCKED is set, all future changes to PPS registers are prevented)
  CONFIG  STVREN = ON           ; Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

; CONFIG3

; CONFIG4
  CONFIG  BBSIZE = BB512        ; Boot Block Size Selection bits (512 words boot block size)
  CONFIG  BBEN = OFF            ; Boot Block Enable bit (Boot Block is disabled)
  CONFIG  SAFEN = OFF           ; SAF Enable bit (SAF is disabled)
  CONFIG  WRTAPP = OFF          ; Application Block Write Protection bit (Application Block is not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot Block is not write-protected)
  CONFIG  WRTC = OFF            ; Configuration Registers Write Protection bit (Configuration Registers are not write-protected)
  CONFIG  WRTSAF = OFF          ; Storage Area Flash (SAF) Write Protection bit (SAF is not write-protected)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR. MCLRE Configuration bit is ignored.)

; CONFIG5
  CONFIG  CP = OFF              ; User Program Flash Memory Code Protection bit (User Program Flash Memory code protection is disabled)


; Common    0x70-0x7F 16 bytes
;-------------------------------------------------------------------------------
ALERT_STATE     equ 0x70
#define STATE_R     ALERT_STATE, 0
#define STATE_A     ALERT_STATE, 1
#define STATE_B     ALERT_STATE, 2
#define STATE_C     ALERT_STATE, 3
#define STATE_D     ALERT_STATE, 4


TIMER           equ 0x71
#define TIMER_20HZ  TIMER, 0

; Button Count
BUTTON_DOWN_COUNT   equ 0x74
BUTTON_UP_COUNT     equ 0x75
BUTTON_STATE        equ 0x76
#define BUTTON_CLICKED  BUTTON_STATE, 0
#define BUTTON_CLICKED1 BUTTON_STATE, 1
#define BUTTON_CLICKED2 BUTTON_STATE, 2
#define BUTTON_CLICKED3 BUTTON_STATE, 3
#define BUTTON_CLICKED4 BUTTON_STATE, 4
#define BUTTON_CLICKED5 BUTTON_STATE, 5
#define BUTTON_CLICKED6 BUTTON_STATE, 6


; BANK 0    0x20-0x6F 80 bytes
;-------------------------------------------------------------------------------

; Lamp Flicker
FLICKER_COUNT   equ 0x24
FLICKER_STATE   equ 0x25
; Lamp PWM
LAMP_PFM_COUNT  equ 0x26
LAMP_PFM_CYCLE  equ 0x27


; Sound Oscillator 1
OSC1_ACC_L      equ 0x30
OSC1_ACC_H      equ 0x31
OSC1_DELTA      equ 0x32
OSC1_AMP        equ 0x33
OSC1_ATT        equ 0x34

; Sound Oscillator 2
OSC2_ACC_L      equ 0x35
OSC2_ACC_H      equ 0x36
OSC2_DELTA      equ 0x37
OSC2_AMP        equ 0x38    ; not used
OSC2_ATT        equ 0x39

; WaveTableSelector
WAVE_TABLE_L    equ 0x3A
WAVE_TABLE_H    equ 0x3B

; Envelope Modulator
ENVELOPE_COUNT_L equ 0x3C
ENVELOPE_COUNT_H equ 0x3D
ENVELOPE_CYCLE   equ 0x3E

;-------------------------------------------------------------------------------
; Pin
#ifdef  SIMPLE

BUTTON      equ 0 ; RA0
LAMP1       equ 5 ; RA5
LAMP2       equ 4 ; RA4
SPEAKER1    equ 1 ; RA1
SPEAKER2    equ 2 ; RA2

#else

BUTTON      equ 1 ; RA1
LAMP1       equ 5 ; RA5
LAMP2       equ 4 ; RA4
SPEAKER     equ 5 ; RC5
#endif

;-------------------------------------------------------------------------------
    PSECT resetVec, abs, class=CODE, delta=2
    ORG         0
    goto        Initialize
;-------------------------------------------------------------------------------
    PSECT intVec, abs, class=CODE, delta=2
    ORG         4
Interrupt:
    banksel     PIR1
InterruptTimer2:
    btfss       TMR2IF
    goto        InterruptTimer1

    ; 20 kHz
    bcf         TMR2IF

#ifndef SIMPLE
    goto        SetSoundPWM
SetMotorPDM:

#endif
SetSoundPWM:
    btfsc       STATE_R
    retfie

    banksel     LATA
    movfw       WAVE_TABLE_H
    movwf       FSR0H

    movfw       OSC1_DELTA
    addwf       OSC1_ACC_L, F
    skpnc
    incf        OSC1_ACC_H, F
    movfw       OSC1_ACC_H
    andlw       0x0F
    addwf       WAVE_TABLE_L, W
    addwf       OSC1_ATT, W
    skpnc
    iorlw       0xF0    ; clamp
    movwf       FSR0L   ; FSR0L = min(WAVE_TABLE_L + OSC1_ATT) + phase
    movfw       INDF0
    movwf       OSC1_AMP

    movfw       OSC2_DELTA
    addwf       OSC2_ACC_L, F
    skpnc
    incf        OSC2_ACC_H, F
    movfw       OSC2_ACC_H
    andlw       0x0F
    addwf       WAVE_TABLE_L, W
    addwf       OSC2_ATT, W
    skpnc
    iorlw       0xF0    ; clamp
    movwf       FSR0L   ; FSR0L = min(WAVE_TABLE_L + OSC2_ATT) + phase
    movfw       INDF0   ; W = OSC2_AMP

    addwf       OSC1_AMP, W ; W = OSC1_AMP + OSC2_AMP
    rrf         WREG, F     ; W = (OSC1_AMP + OSC2_AMP) >> 1
    banksel     PWM3DCH
    movwf       PWM3DCH
#ifdef  SIMPLE
    movwf       PWM4DCH
#endif
    clrf        PWM3DCL
    rrf         PWM3DCL, F  ; bit7: (OSC1_AMP + OSC2_AMP) & 1
#ifdef  SIMPLE
    movfw       PWM3DCL
    movwf       PWM4DCL
#endif

ProcLampPFM:
    banksel     LATA
    movlw       (1 << LAMP1) | (1 << LAMP2) ; off
    decf        LAMP_PFM_COUNT, F
    skpz
    goto        ProcLampPFMOff
    movfw       LAMP_PFM_CYCLE
    movwf       LAMP_PFM_COUNT
    movfw       FLICKER_STATE ; on or default
ProcLampPFMOff:
    movwf       LATA

ProcSoundEnvelope:
    decfsz      ENVELOPE_COUNT_L, F
    retfie
    ; ~ 130 Hz
    movfw       ENVELOPE_CYCLE
    movwf       ENVELOPE_COUNT_L

    decfsz      ENVELOPE_COUNT_H, F
    retfie
    ; ~ 32 Hz
    movlw       4          ; 20 kHz / CYCLE / 4 = 5 kHz / CYCLE
    movwf       ENVELOPE_COUNT_H

    movlw       0x10
    addwf       WAVE_TABLE_L, F    ; change envelope
    retfie

InterruptTimer1:
#ifndef SIMPLE
    btfss       TMR1IF
    goto        InterruptTimer0

    ; 10 Hz, 0.1 s interval
    bcf         TMR1IF
    ; 500 kHz / 10 Hz = 50000
    ; 0x10000 - 50000 = 0x3CB0
    banksel     TMR1H
    movlw       0x3C
    movwf       TMR1H
    movlw       0xB0
    addwf       TMR1L, F

    retfie
#endif

InterruptTimer0:
    ; banksel     PIR0
    btfss       TMR0IF
    goto        InterruptOther

    ; 20 Hz
    bsf         TIMER_20HZ
    bcf         TMR0IF
    ; 500 kHz / 20 Hz = 25000
    ; 0x10000 - 25000 = 0x9E58
    banksel     TMR0H
    movlw       0x9E
    movwf       TMR0H
    movlw       0x58
    addwf       TMR0L, F

    retfie

InterruptOther:
    ; unknown interrupt
    reset

;-------------------------------------------------------------------------------
Initialize:
    banksel     OSCFRQ

    movlw       CONF(OSCFRQ, FRQ, 100B) ; 16 MHz
    movwf       OSCFRQ

#ifdef  SIMPLE
    banksel     TRISA
    movlw       (1 << BUTTON) | (1 << SPEAKER1) | (1 << SPEAKER1)
    movwf       TRISA
    banksel     ANSELA
    clrf        ANSELA
    ; banksel     WPUA
    movlw       11111111B
    movwf       WPUA
#else
    banksel     TRISA
    movlw       (1 << SPEAKER)
    movwf       TRISA
    banksel     ANSELA
    clrf        ANSELA
    ; banksel     WPUA
    movlw       11111111B
    movwf       WPUA
#endif


    ; Timers
    banksel     T0CON0
    movlw       CONF(T0CON0,
                    EN, 1, // The module is enabled and operating
                    MD16, 1, // TMR0 is a 16-bit timer
                    OUTPS, 0000B, // 1:1 Postscaler
                )
    movwf       T0CON0
    movlw       CONF(T0CON1,
                    CS, 011B, // HFINTOSC
                    ASYNC, 1, // The input to the TMR0 counter is not synchronized to system clocks
                    CKPS, 0101B, // 1:32 Prescaler
                )
    ; 16 MHz / 32 / 1 = 500 kHz
    movwf       T0CON1
    ; 500 kHz / 20 Hz = 25000
    ; 0x10000 - 25000 = 0x9E58
    movlw       0x9E
    movwf       TMR0H
    movlw       0x58
    movwf       TMR0L

#ifndef SIMPLE
    banksel     T1CON
    movlw       CONF(T1CON,
                    CKPS, 11B, // 1:8 Prescaler value
                    RD16, 1, // Enables register read/write of Timer in one 16-bit operation
                    ON, 0, // Timer is off
                )
    movwf       T1CON
    movlw       CONF(T1CLK, CS, 00001B) ; Fosc / 4
    ; (16 MHz / 4) / 8 = 500 kHz
    movwf       T1CLK
#endif

    banksel     T2CON
    movlw       CONF(T2CON,
                    ON, 0, // Timer is off
                    CKPS, 000B, // 1:1 Prescaler
                    OUTPS, 0000B, // 1:1 Postscaler
                )
    ; Note that when the prescaler is not configured to 1:1,
    ; the duty cycle may be wrong. See the errata.
    movwf       T2CON
    movlw       CONF(T2CLKCON, CS, 001B) ; Fosc / 4
    ; (16 MHz / 4) / 1 = 4000 kHz
    movwf       T2CLKCON
    movlw       200
    ; 4000 kHz / 200 = 20 kHz
    movwf       T2PR

    banksel     FVRCON
    movlw       CONF(FVRCON,
                    FVREN, 1, // Fixed Voltage Reference is enable
                    ADFVR, 01B, // ADC FVR Buffer Gain is 1x, (1.024V)
                )
    movwf       FVRCON


    banksel     PORTA
    clrf        PORTA
    clrf        ALERT_STATE

    movlw       0xFF
    movwf       BUTTON_UP_COUNT
    clrf        BUTTON_DOWN_COUNT
    call        SetSound1

    call        StopAlert

    banksel     PIE0
    movlw       CONF(PIE0,
                    TMR0IE, 1, // TMR0 interrupts are enabled
                )
    movwf       PIE0
    movlw       CONF(PIE1,
                    TMR2IE, 1, // TMR2 interrupts are enabled
                    TMR1IE, 1, // TMR1 interrupts are enabled
                )
    movwf       PIE1
    clrf        PIE2

    banksel     INTCON
    movlw       CONF(INTCON,
                    GIE, 1, // Enables all active interrupts
                    PEIE, 1, // Enables all active peripheral interrupts
                )
    movwf       INTCON

;-------------------------------------------------------------------------------
GoSleep:
#ifdef __DEBUG
    nop
#else
    sleep
#endif
Main:
    clrwdt
    btfsc       TIMER_20HZ
    goto        Timer20Hz
    banksel     T2CON
    btfsc       T2ON
    goto        Main
    goto        GoSleep

Timer20Hz:
    ; 20 Hz
    bcf         TIMER_20HZ

    call        FlickerLamps

#ifdef  SIMPLE
    banksel     PORTA
    btfss       PORTA, BUTTON
    goto        ProcButtonDown
    goto        ProcButtonUp
#else
    banksel     PORTA
    btfss       PORTA, BUTTON
    goto        ProcButtonDown
    goto        ProcButtonUp
#endif

ProcButtonUp:
    movfw       BUTTON_DOWN_COUNT
    skpnz
    goto        ProcButtonUpC
    clrf        BUTTON_DOWN_COUNT
    andlw       0xF8
    skpnz
    goto        ProcButtonUpClicked
    clrf        BUTTON_STATE        ; too long (> 350 ms)
    goto        ProcButtonUpC
ProcButtonUpClicked:
    bsf         BUTTON_CLICKED
    rlf         BUTTON_STATE, F
    bsf         BUTTON_CLICKED

    btfsc       BUTTON_CLICKED6
    clrf        BUTTON_STATE        ; reset

    call        SetSound1
    btfsc       BUTTON_CLICKED2
    call        SetSound2
    btfsc       BUTTON_CLICKED3
    call        SetSound3
    btfsc       BUTTON_CLICKED4
    call        SetSound4
    btfsc       BUTTON_CLICKED5
    call        SetSound5
ProcButtonUpC:
    incfsz      BUTTON_UP_COUNT, W
    incf        BUTTON_UP_COUNT, F  ; clamped

    call        StopAlert
    goto        Main

ProcButtonDown:
    movfw       BUTTON_UP_COUNT
    skpnz
    goto        ProcButtonDownC
    clrf        BUTTON_UP_COUNT
    andlw       0xF0
    skpz
    clrf        BUTTON_STATE        ; too long (> 750 ms)
ProcButtonDownC:
    incfsz      BUTTON_DOWN_COUNT, W
    incf        BUTTON_DOWN_COUNT, F    ; clamped
    call        StartAlert
    goto        Main

;-------------------------------------------------------------------------------
StartAlert:
    btfss       STATE_R
    return
    bcf         STATE_R

    ; Lamp brightness calibration based on Vdd
    banksel     ADCON0
    movlw       CONF(ADCON0,
                    CHS, 011110B, // FVR Buffer 1
                    GO, 0, // ADC conversion cycle in progress
                    ON, 1, // ADC is enabled
                )
    movwf       ADCON0
    movlw       CONF(ADCON1,
                    FM, 0, // Left-justified. The six Least Significant bits of ADRESL are zero-filled.
                    CS, 000B, // Fosc/2
                    PREF, 00B, // VREF+ is connected to VDD
                )
    movwf       ADCON1
    bsf         ADGO
WatchVddADC:
    btfsc       ADGO
    goto        WatchVddADC
    movfw       ADRESH  ; 3.32 V -> 79, 1.77 V -> 148
    sublw       79      ; W = 79 - ADRESH
    skpc
    clrw                ; W = 79 - max(ADRESH, 79)
    sublw       -70     ; W = (256 - 70) - (79 - max(ADRESH, 79)) = max(ADRESH, 79) + 107
    skpc
    clrw                ; W = min(max(ADRESH, 79), 148) + 107
    addlw       -186    ; W = min(max(ADRESH, 79), 148) - 79
    movwf       FSR0L
    movlw       high(LampPFMCycles) | 0x80
    movwf       FSR0H
    movfw       INDF0
    banksel     LATA
    movwf       LAMP_PFM_CYCLE


    movlw       (0 << LAMP1) | (1 << LAMP2)
    movwf       FLICKER_STATE
    movlw       12   ; 20 Hz / (50 cycle / 60 s) / 2 = 12
    movwf       FLICKER_COUNT

    call        StartSound

    banksel     T2CON
    bsf         T2ON

#ifndef SIMPLE
    ;banksel     T1CON
    movlw       0xFF
    movwf       TMR1H
    movwf       TMR1L
    bsf         T1ON
#endif

    return

StopAlert:
    bsf         STATE_R

    banksel     LATA
    movlw       (1 << LAMP1) | (1 << LAMP2)
    movwf       FLICKER_STATE
    movwf       LATA

    call        StopSound

#ifdef  SIMPLE
    banksel     T2CON
    bcf         T2ON
#endif
    return

;-------------------------------------------------------------------------------
SetSound1:
    ; banksel     LATA
    movlw       143         ; 16 * 0x100 * (700 Hz / 20 kHz) = 143
    movwf       OSC1_DELTA
    clrf        OSC1_ATT
    movlw       154         ; 16 * 0x100 * (750 Hz / 20 kHz) = 154
    movwf       OSC2_DELTA
    clrf        OSC2_ATT
    movlw       144         ; 5 kHz / (130 cycle / 60 s) / 16 = 144
    movwf       ENVELOPE_CYCLE
    movlw       high(WaveTable1) | 0x80
    movwf       WAVE_TABLE_H
    return

SetSound2:
    ; banksel     LATA
    movlw       123         ; 16 * 0x100 * (600 Hz / 20 kHz) = 123
    movwf       OSC1_DELTA
    clrf        OSC1_ATT
    movlw       133         ; 16 * 0x100 * (650 Hz / 20 kHz) = 133
    movwf       OSC2_DELTA
    movlw       4
    movwf       OSC2_ATT
    movlw       188         ; 5 kHz / (100 cycle / 60 s) / 16 = 188
    movwf       ENVELOPE_CYCLE
    movlw       high(WaveTable2) | 0x80
    movwf       WAVE_TABLE_H
    return

SetSound3:
    ; banksel     LATA
    movlw       113         ; 16 * 0x100 * (550 Hz / 20 kHz) = 113
    movwf       OSC1_DELTA
    movlw       4
    movwf       OSC1_ATT
    movlw       133         ; 16 * 0x100 * (650 Hz / 20 kHz) = 133
    movwf       OSC2_DELTA
    clrf        OSC2_ATT
    movlw       160         ; 5 kHz / (117 cycle / 60 s) / 16 = 160
    movwf       ENVELOPE_CYCLE
    movlw       high(WaveTable1) | 0x80
    movwf       WAVE_TABLE_H
    return

SetSound4:
    ; banksel     LATA
    movlw       107         ; 16 * 0x100 * (523 Hz / 20 kHz) = 107
    movwf       OSC1_DELTA
    movlw       6
    movwf       OSC1_ATT
    movlw       135         ; 16 * 0x100 * (660 Hz / 20 kHz) = 135
    movwf       OSC2_DELTA
    clrf        OSC2_ATT
    movlw       188         ; 5 kHz / (100 cycle / 60 s) / 16 = 188
    movwf       ENVELOPE_CYCLE
    movlw       high(WaveTable1) | 0x80
    movwf       WAVE_TABLE_H
    return

SetSound5:
    ; banksel     LATA
    movlw       92          ; 16 * 0x100 * (450 Hz / 20 kHz) = 92
    movwf       OSC1_DELTA
    clrf        OSC1_ATT
    movlw       113         ; 16 * 0x100 * (550 Hz / 20 kHz) = 113
    movwf       OSC2_DELTA
    clrf        OSC2_ATT
    movlw       160         ; 5 kHz / (117 cycle / 60 s) / 16 = 160
    movwf       ENVELOPE_CYCLE
    movlw       high(WaveTable2) | 0x80
    movwf       WAVE_TABLE_H
    return

StartSound:
    clrf        WAVE_TABLE_L
    clrf        OSC1_ACC_L
    clrf        OSC1_ACC_H
    clrf        OSC2_ACC_L
    clrf        OSC2_ACC_H
    movlw       4
    movwf       ENVELOPE_COUNT_H
    movfw       ENVELOPE_CYCLE
    movwf       ENVELOPE_COUNT_L

#ifdef SIMPLE
    banksel     RA2PPS
    movlw       0x03    ; PWM3
    movwf       RA2PPS
    banksel     RA1PPS
    movlw       0x04    ; PWM4
    movwf       RA1PPS
    banksel     TRISA
    bcf         TRISA, SPEAKER1
    bcf         TRISA, SPEAKER2

    banksel     PWM4CON
    movlw       CONF(PWM4CON,
                    EN, 1, // PWM module is enabled
                    POL, 1, // PWM output is inverted
                )
    movwf       PWM4CON
#else
    banksel     RC5PPS
    movlw       0x03    ; PWM3
    movwf       RC5PPS
    banksel     PWM3CON
#endif
    movlw       CONF(PWM3CON,
                    EN, 1, // PWM module is enabled
                    POL, 0, // PWM output is normal
                )
    movwf       PWM3CON

    return

StopSound:
    banksel     PWM3CON
    clrf        PWM3CON
#ifdef  SIMPLE
    clrf        PWM4CON
#endif
    ; Note that Timer2 should not be disabled here,
    ; since it may still be used to control the motors.


#ifdef SIMPLE
    banksel     RA2PPS
    clrf        RA2PPS ; LATA
    clrf        RA1PPS ; LATA
    banksel     TRISA
    bsf         TRISA, SPEAKER1
    bsf         TRISA, SPEAKER2
#else
    banksel     RC5PPS
    clrf        RC5PPS ; LATC
    banksel     LATC
    bcf         LATC, SPEAKER
#endif
    return

;-------------------------------------------------------------------------------
FlickerLamps:
    btfsc       STATE_R
    return
    banksel     LATA
    decfsz      FLICKER_COUNT, F
    return
    movlw       12   ; 20 Hz / (50 cycle / 60 s) / 2 = 12
    movwf       FLICKER_COUNT
    movfw       FLICKER_STATE
    xorlw       (1 << LAMP1) | (1 << LAMP2) ; toggle
    movwf       FLICKER_STATE
    return

;-------------------------------------------------------------------------------
    PSECT waveTable1, pure, class=CODE, delta=2, reloc=0x100, size=0x100
WaveTable1:
#include "wavtable1.inc"
;-------------------------------------------------------------------------------
    PSECT waveTable2, pure, class=CODE, delta=2, reloc=0x100, size=0x100
WaveTable2:
#include "wavtable2.inc"
;-------------------------------------------------------------------------------
    PSECT lampPFMCycles, pure, class=CODE, delta=2, reloc=0x100, size=0x80
LampPFMCycles:
#include "lampcycles.inc"
