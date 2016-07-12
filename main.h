#include <18F26K80.h>
#device adc=16

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES SOSC_DIG                 //Digital mode, I/O port functionality of RC0 and RC1
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES HSH                      //High speed Osc, high power 16MHz-25MHz
#FUSES NOPLLEN                  //4X HW PLL disabled, 4X PLL enabled in software
#FUSES BROWNOUT
#FUSES PUT
#FUSES NOIESO
#FUSES NOFCMEN
#FUSES NOPROTECT
#FUSES CANC

#use delay(clock = 20000000)

// LIGHT OUTPUTS
#define LEFT_OUT_PIN   PIN_A0
#define RIGHT_OUT_PIN  PIN_A1
#define BRAKE_OUT_PIN  PIN_A2
#define HEAD_OUT_PIN   PIN_A3
#define STROBE_OUT_PIN PIN_A5

// SWITCH INPUTS
#define RIGHT_IN_PIN   PIN_B0
#define LEFT_IN_PIN    PIN_B1
#define HAZARD_IN_PIN  PIN_B2
#define HEAD_IN_PIN    PIN_B3
#define REGEN_IN_PIN   PIN_B4
#define MECH_IN_PIN    PIN_B5

// HEARTBEAT LED
#define LED_PIN        PIN_C0

// State machine states
typedef enum
{
    IDLE,
    BLINK,
    N_STATES
} blinker_state_t;
