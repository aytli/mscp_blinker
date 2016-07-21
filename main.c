#include "main.h"
#include "can_telem.h"
#include "can18F4580_mscp.c"

// Timing periods
#define BLINK_PERIOD_MS    500
#define STROBE_PERIOD_MS    50
#define DEBOUNCE_PERIOD_MS  10 // Hardware switch debounce period

// Debounces a hardware pin
#define DEBOUNCE                               \
    int16 i;                                   \
    for (i = 0 ; i < DEBOUNCE_PERIOD_MS ; i++) \
    {                                          \
        delay_ms(1);                           \
    }

static int1            gb_left_sig;
static int1            gb_right_sig;
static int1            gb_hazard_sig;
static int1            gb_regen_sig;
static int1            gb_mech_sig;
static int1            gb_strobe_sig;
static int1            gb_blink;
static blinker_state_t g_state;

void blinker_init(void)
{
    gb_left_sig      = false;
    gb_right_sig     = false;
    gb_hazard_sig    = false;
    gb_regen_sig     = false;
    gb_mech_sig      = false;
    gb_strobe_sig    = false;
    
    // Turn off all lights on startup
    output_low(LEFT_OUT_PIN);
    output_low(RIGHT_OUT_PIN);
    output_low(BRAKE_OUT_PIN);
    output_low(STROBE_OUT_PIN);
}

#int_timer2
void isr_timer2(void)
{
    static int16 ms = 0;
    if (ms >= BLINK_PERIOD_MS)
    {
        ms = 0;
        gb_blink = true;
        
        // Blink heartbeat LED
        output_toggle(LED_PIN);
    }
    else
    {
        ms++;
    }
}

// CAN receive buffer 0 interrupt
#int_canrx0
void isr_canrx0()
{
    int32 rx_id;
    int8  rx_len;
    int8  rx_data[8];
    struct rx_stat rxstat;
    
    if (can_getd(rx_id, rx_data, rx_len, rxstat))
    {
        // A CAN command was received, set the appropriate flag
        switch(rx_id)
        {
            case COMMAND_LEFT_SIGNAL_ID:
                gb_left_sig = !gb_left_sig;
                if (gb_left_sig == true)
                {
                    gb_right_sig = false;
                }
                break;
            case COMMAND_RIGHT_SIGNAL_ID:
                gb_right_sig = !gb_right_sig;
                if (gb_right_sig == true)
                {
                    gb_left_sig = false;
                }
                break;
            case COMMAND_HAZARD_SIGNAL_ID:
                gb_hazard_sig = !gb_hazard_sig;
                if (gb_hazard_sig == true)
                {
                    // If the hazard signal is turned on, reset the turn signals
                    output_low(LEFT_OUT_PIN);
                    output_low(RIGHT_OUT_PIN);
                }
                break;
            case COMMAND_BPS_TRIP_SIGNAL_ID:
                gb_strobe_sig = true;
                break;
            case COMMAND_PMS_BRAKE_LIGHT_ID:
                gb_mech_sig = !gb_mech_sig;
                break;
            default:
                break;
        }
    }
}

// CAN receive buffer 1 interrupt
#int_canrx1
void isr_canrx1()
{
    int32 rx_id;
    int8  rx_len;
    int8  rx_data[8];
    struct rx_stat rxstat;
    
    if (can_getd(rx_id, rx_data, rx_len, rxstat))
    {
        // A CAN command was received, set the appropriate flag
        switch(rx_id)
        {
            case COMMAND_LEFT_SIGNAL_ID:
                gb_left_sig = !gb_left_sig;
                if (gb_left_sig == true)
                {
                    gb_right_sig = false;
                }
                break;
            case COMMAND_RIGHT_SIGNAL_ID:
                gb_right_sig = !gb_right_sig;
                if (gb_right_sig == true)
                {
                    gb_left_sig = false;
                }
                break;
            case COMMAND_HAZARD_SIGNAL_ID:
                gb_hazard_sig = !gb_hazard_sig;
                if (gb_hazard_sig == true)
                {
                    // If the hazard signal is turned on, reset the turn signals
                    output_low(LEFT_OUT_PIN);
                    output_low(RIGHT_OUT_PIN);
                }
                break;
            case COMMAND_BPS_TRIP_SIGNAL_ID:
                gb_strobe_sig = true;
                break;
            case COMMAND_PMS_BRAKE_LIGHT_ID:
                gb_mech_sig = !gb_mech_sig;
                break;
            default:
                break;
        }
    }
}

void idle_state(void)
{
    // Check the strobe signal
    if (gb_strobe_sig == true)
    {
        // The BPS has tripped, go immediately to the trip state
        g_state = BPS_TRIP;
        return;
    }
    
    // Turn on the brake lights if either brake switch is on
    // Ternary statement
    // (Condition)                ? (Action if true)           : (Action if false)
    (gb_regen_sig || gb_mech_sig) ? output_high(BRAKE_OUT_PIN) : output_low(BRAKE_OUT_PIN);
    
    if (gb_blink == true)
    {
        // Time to blink
        g_state = BLINK;
    }
    else
    {
        // Check the switches if not blinking
        g_state = CHECK_SWITCHES;
    }
}

void blink_state(void)
{
    gb_blink = false;
    
    if (gb_hazard_sig == true)
    {
        // Hazard lights are active, blink both turn signals
        output_toggle(LEFT_OUT_PIN);
        output_toggle(RIGHT_OUT_PIN);
    }
    else
    {
        // Hazard lights are not active, blink turn signals if needed
        
        // Ternary statements
        // (Condition)         ? (Action if true)             : (Action if false)
        (gb_left_sig == true)  ? output_toggle(LEFT_OUT_PIN)  : output_low(LEFT_OUT_PIN);
        (gb_right_sig == true) ? output_toggle(RIGHT_OUT_PIN) : output_low(RIGHT_OUT_PIN);
    }
    
    // Return to the idle state
    g_state = IDLE;
}

void check_switches_state(void)
{
    static int1 b_regen_switch  = false;
    static int1 b_mech_switch   = false;
    static int1 b_left_switch   = false;
    static int1 b_right_switch  = false;
    static int1 b_hazard_switch = false;
    
    // Check the regen brake switch
    if ((input_state(REGEN_IN_PIN) == 1) && (b_regen_switch == false))
    {
        DEBOUNCE;
        if (input_state(REGEN_IN_PIN) == 1)
        {
            b_regen_switch = true;
            gb_regen_sig = true;
        }
    }
    else if ((input_state(REGEN_IN_PIN) == 0) && (b_regen_switch == true))
    {
        DEBOUNCE;
        if (input_state(REGEN_IN_PIN) == 0)
        {
            b_regen_switch = false;
            gb_regen_sig = false;
        }
    }
    
    // Check the mechanical brake switch
    if ((input_state(MECH_IN_PIN) == 1) && (b_mech_switch == false))
    {
        DEBOUNCE;
        if (input_state(MECH_IN_PIN) == 1)
        {
            b_mech_switch = true;
            gb_mech_sig   = true;
        }
    }
    else if ((input_state(MECH_IN_PIN) == 0) && (b_mech_switch == true))
    {
        DEBOUNCE;
        if (input_state(MECH_IN_PIN) == 0)
        {
            b_mech_switch = false;
            gb_mech_sig   = false;
        }
    }
    
    // CAN BUS CONTROLLED LIGHTS
    // The following switches are also controlled by CAN bus,
    // need seperate flags to store the state of the hardware switch
    
    // Check the left turn signal
    if ((input_state(LEFT_IN_PIN) == 1) && (b_left_switch == false))
    {
        DEBOUNCE;
        if (input_state(LEFT_IN_PIN) == 1)
        {
            b_left_switch = true;
            gb_left_sig    = true;
            gb_right_sig   = false; // Clear the right flag
        }
    }
    else if ((input_state(LEFT_IN_PIN) == 0) && (b_left_switch == true))
    {
        DEBOUNCE;
        if (input_state(LEFT_IN_PIN) == 0)
        {
            b_left_switch = false;
            gb_left_sig    = false;
        }
    }
    
    // Check the right turn signal
    if ((input_state(RIGHT_IN_PIN) == 1) && (b_right_switch == false))
    {
        DEBOUNCE;
        if (input_state(RIGHT_IN_PIN) == 1)
        {
            b_right_switch = true;
            gb_right_sig    = true;
            gb_left_sig     = false; // Clear the left flag
        }
    }
    else if ((input_state(RIGHT_IN_PIN) == 0) && (b_right_switch == true))
    {
        DEBOUNCE;
        if (input_state(RIGHT_IN_PIN) == 0)
        {
            b_right_switch = false;
            gb_right_sig    = false;
        }
    }
    
    // Check the hazard switch
    if ((input_state(HAZARD_IN_PIN) == 1) && (b_hazard_switch == false))
    {
        DEBOUNCE;
        if (input_state(HAZARD_IN_PIN) == 1)
        {
            b_hazard_switch = true;
            gb_hazard_sig    = true;
        }
    }
    else if ((input_state(HAZARD_IN_PIN) == 0) && (b_hazard_switch == true))
    {
        DEBOUNCE;
        if (input_state(HAZARD_IN_PIN) == 0)
        {
            b_hazard_switch = false;
            gb_hazard_sig    = false;
        }
    }
    
    // Return to idle
    g_state = IDLE;
}

void bps_trip_state(void)
{
    // Turn off all lights
    output_low(LEFT_OUT_PIN);
    output_low(RIGHT_OUT_PIN);
    output_low(BRAKE_OUT_PIN);
    
    // Pulse the strobe light
    while(true)
    {
        output_toggle(STROBE_OUT_PIN);
        delay_ms(STROBE_PERIOD_MS);
    }
    
    // The BPS has tripped, the blinker will fall into this state and will not
    // exit until the car is restarted
}

void main()
{
    // Enable CAN receive interrupts
    clear_interrupt(INT_CANRX0);
    enable_interrupts(INT_CANRX0);
    clear_interrupt(INT_CANRX1);
    enable_interrupts(INT_CANRX1);
    
    // Enable timer interrupts
    setup_timer_2(T2_DIV_BY_4,79,16); // Timer 2 set up to interrupt every 1ms with a 20MHz clock
    enable_interrupts(INT_TIMER2);
    enable_interrupts(GLOBAL);
    
    blinker_init();
    can_init();
    
    g_state = IDLE;
    
    while(true)
    {
        switch(g_state)
        {
            case IDLE:
                idle_state();
                break;
            case BLINK:
                blink_state();
                break;
            case CHECK_SWITCHES:
                check_switches_state();
                break;
            case BPS_TRIP:
                bps_trip_state();
                break;
            default:
                break;
        }
    }
}
