#include "main.h"
#include "can_telem.h"
#include "can18F4580_mscp.c"

// Timing periods
#define BLINK_PERIOD_MS    200
#define DEBOUNCE_PERIOD_MS  10 // Hardware switch debounce period

// Debounces a hardware pin
#define DEBOUNCE                               \
    int16 i;                                   \
    for (i = 0 ; i < DEBOUNCE_PERIOD_MS ; i++) \
    {                                          \
        delay_ms(1);                           \
    }

static int1            gb_blink;
static blinker_state_t g_state;

static int1            gb_left_sig;
static int1            gb_right_sig;
static int1            gb_hazard_sig;
static int1            gb_regen_sig;
static int1            gb_mech_sig;
static int1            gb_strobe_sig;

#int_timer2
void isr_timer2(void)
{
    static int16 ms = 0;
    if (ms >= BLINK_PERIOD_MS)
    {
        ms = 0;
        gb_blink = true;
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
                break;
            case COMMAND_RIGHT_SIGNAL_ID:
                gb_right_sig = !gb_right_sig;
                break;
            case COMMAND_HAZARD_SIGNAL_ID:
                gb_hazard_sig = !gb_hazard_sig;
                break;
            case COMMAND_BPS_TRIP_SIGNAL_ID:
                gb_strobe_sig = true;
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
                break;
            case COMMAND_RIGHT_SIGNAL_ID:
                gb_right_sig = !gb_right_sig;
                break;
            case COMMAND_HAZARD_SIGNAL_ID:
                gb_hazard_sig = !gb_hazard_sig;
                break;
            case COMMAND_BPS_TRIP_SIGNAL_ID:
                gb_strobe_sig = true;
                break;
            default:
                break;
        }
    }
}

void idle_state(void)
{
    if (gb_strobe_sig == true)
    {
        // The BPS has tripped, go immediately to the trip state
        g_state = BPS_TRIP;
        return;
    }
    
    // Check the regen brake switch
    if ((input_state(REGEN_IN_PIN) == 1) && (gb_regen_sig == false))
    {
        DEBOUNCE;
        if (input_state(REGEN_IN_PIN == 1))
        {
            // If the regen switch was turned on, set the regen flag
            gb_regen_sig = true;
        }
    }
    else if ((input_state(REGEN_IN_PIN) == 0) && (gb_regen_sig == true))
    {
        DEBOUNCE;
        if (input_state(REGEN_IN_PIN == 0))
        {
            // If the regen switch was turned off, clear the regen flag
            gb_regen_sig = false;
        }
    }
    
    // Check the mechanical brake switch
    if ((input_state(MECH_IN_PIN) == 1) && (gb_mech_sig == false))
    {
        DEBOUNCE;
        if (input_state(MECH_IN_PIN == 1))
        {
            // If the mechanical switch was turned on, set the regen flag
            gb_mech_sig = true;
        }
    }
    else if ((input_state(MECH_IN_PIN) == 0) && (gb_mech_sig == true))
    {
        DEBOUNCE;
        if (input_state(MECH_IN_PIN == 0))
        {
            // If the mechanical switch was turned off, clear the regen flag
            gb_mech_sig = false;
        }
    }
    
    // Turn on the brake lights if either brake switch is on
    if ((gb_regen_sig | gb_mech_sig) == true)
    {
        output_high(BRAKE_OUT_PIN);
    }
    else
    {
        output_low(BRAKE_OUT_PIN);
    }
    
    if (gb_blink == true)
    {
        // Time to blink
        g_state = BLINK;
    }
    else
    {
        // Return to idle
        g_state = IDLE;
    }
}

void blink_state(void)
{
    gb_blink = false;
    
    // Blink heartbeat LED
    output_toggle(LED_PIN);
    
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
        // (Condition) ? (Action if true) : (Action if false)
        (gb_left_sig == true)  ? output_toggle(LEFT_OUT_PIN)  : output_low(LEFT_OUT_PIN);
        (gb_right_sig == true) ? output_toggle(RIGHT_OUT_PIN) : output_low(RIGHT_OUT_PIN);
    }
    
    g_state = IDLE;
}

void bps_trip_state(void)
{
    // Turn off all lights
    output_low(LEFT_OUT_PIN);
    output_low(RIGHT_OUT_PIN);
    output_low(BRAKE_OUT_PIN);
    
    // Turn on the strobe light
    output_high(STROBE_OUT_PIN);
    
    // The BPS has tripped, the blinker will fall into this state and will not
    // exit until the car is restarted
    g_state = BPS_TRIP; 
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
            case BPS_TRIP:
                bps_trip_state();
                break;
            default:
                break;
        }
    }
}
