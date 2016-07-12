#include "main.h"
#include "can_telem.h"
#include "can18F4580_mscp.c"

#define BLINK_PERIOD_MS 200

static int1            gb_blink;
static blinker_state_t g_state;

static int1            gb_left_sig;
static int1            gb_right_sig;
static int1            gb_hazard_sig;
static int1            gb_headlight_sig;
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
                break;
            case COMMAND_RIGHT_SIGNAL_ID:
                gb_right_sig = !gb_right_sig;
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
            default:
                break;
        }
    }
}

void idle_state(void)
{
}

void check_switches_state(void)
{
}

void blink_state(void)
{
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
            case CHECK_SWITCHES:
                check_switches_state();
                break;
            case BLINK:
                blink_state();
                break;
            default:
                break;
        }
    }
}
