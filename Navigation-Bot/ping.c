#include "ping.h"
#include "driverlib/interrupt.h"

volatile enum {LOW, HIGH} state;
volatile unsigned int rising_time;
volatile unsigned int falling_time;
volatile unsigned int timer_overflow;

void TIMER3B_Handler(void) {
    if (TIMER3_MIS_R & 0x0400) {
        if (state == LOW) {
            TIMER3_ICR_R |= 0x0400;
            rising_time = TIMER3_TBR_R;
            state = HIGH;
        } else if (state == HIGH) {
            TIMER3_ICR_R |= 0x0400;
            falling_time = TIMER3_TBR_R;
            state = LOW;
        }
    }
}

void ping_init() {
    timer_overflow = 0;

    // enable clocks
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCTIMER_R |= 0x08;
    NVIC_EN1_R |= 0x00000010;

    GPIO_PORTB_DEN_R |= 0x08; // digitally enable PB3
    GPIO_PORTB_AFSEL_R |= 0x08; // set PB3 to use the alternate function
    GPIO_PORTB_PCTL_R |= 0x7000; // use T3CCP1

    // timer initialization
    TIMER3_CTL_R &= 0xFEFF; // disable timer 3
    TIMER3_CFG_R |= 0x4; // configure for 16 bit counter mode
    TIMER3_TBMR_R |= 0x007; // enable edge time mode in capture mode and turn into a count down timer
    TIMER3_CTL_R |= 0x0C00; // enable both edge detection
    TIMER3_TBILR_R = 0xFFFF; // starting num to count down from
    TIMER3_TBPR_R = 0xFF; // 24 bit
    TIMER3_IMR_R |= 0x0400; // enable mask for timer event
    TIMER3_CTL_R |= 0x0100; // re-enable timer B

    TIMER3_ICR_R |= 0x0400;
    IntRegister(INT_TIMER3B, TIMER3B_Handler);
}

/*
 * pulse_width = ping_read();
 * distance = pulse_width * 0.00000003125 * 34000;
 */
float ping_read() {
    rising_time = 0;
    falling_time = 0;

    // add disable timer
    TIMER3_IMR_R &= 0xFBFF;
    TIMER3_CTL_R &= 0xFEFF;

    GPIO_PORTB_AFSEL_R &= 0xF7; // disable alternate function for PB3
    GPIO_PORTB_DEN_R |= 0x08; // digitally enable PB3
    GPIO_PORTB_DIR_R |= 0x08; // set PB3 to output
    GPIO_PORTB_DATA_R &= 0xF7;
    GPIO_PORTB_DATA_R |= 0x08; // set PB3 to high
    timer_waitMillis(0.005); // SuS delay

    // switch to timer mode
    GPIO_PORTB_DATA_R &= 0xF7; // set PB3 to lows
    GPIO_PORTB_DIR_R &= 0xF7; // set PB3 to input
    GPIO_PORTB_AFSEL_R |= 0x08; // set PB3 to use the alternate function

    TIMER3_ICR_R |= 0x0400;

    TIMER3_IMR_R |= 0x0400;
    TIMER3_CTL_R |= 0x0100;
    state = LOW;

    while(rising_time == 0 || falling_time == 0);

    if (rising_time < falling_time) {
        timer_overflow += 1;
        return ((0xFFFFFF - falling_time) + rising_time) * 0.00000003125 * 34000;
    }

    else {
        return (rising_time - falling_time) * 0.00000003125 * 34000;
    }
}
