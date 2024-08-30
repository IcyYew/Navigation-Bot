#include "LED.h"



void botStatus_init(void) {
    SYSCTL_RCGCGPIO_R |= 0b100010;
    GPIO_PORTF_DIR_R |= 0b1100;
    GPIO_PORTF_DEN_R |= 0b1100;
    GPIO_PORTB_DIR_R |= 0b1000;
    GPIO_PORTB_DEN_R |= 0b1000;
}

void botStatus_update(char status) {
    if (status == 0) {
        //Red LED -> PF2
        GPIO_PORTF_DATA_R |= 0b100;
        //Make sure other LEDs are off
        GPIO_PORTF_DATA_R &= 0b0100;
        GPIO_PORTB_DATA_R &= 0b0000;
    }
    else if (status == 1) {
        //Yellow LED -> PF3
        GPIO_PORTF_DATA_R |= 0b1000;
        //Make sure other LEDs are off
        GPIO_PORTF_DATA_R &= 0b1000;
        GPIO_PORTB_DATA_R &= 0b0000;
    }
    else if (status == 2) {
        //Green LED -> PF4
        GPIO_PORTF_DATA_R |= 0b10000;
        //Make sure other LEDs are off
        GPIO_PORTF_DATA_R &= 0b10000;
    }
}
