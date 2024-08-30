/*
 * adc.c
 *
 *  Created on: Oct 18, 2023
 *      Author: ntegeler
 */

#include "adc.h"

void adc_init(void){
    NVIC_EN0_R |=  0x00020000;            // enable interrupt for ADC0 ss0 with bit 17
    SYSCTL_RCGCGPIO_R |= 0x02;           //enable clock port B
    SYSCTL_RCGCADC_R |= 0x1;              // enable clock for ADC module 0 (ADC0)

    GPIO_PORTB_AFSEL_R |= 0x10;        // PB4 for ADC
    GPIO_PORTB_ADCCTL_R  |= 0x10;     // PB4 for ADC trigger source set to default
    GPIO_PORTB_DEN_R   &= 0xEF;       // disable pb4 as digital
    GPIO_PORTB_DIR_R   &= 0xEF;        // sets pb4 as input
    GPIO_PORTB_AMSEL_R |=  0x10;   // enables pb4 analog function
    GPIO_PORTB_ADCCTL_R = 0x0;      // use default trigger set up ADC0

    ADC0_ACTSS_R     &= 0xE ;               // disables sample sequencer 0 for ADC0
    ADC0_EMUX_R     &=  0xF;               // set up to continuously take samples
    ADC0_SSMUX0_R   = 0x0AAA;     // set up for ss0 to input 10 (AIN10) sample 3 times end
    ADC0_SSCTL0_R |= 0x00000644;        // set up to enable interrupt and end bit enabled after sample 3 bit 2, 6, and 10 used to enable interrupts bit 11 for end bit
    ADC0_IM_R   |=  0x1;  //sends interrupt status to interrupt controller  for SS0
    ADC0_ACTSS_R     |= 0x1 ;               // enables sample sequencer 0 for ADC0
}

int adc_read(void){
    volatile int sensorval = 0;
    //add code to tell SS to sample values

    ADC0_PSSI_R |= 0x1; //tells SS0 to start

    while ((ADC0_RIS_R & 0x1) == 0);

    int i = 0;
    for (i = 0; i < 3; i++){
        sensorval += ADC0_SSFIFO0_R; // add sampled value to sensorval
    }

    return sensorval/3;
}
