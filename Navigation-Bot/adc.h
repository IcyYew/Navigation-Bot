/*
 * adc.h
 *
 *  Created on: Oct 18, 2023
 *      Author: ntegeler
 */

#ifndef ADC_H_
#define ADC_H_
#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
void adc_init(void);
int adc_read(void);

#endif /* ADC_H_ */
