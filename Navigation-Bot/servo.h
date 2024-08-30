/*
 * servo.h
 *
 *  Created on: Nov 1, 2023
 *      Author: jddwight
 */

#include "Timer.h"
#include "button.h"
#include "lcd.h"
#include "uart.h"

#ifndef SERVO_H_
#define SERVO_H_

void servo_init(void);

int servo_move(float degrees);

typedef struct {
    int right;
    int left;
} servo_cal_vals;

servo_cal_vals servo_cal(void);

#endif /* SERVO_H_ */
