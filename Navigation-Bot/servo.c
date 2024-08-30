/*
 * servo.c
 *
 *  Created on: Nov 1, 2023
 *      Author: jddwight
 */

#include "servo.h"

volatile extern int button_event;
volatile extern int button_num;

void servo_init() {
    // clock enables
    SYSCTL_RCGCGPIO_R |= 0x02; // portb clock
    SYSCTL_RCGCTIMER_R |= 0x02; // timer 1

    // GPIO
    GPIO_PORTB_DEN_R |= 0x20; // pin 5 DEN
    GPIO_PORTB_DIR_R |= 0x20; // pin 5 output
    GPIO_PORTB_AFSEL_R |= 0x20; // pin 5 peripheral select
    GPIO_PORTB_PCTL_R |= 0x00700000; // T1CCP1

    // timer
    TIMER1_CTL_R &= 0xFEFF; // disable timer b
    TIMER1_CFG_R |= 0x4; // 16 bit mode
    TIMER1_TBMR_R |= 0x00A; // PWM mode in periodic
    TIMER1_TBMR_R &= 0xFFA; // clear capture mode bit to ensure periodic
    TIMER1_TBILR_R = 0xE200; // preset
    TIMER1_TBPR_R = 0x04; // preset
    TIMER1_CTL_R |= 0x0100; // re-enable clock
}

int servo_move(float degrees) {
    int rightBound = 313132;
    int leftBound = 283804;

    int span = rightBound - leftBound;
    int countPerDegree = span / 180;

    int position = countPerDegree * (int)degrees;
    int finalPosition = rightBound - position;

    TIMER1_TBMATCHR_R = finalPosition - 0x40000;
    TIMER1_TBPMR_R = 0x04;

    timer_waitMillis(50);

    return (int)degrees;
}

servo_cal_vals servo_cal() {
    button_init();
    init_button_interrupts();
    lcd_init();
    servo_cal_vals bounds;

    TIMER1_TBMATCHR_R = 0x8C10;
    TIMER1_TBPMR_R = 0x04;

    bounds.left = 0;
    bounds.right = 0;

    int direction = 1;
    while (bounds.left == 0 || bounds.right == 0) {
        if (direction == 1) {
            lcd_printf("1: right 1 deg\n2: right 5 deg\n3: swap dir\n4: set 0 deg cal");
        } else {
            lcd_printf("1: left 1 deg\n2: left 5 deg\n3: swap dir\n4: set 180 deg cal");
        }

        if (button_event == 1) {
            if (direction == 1) {
                if (button_num == 1) {
                    TIMER1_TBMATCHR_R += 156;
                    TIMER1_TBPMR_R = 0x04;
                } else if (button_num == 2) {
                    TIMER1_TBMATCHR_R += 780;
                    TIMER1_TBPMR_R = 0x04;
                } else if (button_num == 3) {
                    direction = 2;
                } else if (button_num == 4) {
                    int set = (TIMER1_TBPMR_R << 16) + TIMER1_TBMATCHR_R;
                    bounds.right = set;
                }
            } else {
                if (button_num == 1) {
                    TIMER1_TBMATCHR_R -= 156;
                    TIMER1_TBPMR_R = 0x04;
                } else if (button_num == 2) {
                    TIMER1_TBMATCHR_R -= 780;
                    TIMER1_TBPMR_R = 0x04;
                } else if (button_num == 3) {
                    direction = 1;
                } else if (button_num == 4) {
                    int set = (TIMER1_TBPMR_R << 16) + TIMER1_TBMATCHR_R;
                    bounds.left = set;
                }
            }

            button_event = 0;
        }
    }

    lcd_printf("Bound R: %d\nBound L: %d", bounds.right, bounds.left);

    return bounds;
}


