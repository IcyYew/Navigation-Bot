

/**
 * main.c
 */

#include "servo.h"
#include "lcd.h"

void main(void)
{
    int counterPos = 0;
    servo_init();
    lcd_init();
    int i = 0;

    while(i <= 90) {


        servo_move(i);
        counterPos = (int)TIMER1_TBMATCHR_R;
        lcd_printf("%d", counterPos);
    }

}
