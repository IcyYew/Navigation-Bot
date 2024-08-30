/*
 * movement.c
 *
 *  Created on: Sep 6, 2023
 *      Author: ntegeler
 */


#include "movement.h"
#include "open_interface.h"

extern int crossLines;
volatile unsigned char lineCrossed = 0;
volatile unsigned char bumpedLastLeft = 0;
volatile unsigned char bumpedLastRight = 0;

void move_forward(oi_t *sensor, int centimeters) {
    double sum = 0;
    oi_setWheels(200, 200);

    while (sum < centimeters) {
        oi_update(sensor);
        sum += sensor->distance;
    }

    oi_setWheels(0, 0);
}

void move_forward_without_stop(oi_t *sensor, int centimeters) {
    double sum = 0;
    oi_setWheels(200, 200);

    while (sum < centimeters) {
        oi_update(sensor);
        sum += sensor->distance;
    }
}

void move_backwards(oi_t *sensor, int centimeters) {
    double sum = 0;
    oi_setWheels(-200, -200);

    while (abs(sum) < centimeters) {
        oi_update(sensor);
        sum += sensor->distance;
    }

    oi_setWheels(0, 0);
}

void move_forward_with_avoid(oi_t *sensor, int centimeters) {
    double sum = 0;
    oi_setWheels(200, 200);

    while (sum < centimeters) {
        oi_update(sensor);
        sum += sensor->distance;

        if (sensor->bumpLeft) {
            avoid_object_left(sensor);
            break;
        } else if (sensor->bumpRight) {
            avoid_object_right(sensor);
            break;
        }

        if (sensor->cliffFrontLeftSignal < 1000) {
            oi_setWheels(0, 0);
            oi_update(sensor);
            move_backwards(sensor, 25);
            turn_clockwise(sensor, 125);
            lcd_printf("front left cliff");
            break;
        } else if (sensor->cliffLeftSignal < 1000) {
            oi_setWheels(0, 0);
            oi_update(sensor);
            move_backwards(sensor, 25);
            turn_clockwise(sensor, 90);
            lcd_printf("left cliff");
            break;
        } else if (sensor->cliffFrontRight) {
            oi_setWheels(0, 0);
            oi_update(sensor);
            move_backwards(sensor, 25);
            turn_counterClockwise(sensor, 125);
            lcd_printf("front right cliff");
            break;
        } else if (sensor->cliffRight) {
            oi_setWheels(0, 0);
            oi_update(sensor);
            move_backwards(sensor, 25);
            turn_counterClockwise(sensor, 90);
            lcd_printf("right cliff");
            break;
        }

        if (!crossLines) {
            if (sensor->cliffFrontLeftSignal > 2700) {
                oi_setWheels(0, 0);
                oi_update(sensor);
                move_backwards(sensor, 25);
                turn_clockwise(sensor, 80);
                lcd_printf("left cliff");
                break;
            } else if (sensor->cliffLeftSignal > 2700) {
                oi_setWheels(0, 0);
                oi_update(sensor);
                move_backwards(sensor, 25);
                turn_clockwise(sensor, 45);
                lcd_printf("left cliff");
                break;
            } else if (sensor->cliffFrontRightSignal > 2700) {
                oi_setWheels(0, 0);
                oi_update(sensor);
                move_backwards(sensor, 25);
                turn_counterClockwise(sensor, 80);
                lcd_printf("right cliff");
                break;
            } else if (sensor->cliffRightSignal > 2700) {
                oi_setWheels(0, 0);
                oi_update(sensor);
                move_backwards(sensor, 25);
                turn_counterClockwise(sensor, 45);
                lcd_printf("right cliff");
                break;
            }
        } else {
            if (sensor->cliffFrontLeftSignal > 2700) {
                oi_setWheels(0, 0);
                oi_update(sensor);
                move_forward(sensor, 400);
                lineCrossed = 1;
                //turn_clockwise(sensor, 90);
                lcd_printf("left cliff");
                break;
            } else if (sensor->cliffLeftSignal > 2700) {
                oi_setWheels(0, 0);
                oi_update(sensor);
                move_forward(sensor, 400);
                lineCrossed = 1;
                //turn_clockwise(sensor, 90);
                lcd_printf("left cliff");
                break;
            } else if (sensor->cliffFrontRightSignal > 2700) {
                oi_setWheels(0, 0);
                oi_update(sensor);
                move_forward(sensor, 400);
                lineCrossed = 1;
                //turn_counterClockwise(sensor, 90);
                lcd_printf("right cliff");
                break;
            } else if (sensor->cliffRightSignal > 2700) {
                oi_setWheels(0, 0);
                oi_update(sensor);
                move_forward(sensor, 400);
                lineCrossed = 1;
                //turn_counterClockwise(sensor, 90);
                lcd_printf("right cliff");
                break;
            }
        }


    }

    oi_setWheels(0, 0);
}

void turn_clockwise(oi_t *sensor, int degrees) {
    double angleSum = 0;
    oi_setWheels(-50, 50);

    while (angleSum > (degrees*-1)) {
        oi_update(sensor);
        angleSum += sensor->angle;
    }

    oi_setWheels(0, 0);
}

void turn_counterClockwise(oi_t *sensor, int degrees) {
    double angleSum = 0;
    oi_setWheels(50, -50);

    while (angleSum < degrees) {
        oi_update(sensor);
        angleSum += sensor->angle;
    }

    oi_setWheels(0, 0);
}

void avoid_object_left(oi_t *sensor) {
    move_backwards(sensor, 50);
    turn_clockwise(sensor, 25);
    bumpedLastLeft = 1;
    //move_forward(sensor, 50);
}

void avoid_object_right(oi_t *sensor) {
    move_backwards(sensor, 50);
    turn_counterClockwise(sensor, 25);
    bumpedLastRight = 1;
    //move_forward(sensor, 50);
}





