/*
 * scan.c
 *
 *  Created on: Sep 14, 2023
 *      Author: ntegeler
 */

#include "open_interface.h"
#include "movement.h"
#include "lcd.h"
#include "timer.h"
#include "math.h"
#include "uart.h"
#include "servo.h"
#include "ping.h"
#include "adc.h"

///*
void main() {
    servo_init();
    lcd_init();
    timer_init();
    uart_init();
    adc_init();
    ping_init();


    servo_move(0);

    // add MATCHR calibration to servo.c

    char labels[] = "Degrees\t\tIR Distance\n\r";

    char letter = ' ';

    char toggle = 1;

    char targetTog;

    while (letter != 'm' && letter != 't' && letter != 'h') {
        letter = uart_receive();
    }

    int IRVals[90];
    int pingDistances[90];

    while (letter == 'h' || letter == 't' || letter == 'm') {
        if (letter == 't') {
            int i;
            if (toggle == 1) {
                toggle = 0;
            } else if (toggle == 0) {
                toggle = 1;
            }

            if (toggle == 1) {
                char toggleMessage[] = "Now in Autonomous mode\n\r\nPress 'h' to autonomously seek smallest object or 't' again to switch to manual\n\r\n";

                for (i = 0; i < strlen(toggleMessage); i++) {
                    uart_sendChar(toggleMessage[i]);
                }
            } else if (toggle == 0) {
                char toggleMessage[] = "Now in Manual mode\n\r\nPress 'm' to start manual object seeking or 't' again to switch to autonomous\n\r\n";

                for (i = 0; i < strlen(toggleMessage); i++) {
                    uart_sendChar(toggleMessage[i]);
                }
            }

            letter = uart_receive();
            continue;
        }

        if (toggle == 1) {
            servo_move(0);
            timer_waitMillis(500);
            int i;
            for (i = 0; i < strlen(labels); i++) {
                uart_sendChar(labels[i]);
            }

            for (i = 0; i <= 180; i+=2) {
                servo_move(i);

                char data[20];

                int pulse_width;
                int pingDistance;
                int rawIR1;
                int rawIR2;
                int rawIR3;
                int IRAVG;


                pulse_width = ping_read();
                pingDistance = pulse_width * 0.00000003125 * 34000;

                rawIR1 = adc_read(); // scnr.IR_raw_val
                rawIR2 = adc_read(); // scnr.IR_raw_val
                rawIR3 = adc_read(); // scnr.IR_raw_val
                IRAVG = (rawIR1 + rawIR2 + rawIR3) / 3;

                if ((IRAVG % 50) != 0) {
                    IRAVG -= IRAVG % 50;
                }

                sprintf(data, "%d\t\t%d\n\r", i, IRAVG);

                int j;
                for (j = 0; j < strlen(data); j++) {
                    uart_sendChar(data[j]);
                }


                IRVals[i/2] = IRAVG;
                pingDistances[i/2] = pingDistance;
            }

            int counts[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int startingAngles[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int endingAngles[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int IRDistances[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int count = 0;

            for (i = 0; i < (sizeof(IRVals) / sizeof(IRVals[0])); i++) {

                if (IRVals[i] > 900) {
                    if (i == ((sizeof(IRVals) / sizeof(IRVals[0])) - 1)) { // used to check if we are at the last index of IRVals
                        if (count > 0) { // if we have seen an object
                            int j;
                            for (j = 0; j < (sizeof(counts) / sizeof(counts[0])); j++) { // go until we reach an empty position of the array counts and fill it with the current count
                                if (counts[j] == 0) {
                                    counts[j] = count;
                                    break;
                                }
                            }
                            for (j = 0; j < (sizeof(endingAngles) / sizeof(endingAngles[0])); j++) { // go until we reach an empty position of endingAngles and fill it
                                if (endingAngles[j] == 0) {
                                    endingAngles[j] = i * 2;
                                }
                            }
                        }
                    }

                    if (count == 0) { // if the current measurement is equal to the next one and we are not currently in an object
                        int j;
                        for (j = 0; j < (sizeof(startingAngles) / sizeof(startingAngles[0])); j++) { // go until we reach an empty position and fill it with the starting angle
                            if (startingAngles[j] == 0 && endingAngles[j] == 0) {
                                startingAngles[j] = i * 2;
                                break;
                            }
                        }

                        for (j = 0; j < (sizeof(IRDistances) / sizeof(IRDistances[0])); j++) { // go until we reach an empty position and fill it with the starting angle
                            if (IRDistances[j] == 0) {
                                IRDistances[j] = IRVals[i];
                                break;
                            }
                        }

                        count++;
                    }

                    else if (count != 0) { // if already in object
                        count++;
                    }

                } else if (IRVals[i] <= 900 && count != 0) { // if not in object but were in an object
                    int j;
                    for (j = 0; j < (sizeof(endingAngles) / sizeof(endingAngles[0])); j++) { // go until empty and fill it
                        if (endingAngles[j] == 0) {
                            endingAngles[j] = i * 2;
                            break;
                        }
                    }

                    for (j = 0; j < (sizeof(counts) / sizeof(counts[0])); j++) { // go until empty and fill it
                        if (counts[j] == 0 && count > 1) {
                            counts[j] = count;
                            break;
                        } else if (counts[j] == 0 && count <= 1) {
                            break;
                        }
                    }

                    count = 0;
                }
            }


            int PINGDistances[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int centerAngles[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int linearWidth[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

            if (counts[0] != 0) { // as long as we found an object
                for (i = 0; i < (sizeof(counts) / sizeof(counts[0])); i++) {
                    if (counts[i] == 0) {
                        break;
                    }

                    else {
                        centerAngles[i] = endingAngles[i] - ((endingAngles[i] - startingAngles[i]) / 2); // set center angle value to middle of object
                    }

                }
            }

            for (i = 0; i < (sizeof(centerAngles) / sizeof(centerAngles[i])); i++) {
                if (centerAngles[i] == 0) {
                    break;
                }

                else {
                    PINGDistances[i] = pingDistances[(centerAngles[i] / 2)]; // set ping distance to distance at that angle
                }
            }

            char test[] = "\n\rDistance\tStarting Angle\tEnding Angle\tCount\n\r";

            for (i = 0; i < strlen(test); i++) {
                uart_sendChar(test[i]);
            }



            for (i = 0; i < 10; i++) {
                if (PINGDistances[i] == 0) {
                    break;
                }
                char testData[20];
                sprintf(testData, "%d\t\t%d\t\t%d\t\t%d\n\r", PINGDistances[i], startingAngles[i], endingAngles[i], counts[i]);
                int j;
                for (j = 0; j < strlen(testData); j++) {
                    uart_sendChar(testData[j]);
                }
            }


            for (i = 0; i < sizeof(linearWidth) / sizeof(linearWidth[0]); i++) {
                if (counts[i] == 0) {
                    break;
                }

                if (linearWidth[i] == 0) {
                    linearWidth[i] = PINGDistances[i] * tan(counts[i] * (3.14/180));
                }
            }

            int smallestWidth = 1000;
            int smallestWidthIndex = 0;
            for (i = 0; i < 10; i++) {
                if (linearWidth[i] == 0) {
                    break;
                }

                if (linearWidth[i] < smallestWidth) { // find the object with the smallest count (least linear width)
                    smallestWidth = linearWidth[i];
                    smallestWidthIndex = i;
                }
            }

            char widths[50] = "\nThe linear widths are: \n\r";
            for (i = 0; i < strlen(widths); i++) {
                uart_sendChar(widths[i]);
            }

            for (i = 0; i < 10; i++) {
                if (counts[i] == 0) {
                    break;
                }

                else {
                    char widthVals[30];
                    sprintf(widthVals, "Angle: %d, Width: %d\n\r", centerAngles[i], linearWidth[i]);
                    int j;
                    for (j = 0; j < strlen(widthVals); j++) {
                        uart_sendChar(widthVals[j]);
                    }
                }
            }

            int angleOfSmallest = centerAngles[smallestWidthIndex]; // smallest angle is angle at smallest count
            char result[50];
            sprintf(result, "\nThe smallest object is at %d degrees\n\r\n", angleOfSmallest);

            int goalReached = 0;

            for (i = 0; i < sizeof(PINGDistances) / sizeof(PINGDistances[0]); i++) {
                if (PINGDistances[i] == 0) {
                    break;
                }

                if (PINGDistances[i] <= 10) {
                    lcd_printf("Goal reached");
                    char goalMessage[] = "Goal has been reached. Shutting down program.\n\r";

                    int j;
                    for (j = 0; j < strlen(goalMessage); j++) {
                        uart_sendChar(goalMessage[j]);
                    }

                    goalReached = 1;
                    break;
                }
            }

            if (goalReached == 1) {
                break;
            }

            char plan[100];
            sprintf(plan, "Planned movement is turning to the object at %d degrees and moving %d centimeters\n\rPress 'h' to confirm or 't' to toggle to manual\n\r", angleOfSmallest, PINGDistances[smallestWidthIndex]);

            letter = 'z';

            for (i = 0; i < strlen(result); i++) {
                uart_sendChar(result[i]);
            }

            for (i = 0; i < strlen(plan); i++) {
                uart_sendChar(plan[i]);
            }

            while (letter != 't' && letter != 'h') {
                letter = uart_receive();
            }

            if (letter == 't') {
                continue;
            }

            servo_move(centerAngles[smallestWidthIndex]);
            timer_waitMillis(1000);

            if (centerAngles[smallestWidthIndex] >= 90) { // if the object is at an angle past or equal to 90
                oi_t *sensor_data = oi_alloc();
                oi_init(sensor_data);

                int inBetween = 180 - centerAngles[smallestWidthIndex];
                int final = 90 - inBetween - 15;

                if (final < 0) {
                    final = 0;
                }

                lcd_printf("%d degrees", final);

                turn_counterClockwise(sensor_data, final);
                timer_waitMillis(1000);
                move_forward_with_avoid(sensor_data, (PINGDistances[smallestWidthIndex] * 10) - 100);

                oi_free(sensor_data);
            } else if (centerAngles[smallestWidthIndex] < 90) { // if the object is at an angle before 90
                oi_t *sensor_data = oi_alloc();
                oi_init(sensor_data);

                int final = 90 - centerAngles[smallestWidthIndex] - 15;

                if (final < 0) {
                    final = 0;
                }

                lcd_printf("%d degrees", final);

                turn_clockwise(sensor_data, final);
                timer_waitMillis(1000);
                move_forward_with_avoid(sensor_data, (PINGDistances[smallestWidthIndex] * 10) - 100);

                oi_free(sensor_data);
            }

            letter = uart_receive();

        } else if (toggle == 0) {
            servo_move(0);
            int i;
            timer_waitMillis(1000);
            for (i = 0; i < strlen(labels); i++) {
                uart_sendChar(labels[i]);
            }

            for (i = 0; i <= 180; i+=2) {
                servo_move(i);

                char data[20];

                int pulse_width;
                int pingDistance;
                int rawIR1;
                int rawIR2;
                int rawIR3;
                int IRAVG;

                pulse_width = ping_read();
                pingDistance = pulse_width * 0.00000003125 * 34000;

                rawIR1 = adc_read();
                rawIR2 = adc_read();
                rawIR3 = adc_read();
                IRAVG = (rawIR1 + rawIR2 + rawIR3) / 3;

                if ((IRAVG % 50) != 0) {
                    IRAVG -= IRAVG % 50;
                }

                sprintf(data, "%d\t\t%d\n\r", i, IRAVG);

                int j;
                for (j = 0; j < strlen(data); j++) {
                    uart_sendChar(data[j]);
                }


                IRVals[i/2] = IRAVG;
                pingDistances[i/2] = pingDistance;
            }

            int counts[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int startingAngles[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int endingAngles[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int IRDistances[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int count = 0;

            for (i = 0; i < (sizeof(IRVals) / sizeof(IRVals[0])); i++) {

                if (IRVals[i] > 900) {
                    if (i == ((sizeof(IRVals) / sizeof(IRVals[0])) - 1)) { // used to check if we are at the last index of IRVals
                        if (count > 0) { // if we have seen an object
                            int j;
                            for (j = 0; j < (sizeof(counts) / sizeof(counts[0])); j++) { // go until we reach an empty position of the array counts and fill it with the current count
                                if (counts[j] == 0) {
                                    counts[j] = count;
                                    break;
                                }
                            }
                            for (j = 0; j < (sizeof(endingAngles) / sizeof(endingAngles[0])); j++) { // go until we reach an empty position of endingAngles and fill it
                                if (endingAngles[j] == 0) {
                                    endingAngles[j] = i * 2;
                                }
                            }
                        }
                    }

                    if (count == 0) { // if the current measurement is equal to the next one and we are not currently in an object
                        int j;
                        for (j = 0; j < (sizeof(startingAngles) / sizeof(startingAngles[0])); j++) { // go until we reach an empty position and fill it with the starting angle
                            if (startingAngles[j] == 0 && endingAngles[j] == 0) {
                                startingAngles[j] = i * 2;
                                break;
                            }
                        }

                        for (j = 0; j < (sizeof(IRDistances) / sizeof(IRDistances[0])); j++) { // go until we reach an empty position and fill it with the starting angle
                            if (IRDistances[j] == 0) {
                                IRDistances[j] = IRVals[i];
                                break;
                            }
                        }

                        count++;
                    }

                    else if (count != 0) { // if already in object
                        count++;
                    }

                } else if (IRVals[i] <= 900 && count != 0) { // if not in object but were in an object
                    int j;
                    for (j = 0; j < (sizeof(endingAngles) / sizeof(endingAngles[0])); j++) { // go until empty and fill it
                        if (endingAngles[j] == 0) {
                            endingAngles[j] = i * 2;
                            break;
                        }
                    }

                    for (j = 0; j < (sizeof(counts) / sizeof(counts[0])); j++) { // go until empty and fill it
                        if (counts[j] == 0 && count > 1) {
                            counts[j] = count;
                            break;
                        } else if (counts[j] == 0 && count <= 1) {
                            break;
                        }
                    }

                    count = 0;
                }
            }


            int PINGDistances[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int centerAngles[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            int linearWidth[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

            if (counts[0] != 0) { // as long as we found an object
                for (i = 0; i < (sizeof(counts) / sizeof(counts[0])); i++) {
                    if (counts[i] == 0) {
                        break;
                    }

                    else {
                        centerAngles[i] = endingAngles[i] - ((endingAngles[i] - startingAngles[i]) / 2); // set center angle value to middle of object
                    }

                }
            }

            for (i = 0; i < (sizeof(centerAngles) / sizeof(centerAngles[i])); i++) {
                if (centerAngles[i] == 0) {
                    break;
                }

                else {
                    PINGDistances[i] = pingDistances[(centerAngles[i] / 2)]; // set ping distance to distance at that angle
                }
            }

            char test[] = "\n\rDistance\tStarting Angle\tEnding Angle\tCount\n\r";

            for (i = 0; i < strlen(test); i++) {
                uart_sendChar(test[i]);
            }



            for (i = 0; i < 10; i++) {
                if (PINGDistances[i] == 0) {
                    break;
                }
                char testData[20];
                sprintf(testData, "%d\t\t%d\t\t%d\t\t%d\n\r", PINGDistances[i], startingAngles[i], endingAngles[i], counts[i]);
                int j;
                for (j = 0; j < strlen(testData); j++) {
                    uart_sendChar(testData[j]);
                }
            }


            for (i = 0; i < sizeof(linearWidth) / sizeof(linearWidth[0]); i++) {
                if (counts[i] == 0) {
                    break;
                }

                if (linearWidth[i] == 0) {
                    linearWidth[i] = PINGDistances[i] * tan(counts[i] * (3.14/180));
                }
            }

            int smallestWidth = 1000;
            int smallestWidthIndex = 0;
            for (i = 0; i < 10; i++) {
                if (linearWidth[i] == 0) {
                    break;
                }

                if (linearWidth[i] < smallestWidth) { // find the object with the smallest count (least linear width)
                    smallestWidth = linearWidth[i];
                    smallestWidthIndex = i;
                }
            }

            char widths[50] = "\nThe linear widths are: \n\r";
            for (i = 0; i < strlen(widths); i++) {
                uart_sendChar(widths[i]);
            }

            for (i = 0; i < 10; i++) {
                if (counts[i] == 0) {
                    break;
                }

                else {
                    char widthVals[30];
                    sprintf(widthVals, "Angle: %d, Width: %d\n\r", centerAngles[i], linearWidth[i]);
                    int j;
                    for (j = 0; j < strlen(widthVals); j++) {
                        uart_sendChar(widthVals[j]);
                    }
                }
            }

            int angleOfSmallest = centerAngles[smallestWidthIndex]; // smallest angle is angle at smallest count
            char result[50];
            sprintf(result, "\nThe smallest object is at %d degrees\n\r\n", angleOfSmallest);

            for (i = 0; i < strlen(result); i++) {
                uart_sendChar(result[i]);
            }

            servo_move(centerAngles[smallestWidthIndex]);
            timer_waitMillis(1000);

            if (centerAngles[smallestWidthIndex] >= 90) { // if the object is at an angle past or equal to 90
                oi_t *sensor_data = oi_alloc();
                oi_init(sensor_data);

                int inBetween = 180 - centerAngles[smallestWidthIndex];
                int final = 90 - inBetween - 15;

                if (final < 0) {
                    final = 0;
                }

                lcd_printf("%d degrees", final);

                char turnInstr[50];
                sprintf(turnInstr, "PRESS a TO TURN TO THE OBJECT\n\r\n");

                for (i = 0; i < strlen(turnInstr); i++) {
                    uart_sendChar(turnInstr[i]);
                }

                while (letter != 'a') {
                    letter = uart_receive();
                }

                turn_counterClockwise(sensor_data, final);
                timer_waitMillis(1000);

                sprintf(turnInstr, "PRESS w UNTIL YOU REACH THE OBJECT (5 cm steps)\n\r\n");

                letter = uart_receive();

                while (letter == 'w') {
                    move_forward(sensor_data, 40);
                    letter = uart_receive();
                }

                oi_free(sensor_data);
            } else if (centerAngles[smallestWidthIndex] < 90) { // if the object is at an angle before 90
                oi_t *sensor_data = oi_alloc();
                oi_init(sensor_data);

                int final = 90 - centerAngles[smallestWidthIndex] - 15;

                if (final < 0) {
                    final = 0;
                }

                lcd_printf("%d degrees", final);

                char turnInstr[50];
                sprintf(turnInstr, "PRESS d TO TURN TO THE OBJECT\n\r\n");

                for (i = 0; i < strlen(turnInstr); i++) {
                    uart_sendChar(turnInstr[i]);
                }

                letter = uart_receive();

                while (letter != 'd') {
                    letter = uart_receive();
                }


                turn_clockwise(sensor_data, final);
                timer_waitMillis(1000);

                sprintf(turnInstr, "PRESS w UNTIL YOU REACH THE OBJECT (5 cm steps)\n\r\n");

                letter = uart_receive();

                while (letter == 'w') {
                    move_forward(sensor_data, 40);
                    letter = uart_receive();
                }

                oi_free(sensor_data);
            }

            letter = uart_receive();
        }
    }
}
