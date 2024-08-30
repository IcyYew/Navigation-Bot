#include "stdio.h"
#include "math.h"
#include "time.h"
#include "stdlib.h"
#include "uart.h"
#include "servo.h"
//#include "cyBot_uart.h"
#include "open_interface.h"
#include "movement.h"
#include "Timer.h"
#include "lcd.h"
#include "adc.h"


typedef struct {
    float distance;
    int firstDegree;
    int lastDegree;
}object_t;



void main(void) {
    adc_init();
    timer_init();
    lcd_init();
    uart_init();
    servo_init();
    object_t object[20] = {0};
    int i = 0;
    int k = 0;
    int j = 0;
    float distance[180] = {0};
    float nextDist = 0.0;
    char line[30];
    char line2[100];
    char line3[100];
    float IRcalc = 0.0;
    float tempDist = 0.0;
    short degrees = 0;
    char objectNum = 0;
    float adcValue = 0.0;

    sprintf(line, "Degrees           Distance (cm)\n\r");
    for (i = 0; i < strlen(line) + 1; i++) {
      uart_sendChar(line[i]);
    }
    for (i = 0; i < 180; i++) {
	// do two scans at each degree
        servo_move(i);
        adcValue = adc_read();
        IRcalc = (float)(88519) * pow((float)adcValue, -1.19);
        distance[i] += IRcalc;
        adcValue = adc_read();
        //IRcalc = (float)(2 * pow(10, 8)) * pow((float)scan.IR_raw_val, -2.176); // for bot #3
        IRcalc = (float)(88519) * pow((float)adcValue, -1.19); // bot 5
        distance[i] += IRcalc;
        distance[i] /= 2;
        sprintf(line2, "%d                 %.2f\n\r", i, distance[i]);
        for (j = 0; j < strlen(line2) + 1; j++) {
            uart_sendChar(line2[j]);
        }
    }
    for (i = 0; i < 180; i++) {
        if (i != 179) { // handing to ensure overflow doesn't occur
            nextDist = distance[i + 1];
        }
        else {
            nextDist = distance[i];
        }
        // if two recurring points found with tolerance enter if statement
        if (fabs(nextDist - distance[i]) <= 10){
            tempDist = distance[i];
            // first degree of object
            object[k].firstDegree = i;
            object[k].distance = distance[i];
            // while loop to iterate over whole object, bounded with iterator maximum
            while (i < 179 && fabs(nextDist - tempDist) <= 25) {
                i++; // update i val to keep up with outer loop
                nextDist = distance[i + 1];
            }
            object[k].lastDegree = i; // last degree of object
            int width = object[k].lastDegree - object[k].firstDegree; //width of detected object
            // checking if object is actually big enough to be an object, if it is update k, if not k remains and overwrites old data
            if (width > 5 && object[k].distance < 150) {
                object[k].distance = 0;
                //settings mid object distance
                object[k].distance = distance[object[k].firstDegree + (width / 2)];
                k++;
            }
        }
    }
    IRcalc = 0;
    adcValue = 0;
    int numObjects = k;
    for (i = 0; i < numObjects; i++) {
        sprintf(line3, "Object: %d  Distance: %.2f   Width: %d  Start at: %d\n\r", i + 1, object[i].distance, object[i].lastDegree - object[i].firstDegree + 1, object[i].firstDegree);
        for(j = 0; j < strlen(line3) + 1; j++) {
            uart_sendChar(line3[j]);
        }
    }
    // Find smallest width
    int smallestWidth = object[0].lastDegree - object[0].firstDegree;
    int objectAngle = (object[0].lastDegree + object[0].firstDegree) / 2;
    for(i = 0; i < numObjects; i++) {
        if (smallestWidth > object[i].lastDegree - object[i].firstDegree) {
            smallestWidth = object[i].lastDegree - object[i].firstDegree;
            objectAngle = object[i].firstDegree + ((object[i].lastDegree - object[i].firstDegree) / 2);
            objectNum = i;
        }
    }

    //Point IRscanner to smallest object
    IRcalc = 0;
    adcValue = 0;
    timer_waitMillis(300);
    servo_move(objectAngle);

    //Send location to putty
    sprintf(line3, "Smallest Object\n\r");
    for(i = 0; i < strlen(line3) + 1; i++) {
        uart_sendChar(line3[i]);
    }
    sprintf(line3, "Angle: %d       Distance: %.2f\n\r", objectAngle, object[objectNum].distance);
    for(i = 0; i < strlen(line3) + 1; i++) {
        uart_sendChar(line3[i]);
    }
//  PART 2
     oi_t *sensor_data = oi_alloc();
     oi_init(sensor_data);
    //turn to correct angle
     if (objectAngle < 90) {
         degrees = 105 - objectAngle;
         turn_clockwise(sensor_data, degrees);
     }
     else {
         degrees = objectAngle - 110;
         turn_counterclockwise(sensor_data, degrees);
     }
    //Drive forward to object

     int forwardDistance = object[objectNum].distance * 10 + 10;
     move_forward(sensor_data, forwardDistance, 100);
     oi_free(sensor_data);
     oi_update(sensor_data);
}
