#include "stdio.h"
#include "math.h"
#include "time.h"
#include "stdlib.h"
#include "uart.h"
#include "servo.h"
#include "open_interface.h"
#include "movement.h"
#include "Timer.h"
#include "lcd.h"
#include "adc.h"
#include "ping.h"
#include "button.h"
#include "success.h"
#include "LED.h"

// stores if we can cross lines (1 if we found the object)
int crossLines;
extern volatile unsigned char lineCrossed;
extern volatile unsigned char bumpedLastLeft;
extern volatile unsigned char bumpedLastRight;

// struct used to store the pingDistance and irDistance at each angle
typedef struct {
    float pingDistance;
    int irDistance;
}distances;

// struct used to store the firstDegree, lastDegree, midpoint degree, angular width, and linear width
typedef struct {
    unsigned firstDegree;
    unsigned lastDegree;
    unsigned mid;
    unsigned count;
    float linearWidth;
}object;

// struct used to store different parts of a gap, including linear width, angular width, distance, starting angle, ending angle, and midpoint
typedef struct {
    unsigned gapWidth;
    unsigned gapCount;
    unsigned gapDist;
    unsigned gapStart;
    unsigned gapEnd;
    unsigned gapMid;
}gap;



void main(void) {
    // initializations
    adc_init();
    ping_init();
    timer_init();
    lcd_init();
    uart_init();
    servo_init();
    botStatus_init();
    bumpedLastLeft = 0;
    bumpedLastRight = 0;
    lineCrossed = 0;

    // variable to check if we previously went halfway to an object (to ensure we are actually headed towards a small object instead of a large one)
    char previouslyHalfway = 0;

    // initialize cross lines with 0 until we reach the small object
    crossLines = 0;

    // point dead ahead since it was grinding the motor before at initialization time
    servo_move(90);


    // kill
    oi_t *sen_data = oi_alloc();
    oi_init(sen_data);

    move_forward(sen_data, 0);
    oi_free(sen_data);

    // iterators initialization
    int i = 0, j = 0, k = 0;

    // wait until we send an h through PuTTY to actually start our code
    char letter = uart_receive();

    while (letter != 'h') {
        letter = uart_receive();
    }

    // move into the zone
    oi_t *s_data = oi_alloc();
    oi_init(s_data);

    move_forward(s_data, 400);
    oi_free(s_data);

    // main scan and movement loop

    while (1) {
        // variable initializations (names are pretty self explanatory)
        distances dists[90] = {0};
        object obj[10] = {0};
        char line[50];
        char currentObject = 0;
        char count = 0;
        char smallestIndex = 0;
        char smallestWidth = 100;
        char noGaps = 1;
        botStatus_update(1);

        // for loop to instantiate all object parameters to 0
        for (i = 0; i < 10; i++) {
            obj[i].firstDegree = 0;
            obj[i].lastDegree = 0;
            obj[i].count = 0;
            obj[i].linearWidth = 0;
        }

        // move servo to 0 and wait a second before we start our 180 degree scan
        servo_move(0);
        timer_waitMillis(500);

        // main scan loop
        for (i = 0; i <= 180; i += 2) {
            servo_move(i);

            // store distance info
            dists[i / 2].pingDistance = ping_read();
            dists[i / 2].irDistance = adc_read();
            dists[i / 2].irDistance += adc_read();
            dists[i / 2].irDistance += adc_read();
            dists[i / 2].irDistance /= 3;

            // ensure IR values are a multiple of 50
            if ((dists[i / 2].irDistance % 50) != 0) {
                dists[i / 2].irDistance -= dists[i / 2].irDistance % 50;
            }

            // print angle info
            sprintf(line, "%d\t\t%d\t\t%.2f\n\r", i, dists[i / 2].irDistance, dists[i / 2].pingDistance);

            for (j = 0; j < strlen(line); j++) {
                uart_sendChar(line[j]);
            }
        }

        // object detection loop
        for (i = 1; i < 90; i++) {
            // checks if the current and previous angle's IR distance is > 900 (should get rid of 0 count objects, but sometimes doesn't work)
            if (dists[i].irDistance > 900 && dists[i - 1].irDistance > 900) {
                // checks if we're on the last angle, and ends any objects currently in scan
                if (i == 89) {
                    // if we're currently in an object, end it
                    if (count > 0) {
                        obj[currentObject].count = count;
                        obj[currentObject].lastDegree = i * 2;
                        currentObject++;
                    }
                }
                // if we weren't in an object yet, set currentObject first degree and increment count
                if (count == 0) {
                    obj[currentObject].firstDegree = i * 2;
                    count++;
                }
                // if we were in an object, increment count
                else if (count != 0) {
                    count++;
                }
            }
            // if we were in an object but are no longer in an object, set the currentObjects last degree, count, reset angular width count, and increment currentObject
            else if (dists[i].irDistance <= 900 && count != 0) {
                obj[currentObject].lastDegree = i * 2;
                obj[currentObject].count = count;

                count = 0;
                currentObject++;
            }
        }

        // for all objects, their midpoint is the firstDegree plus the count
        for (i = 0; i < currentObject; i++) {
            obj[i].mid = obj[i].count + obj[i].firstDegree;
        }
        if ((bumpedLastLeft || bumpedLastRight) && currentObject == 1) {
            oi_t *sensor_data = oi_alloc();
            oi_init(sensor_data);
            if (bumpedLastLeft) {
                turn_clockwise(sensor_data, 130);
                bumpedLastLeft = 0;
            }
            else if (bumpedLastRight) {
                turn_counterClockwise(sensor_data, 130);
                bumpedLastRight = 0;
            }

            oi_free(sensor_data);
            continue;
        }


        // if we aren't supposed to cross lines yet, run this code
        if (!crossLines) {

            // for all objects, determine the object with the smallest linear width, and store its width and index in the appropriate variables
            for (i = 0; i < currentObject; i++) {
                float bruh = dists[obj[i].mid / 2].pingDistance;
                obj[i].linearWidth = (bruh * tan((obj[i].count * 2) * (3.14/180)));
                // checks:
                // 1. if current smallestWidth is bigger than the width of the object at the current index
                // 2. if we are not at index 0
                // 3. if the linearWidth of the object at the current index is not 0
                // 4. if the linearWidth of the object at the current index is <= 4
                // 5. if the first angle is bigger than 10 degrees and the last angle is smaller than 170 degrees
                if (smallestWidth > obj[i].linearWidth && i != 0 && obj[i].linearWidth != 0 && obj[i].linearWidth <= 8 && (obj[i].firstDegree >= 10 && obj[i].lastDegree <= 170)) {
                    smallestWidth = obj[i].linearWidth;
                    smallestIndex = i;
                }
                // just sets the first width to the smallest if at index 0 and outside of edge case scenario
                else if (i == 0 && obj[i].linearWidth != 0 && (obj[i].firstDegree >= 10 && obj[i].lastDegree <= 170)) {
                    smallestWidth = obj[i].linearWidth;
                    smallestIndex = i;
                }
            }
        }

        // print out object info header
        sprintf(line, "\n\rFirst Angle\tLast Angle\tCount\tLinear Width\n\r");
        for (i = 0; i < strlen(line); i++) {
            uart_sendChar(line[i]);
        }

        // print out object info
        for (i = 0; i < currentObject; i++) {
            sprintf(line, "%d\t\t%d\t\t%d\t%.2f\n\r", obj[i].firstDegree, obj[i].lastDegree, obj[i].count, obj[i].linearWidth);

            for (j = 0; j < strlen(line); j++) {
                uart_sendChar(line[j]);
            }
        }

        // print smallest object info
        sprintf(line, "\n\rSmallest Object at angle: %d Width: %.2f\n\r", obj[smallestIndex].mid, obj[smallestIndex].linearWidth);

        for (i = 0; i < strlen(line); i++) {
            uart_sendChar(line[i]);
        }
        //end of scan code


        // start of movement code
        // if we aren't supposed to cross lines, run this code
        if (!crossLines) {
            // checks:
            // 1. if the first degree of the smallest object is greater than 10 and less than 170
            // 2. if the object's ping distance is greater than 10 and it's linear width is equal to 2, 3, or 4. Only accepts 4 if we previously went halfway
            // 3. if the object's ping distance is less than 10 and the count is less than or equal to 20 (no longer using linear width here, since it was very inaccurate)
            if ((obj[smallestIndex].firstDegree > 10 && obj[smallestIndex].lastDegree < 170) && ((dists[obj[smallestIndex].mid / 2].pingDistance > 10 && (obj[smallestIndex].linearWidth <= 8 || (obj[smallestIndex].linearWidth == 8 && previouslyHalfway == 1))) ||
                    (dists[obj[smallestIndex].mid / 2].pingDistance < 10 && obj[smallestIndex].count <= 20))) {
                servo_move(obj[smallestIndex].mid);

                // if we are right up on the object and its count is less than 20 and it previously went halfway, update crossLines, update LED to red while defusing and green when done, play sound, and turn all the way around
                if (dists[obj[smallestIndex].mid / 2].pingDistance < 12 && obj[smallestIndex].count <= 20 && previouslyHalfway) {
                    crossLines = 1;
                    botStatus_update(0);
                    success();
                    botStatus_update(2);
                    oi_t *sensor_data = oi_alloc();
                    oi_init(sensor_data);

                    turn_clockwise(sensor_data, 180);
                    oi_free(sensor_data);
                    continue;
                }

                // if the object's midpoint is greater than or equal to 90, turn counter clockwise towards it and either go halfway if we haven't already, or go all the way if we previously went halfway
                if (obj[smallestIndex].mid >= 90) {
                    oi_t *sensor_data = oi_alloc();
                    oi_init(sensor_data);

                    int final = 90 - (180 - obj[smallestIndex].mid) - 15;

                    if (final < 0) {
                        final = 0;
                    }

                    turn_counterClockwise(sensor_data, final);
                    timer_waitMillis(1000);

                    int moveForward = (int)(dists[obj[smallestIndex].mid / 2].pingDistance) * 10 - 125;

                    if (previouslyHalfway) {
                        move_forward_with_avoid(sensor_data, moveForward + 10);
                        previouslyHalfway = 0;
                    } else {
                        previouslyHalfway = 1;
                        moveForward /= 2;
                        move_forward_with_avoid(sensor_data, moveForward);
                    }

                    oi_free(sensor_data);
                }
                // if the object's midpoint is less than 90, turn clockwise towards it and either go halfway if we haven't already, or go all the way if we previously went halfway
                else if (obj[smallestIndex].mid < 90 && currentObject != 0) {
                    oi_t *sensor_data = oi_alloc();
                    oi_init(sensor_data);

                    int final = 90 - obj[smallestIndex].mid - 15;

                    if (final < 0) {
                        final = 0;
                    }

                    turn_clockwise(sensor_data, final);
                    timer_waitMillis(1000);

                    int moveForward = (int)(dists[obj[smallestIndex].mid / 2].pingDistance) * 10 - 125;

                    if (previouslyHalfway) {
                        move_forward_with_avoid(sensor_data, moveForward);
                        previouslyHalfway = 0;
                    } else {
                        previouslyHalfway = 1;
                        moveForward /= 2;
                        move_forward_with_avoid(sensor_data, moveForward);
                    }

                    oi_free(sensor_data);
                }
                // don't know if this is ever reachable, but if there is an object at angle 0, turn away from it
                else if (obj[0].firstDegree == 2) {
                    oi_t *sensor_data = oi_alloc();
                    oi_init(sensor_data);

                    turn_clockwise(sensor_data, 90);
                    timer_waitMillis(500);
                    oi_free(sensor_data);
                    continue;
                }
                // don't know if this is possible either, but if there is an object at angle 180, turn away from it
                else if (obj[currentObject-1].lastDegree == 178) {
                    oi_t *sensor_data = oi_alloc();
                    oi_init(sensor_data);

                    turn_counterClockwise(sensor_data, 90);
                    timer_waitMillis(500);

                    oi_free(sensor_data);
                    continue;
              }
                else {
                    oi_t *sensor_data = oi_alloc();
                    oi_init(sensor_data);

                    move_forward_with_avoid(sensor_data, 200);
                    oi_free(sensor_data);
                    continue;
                }
            }
            // if we didn't detect a small object, run our gap finding code
            else {
                // reset previously halfway
                if (previouslyHalfway == 1) {
                    previouslyHalfway = 0;
                }

                // as long as we saw an object, run this code
                if (currentObject > 0) {
                    // initialize gap specific variables
                    gap gaps[5] = {0};
                    int gapCount = 0;
                    char frontEmpty = 1;
                    int largestGap = 0;
                    int largestGapIndex = 100;

                    // if there is an object between 60 degrees and 120 degrees, we count the front as not safe to go through.
                    // THIS MAY NEED TO BE CHANGED!!!!!!!!
                    for (i = 0; i < currentObject; i++) {
                        //if (((dists[obj[i].mid / 2].pingDistance * cos(180 - obj[i].firstDegree)) > 20 && (obj[i].firstDegree > 90)) || ((dists[obj[i].mid / 2] * cos(obj[i].lastDegree)) > 20 && obj[i].lastDegree < 90)) {
                        if ((obj[i].lastDegree < 120 && obj[i].lastDegree > 90) || (obj[i].firstDegree < 120 && obj[i].firstDegree > 90) || (obj[i].lastDegree < 90 && obj[i].lastDegree > 60) || (obj[i].firstDegree < 90 && obj[i].lastDegree > 60)) {
                            frontEmpty = 0;
                        }
                    }

                    // for all the objects
                    for (i = 0; i < currentObject + 1; i++) {
                        // if the front is empty, go through it and break out of the loop
                        if (frontEmpty) {
                            noGaps = 0;
                            oi_t *sensor_data = oi_alloc();
                            oi_init(sensor_data);

                            lcd_init();

                            lcd_printf("Front Empty");

                            move_forward_with_avoid(sensor_data, 100);
                            oi_free(sensor_data);
                            break;
                        }
                        // if at first index, initialize first gap (gap at start)
                        else if (i == 0) {
                            gaps[0].gapStart = 0; // gap starts at 0
                            gaps[0].gapEnd = obj[i].firstDegree; // gap ends at wherever the first object starts
                            gaps[0].gapMid = (gaps[0].gapEnd - gaps[0].gapStart) / 2; // midpoint is between start and end
                            gaps[0].gapDist = dists[obj[i].mid / 2].pingDistance; // distance is calculated using the distance of the nearest object
                            gaps[0].gapCount = obj[i].firstDegree / 2; // count is angular width
                            gaps[0].gapWidth = (gaps[0].gapDist * tan(gaps[i].gapCount * (3.14/180))); // gapWidth is linear width
                            gapCount++; // increment gap counter

                            // if there was only one object, initialize second gap to be our last (gap at end)
                            if (currentObject == 1) {
                                gaps[1].gapStart = obj[0].lastDegree; // gap starts at end of object
                                gaps[1].gapEnd = 180; // gap ends at 180
                                gaps[1].gapMid = gaps[1].gapStart + ((gaps[1].gapEnd - gaps[1].gapStart) / 2); // middle between start and end
                                gaps[1].gapDist = dists[obj[0].mid / 2].pingDistance; // distance of nearest object
                                gaps[1].gapCount = (180 - obj[0].lastDegree) / 2; // angular width of gap
                                gaps[1].gapWidth = (gaps[1].gapDist * tan(gaps[i].gapCount * (3.14/180))); // linear width of gap
                                gapCount++; // increment gap counter
                                break;
                            }
                        }
                        // if we are not at first object and less than or equal to however many objects there were
                        else if (i <= currentObject) {
                            gaps[i].gapStart = obj[i-1].lastDegree; // start is the end of the last object
                            gaps[i].gapEnd = obj[i].firstDegree; // end if the beginning of the current object
                            gaps[i].gapMid = gaps[i].gapStart + ((gaps[i].gapEnd - gaps[i].gapStart) / 2); // midpoint between start and end
                            gaps[i].gapCount = (obj[i].firstDegree - obj[i-1].lastDegree) / 2; // angular width
                            gaps[i].gapDist = (dists[obj[i].mid / 2].pingDistance + dists[obj[i-1].mid / 2].pingDistance) / 2; // distance of nearest object
                            gaps[i].gapWidth = (gaps[i].gapDist * tan(gaps[i].gapCount * (3.14/180))); // linear width of gap
                            gapCount++; // increment gap counter
                        }
                        // if we are past the last object
                        else {
                            gaps[i].gapStart = obj[i-1].lastDegree; // set start to the last degree of the previous object
                            gaps[i].gapEnd = 180; // set end to 180
                            gaps[i].gapMid = gaps[i].gapStart + ((gaps[i].gapEnd - gaps[i].gapStart) / 2); // midpoint between start and end
                            gaps[i].gapCount = (180 - obj[i-1].lastDegree) / 2; // angular width
                            gaps[i].gapDist = dists[obj[i-1].mid / 2].pingDistance; // distance of nearest object
                            gaps[i].gapWidth = (gaps[i].gapDist * tan(gaps[i].gapCount * (3.14/180))); // linear width of gap
                            gapCount++; // increment gap counter
                        }
                    }

                    // go through all gaps to see if there's one that's safe to go through. Also keeps track of the largest gap and stores that and its index
                    for (i = 0; i < gapCount; i++) {
                        if (gaps[i].gapWidth > largestGap && gaps[i].gapWidth >= 10) {
                            largestGap = gaps[i].gapWidth;
                            largestGapIndex = i;
                            noGaps = 0;
                        }
                    }

                    // turn stores amount to turn and direction stores which way we are turning
                    int turn = 0;
                    int direction = 100;

                    // as long as we found a gap
                    if (noGaps == 0) {
                        if (gaps[largestGapIndex].gapMid <= 90) {
                            turn = 85 - gaps[largestGapIndex].gapMid;
                            direction = 1; // turn right
                        } else if (gaps[largestGapIndex].gapMid > 90) {
                            turn = 90 - (180 - gaps[largestGapIndex].gapMid);
                            direction = 2; // turn left
                        }
                    }
                    // if we didn't find a gap, turn 90 in which ever way is safer
                    else if (noGaps == 1) {
                        oi_t *sensor_data = oi_alloc();
                        oi_init(sensor_data);
                        sprintf(line, "No gaps\n\r");
                        for (k = 0; k < strlen(line); k++) {
                            uart_sendChar(line[k]);
                        }
                        timer_waitMillis(500);
                        // if there is an object in the first 30 degrees, turn away from it
                        if (obj[0].firstDegree > 30) {
                          turn_clockwise(sensor_data, 80);
                        }
                        // if there is an object in the last 30 degrees, turn away from it
                        else if (obj[currentObject - 1].lastDegree < 150) {
                          turn_counterClockwise(sensor_data, 80);
                        }
                        // otherwise, just turn right 45 degrees
                        else {
                            turn_clockwise(sensor_data, 45);
                        }
                        oi_free(sensor_data);
                        continue;
                    }

                    // as long as we are supposed to turn, run the turn code (implemented since just having it turn when calculating kept breaking things, or maybe the bot just died)
                    if (turn != 0) {
                        oi_t *sensor_data = oi_alloc();
                        oi_init(sensor_data);

                        // turn right if direction is 1
                        if (direction == 1) {
                            turn_clockwise(sensor_data, turn);
                        }
                        // turn left if direction is 2
                        else if (direction == 2) {
                            turn_counterClockwise(sensor_data, turn);
                        }

                        timer_waitMillis(500);
                        oi_free(sensor_data);
                        continue;
                    }
                }
                // if we didn't see any objects, move forward 20 cm
                else if (currentObject == 0) {
                    oi_t *sensor_data = oi_alloc();
                    oi_init(sensor_data);
                    move_forward_with_avoid(sensor_data, 200);

                    lcd_init();

                    lcd_printf("No objects\nMoving %d", 20);

                    oi_free(sensor_data);
                    continue;
                }
            }
        }
        // if we are supposed to cross lines, that means we found our goal, so we no longer need to search for the small objects, we just need to find gaps and go through them until we reach the edge
        else {
            // as long as we saw an object, run this code
            if(lineCrossed) {
                botStatus_update(2);
                break;
            }
            if (currentObject > 0) {
                // initialize gap specific variables
                gap gaps[5] = {0};
                int gapCount = 0;
                char frontEmpty = 1;
                int largestGap = 0;
                int largestGapIndex = 100;

                // if there is an object between 60 degrees and 120 degrees, we count the front as not safe to go through.
                // THIS MAY NEED TO BE CHANGED!!!!!!!!
                for (i = 0; i < currentObject; i++) {
                    if ((obj[i].lastDegree < 120 && obj[i].lastDegree > 90) || (obj[i].firstDegree < 120 && obj[i].firstDegree > 90) || (obj[i].lastDegree < 90 && obj[i].lastDegree > 60) || (obj[i].firstDegree < 90 && obj[i].lastDegree > 60)) {
                        frontEmpty = 0;
                    }
                }

                // for all the objects
                for (i = 0; i < currentObject + 1; i++) {
                    // if the front is empty, go through it and break out of the loop
                    if (frontEmpty) {
                        noGaps = 0;
                        oi_t *sensor_data = oi_alloc();
                        oi_init(sensor_data);

                        lcd_init();

                        lcd_printf("Front Empty");

                        move_forward_with_avoid(sensor_data, 250);
                        oi_free(sensor_data);
                        break;
                    }
                    // if at first index, initialize first gap (gap at start)
                    else if (i == 0) {
                        gaps[0].gapStart = 0; // gap starts at 0
                        gaps[0].gapEnd = obj[i].firstDegree; // gap ends at wherever the first object starts
                        gaps[0].gapMid = (gaps[0].gapEnd - gaps[0].gapStart) / 2; // midpoint is between start and end
                        gaps[0].gapDist = dists[obj[i].mid / 2].pingDistance; // distance is calculated using the distance of the nearest object
                        gaps[0].gapCount = obj[i].firstDegree / 2; // count is angular width
                        gaps[0].gapWidth = ceilf((gaps[0].gapDist * tan(gaps[0].gapCount * (3.14/180)))); // gapWidth is linear width
                        gapCount++; // increment gap counter

                        // if there was only one object, initialize second gap to be our last (gap at end)
                        if (currentObject == 1) {
                            gaps[1].gapStart = obj[0].lastDegree; // gap starts at end of object
                            gaps[1].gapEnd = 180; // gap ends at 180
                            gaps[1].gapMid = gaps[1].gapStart + ((gaps[1].gapEnd - gaps[1].gapStart) / 2); // middle between start and end
                            gaps[1].gapDist = dists[obj[0].mid / 2].pingDistance; // distance of nearest object
                            gaps[1].gapCount = (180 - obj[0].lastDegree) / 2; // angular width of gap
                            gaps[1].gapWidth = ceilf((gaps[1].gapDist * tan(gaps[1].gapCount * (3.14/180)))); // linear width of gap
                            gapCount++; // increment gap counter
                            break;
                        }
                    }
                    // if we are not at first object and less than or equal to however many objects there were
                    else if (i <= currentObject) {
                        gaps[i].gapStart = obj[i-1].lastDegree; // start is the end of the last object
                        gaps[i].gapEnd = obj[i].firstDegree; // end if the beginning of the current object
                        gaps[i].gapMid = gaps[i].gapStart + ((gaps[i].gapEnd - gaps[i].gapStart) / 2); // midpoint between start and end
                        gaps[i].gapCount = (obj[i].firstDegree - obj[i-1].lastDegree) / 2; // angular width
                        gaps[i].gapDist = (dists[obj[i].mid / 2].pingDistance + dists[obj[i-1].mid / 2].pingDistance) / 2; // distance of nearest object
                        gaps[i].gapWidth = ceilf((gaps[i].gapDist * tan(gaps[i].gapCount * (3.14/180)))); // linear width of gap
                        gapCount++; // increment gap counter
                    }
                    // if we are past the last object
                    else {
                        gaps[i].gapStart = obj[i-1].lastDegree; // set start to the last degree of the previous object
                        gaps[i].gapEnd = 180; // set end to 180
                        gaps[i].gapMid = gaps[i].gapStart + ((gaps[i].gapEnd - gaps[i].gapStart) / 2); // midpoint between start and end
                        gaps[i].gapCount = (180 - obj[i-1].lastDegree) / 2; // angular width
                        gaps[i].gapDist = dists[obj[i-1].mid / 2].pingDistance; // distance of nearest object
                        gaps[i].gapWidth = ceilf((gaps[i].gapDist * tan(gaps[i].gapCount * (3.14/180)))); // linear width of gap
                        gapCount++; // increment gap counter
                    }
                }

                // go through all gaps to see if there's one that's safe to go through. Also keeps track of the largest gap and stores that and its index
                for (i = 0; i < gapCount; i++) {
                    if (gaps[i].gapWidth > largestGap && gaps[i].gapWidth >= 10) {
                        largestGap = gaps[i].gapWidth;
                        largestGapIndex = i;
                        noGaps = 0;
                    }
                }

                // turn stores amount to turn and direction stores which way we are turning
                int turn = 0;
                int direction = 100;

                // as long as we found a gap
                if (noGaps == 0) {
                    if (gaps[largestGapIndex].gapMid <= 90) {
                        turn = 90 - gaps[largestGapIndex].gapMid;
                        direction = 1; // turn right
                    } else if (gaps[largestGapIndex].gapMid > 90) {
                        turn = 90 - (180 - gaps[largestGapIndex].gapMid);
                        direction = 2; // turn left
                    }
                }
                // if we didn't find a gap, turn 90 in which ever way is safer
                else if (noGaps == 1) {
                    oi_t *sensor_data = oi_alloc();
                    oi_init(sensor_data);
                    sprintf(line, "No gaps\n\r");
                    for (k = 0; k < strlen(line); k++) {
                        uart_sendChar(line[k]);
                    }
                    timer_waitMillis(500);
                    // if there is an object in the first 30 degrees, turn away from it
                    if (obj[0].firstDegree > 30) {
                        turn_clockwise(sensor_data, 80);
                    }
                    // if there is an object in the last 30 degrees, turn away from it
                    else if (obj[currentObject - 1].lastDegree < 150) {
                        turn_counterClockwise(sensor_data, 80);
                    }
                    // otherwise, just turn right 45 degrees
                    else {
                        turn_clockwise(sensor_data, 45);
                    }
                    oi_free(sensor_data);
                    continue;
                }

                // as long as we are supposed to turn, run the turn code (implemented since just having it turn when calculating kept breaking things, or maybe the bot just died)
                if (turn != 0) {
                    oi_t *sensor_data = oi_alloc();
                    oi_init(sensor_data);

                    // turn right if direction is 1
                    if (direction == 1) {
                        turn_clockwise(sensor_data, turn);
                    }
                    // turn left if direction is 2
                    else if (direction == 2) {
                        turn_counterClockwise(sensor_data, turn);
                    }

                    timer_waitMillis(500);
                    oi_free(sensor_data);
                    continue;
                }
            }
            // if we didn't see any objects, move forward 20 cm
            else if (currentObject == 0) {
                oi_t *sensor_data = oi_alloc();
                oi_init(sensor_data);
                move_forward_with_avoid(sensor_data, 200);

                lcd_init();

                lcd_printf("No objects\nMoving %d", 20);

                oi_free(sensor_data);
                continue;
            }
        }
    }
}
