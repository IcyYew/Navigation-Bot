/*
 * movement.h
 *
 *  Created on: Sep 6, 2023
 *      Author: ntegeler
 */

#include "open_interface.h"
#ifndef MOVEMENT_H_
#define MOVEMENT_H_

void move_forward(oi_t * sensor, int centimeters);

void move_forward_without_stop(oi_t *sensor, int centimeters);

void move_backwards(oi_t *sensor, int centimeters);

void move_forward_with_avoid(oi_t *sensor, int centimeters);

void turn_clockwise(oi_t *sensor, int degrees);

void turn_counterClockwise(oi_t *sensor, int degrees);

void avoid_object_left(oi_t *sensor);

void avoid_object_right(oi_t *sensor);

#endif /* MOVEMENT_H_ */
