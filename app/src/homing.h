#ifndef HOMING_H
#define HOMING_H

#include "motors.h"

int axis_limit_pressed(const struct axis *axis);
int axis_home(const struct axis *axis, int dir, uint32_t step_interval_us);

#endif
