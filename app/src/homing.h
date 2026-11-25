#ifndef HOMING_H
#define HOMING_H

#include "motors.h"

int axis_limit_pressed(const struct axis *axis);
int home_axis(const struct axis *axis);
#endif
