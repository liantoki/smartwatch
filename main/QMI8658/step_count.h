#ifndef _STEP_COUNT_H
#define _STEP_COUNT_H

#include "QMI8658.h"

void step_counter_init(void);

int step_counter_update(IMUdata *current_accel);


#endif