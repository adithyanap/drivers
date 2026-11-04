/**
 * @file    main.c
 * @brief   ARIES v3 LSM6DSO entry point.
 *
 * All hardware setup, sampling and UART output live in lsm6dso.c.
 */

#include "lsm6dso.h"

int main(void)
{
    imu_run();
    return 0;
}
