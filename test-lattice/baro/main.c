/**
 * @file    main.c
 * @brief   ARIES v3 BMP280 barometer entry point.
 *
 * The BMP280 is connected to SPI2. All hardware setup, sampling and UART
 * output live in barometer.c.
 */

#include "barometer.h"

int main(void)
{
    baro_run();
    return 0;
}
