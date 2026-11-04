/**
 * barometer.h - Minimal BMP280 SPI driver
 *
 * Reads temperature and pressure and calculates altitude in meters.
 *
 * The driver itself is platform independent: it talks to the sensor through
 * the four function pointers in bmp280_port_t. This project binds that port to
 * the ARIES v3 SPI2 controller (see bmp280_spi2_init() below).
 *
 * Based on:
 * ebrezadev/BMP280-Barometric-Pressure-and-Temperature-Sensor-C-Driver
 * MIT License, Copyright (c) 2025 Reza G. Ebrahimi
 */

#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Return codes
 * -------------------------------------------------------------------------- */
typedef enum
{
    BMP280_OK = 0,
    BMP280_ERR_NULL_HANDLE,
    BMP280_ERR_NULL_PORT,
    BMP280_ERR_NULL_ARG,
    BMP280_ERR_SENSOR_ID,
    BMP280_ERR_TIMEOUT,
    BMP280_ERR_CALIB,
    BMP280_ERR_DATA
} bmp280_error_t;

/* --------------------------------------------------------------------------
 * Register addresses
 * -------------------------------------------------------------------------- */
#define BMP280_REG_CALIB        0x88
#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_TEMP_MSB     0xFA

/* --------------------------------------------------------------------------
 * Register values
 * -------------------------------------------------------------------------- */
#define BMP280_CHIP_ID          0x58
#define BMP280_RESET_VALUE      0xB6
#define BMP280_STATUS_MEASURING 0x08

/* SPI address byte: bit 7 = 1 for read, 0 for write */
#define BMP280_SPI_READ         0x80
#define BMP280_SPI_WRITE_MASK   0x7F

/* --------------------------------------------------------------------------
 * User configuration (override before including this header if desired)
 * -------------------------------------------------------------------------- */
#ifndef BMP280_OSRS_T
#define BMP280_OSRS_T           0x02    /* temperature oversampling x2  */
#endif

#ifndef BMP280_OSRS_P
#define BMP280_OSRS_P           0x05    /* pressure oversampling x16    */
#endif

#ifndef BMP280_IIR_FILTER
#define BMP280_IIR_FILTER       0x04    /* IIR filter x16               */
#endif

#ifndef BMP280_SEA_LEVEL_PA
#define BMP280_SEA_LEVEL_PA     101325.0f
#endif

/* --------------------------------------------------------------------------
 * Timing
 * -------------------------------------------------------------------------- */
#define BMP280_RESET_DELAY_MS   10
#define BMP280_POLL_TIMEOUT_MS  100
#define BMP280_POLL_PERIOD_MS   1
#define BMP280_MEASURE_DELAY_MS 50

/* --------------------------------------------------------------------------
 * Calibration data (read once during init)
 * -------------------------------------------------------------------------- */
typedef struct
{
    uint16_t T1;
    int16_t  T2;
    int16_t  T3;
    uint16_t P1;
    int16_t  P2;
    int16_t  P3;
    int16_t  P4;
    int16_t  P5;
    int16_t  P6;
    int16_t  P7;
    int16_t  P8;
    int16_t  P9;
} bmp280_calib_t;

/* --------------------------------------------------------------------------
 * Platform hooks
 *
 * Implement these four functions for your MCU.
 * SPI must be mode 0 or mode 3, MSB first, max 10 MHz.
 * -------------------------------------------------------------------------- */
typedef struct
{
    void    (*cs_low)(void);            /* drive CSB low  */
    void    (*cs_high)(void);           /* drive CSB high */
    uint8_t (*transfer)(uint8_t tx);    /* transfer one byte, return received byte */
    void    (*delay_ms)(uint32_t ms);   /* millisecond delay */
} bmp280_port_t;

/* --------------------------------------------------------------------------
 * Driver instance
 * -------------------------------------------------------------------------- */
typedef struct
{
    bmp280_calib_t calib;
    int32_t        t_fine;
    int32_t        last_adc_p;
    int32_t        last_adc_t;
    int32_t        last_temp_centi;
    uint32_t       last_pressure_pa;
    bmp280_port_t  port;
} bmp280_t;

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */

/**
 * Reset the sensor, verify its chip ID, read calibration data and apply
 * the oversampling/filter settings from this header.
 */
bmp280_error_t bmp280_init(bmp280_t *dev);

/**
 * Perform one forced measurement.
 *
 * temperature_c and pressure_pa may be NULL if that value is not needed.
 * pressure_pa is returned in pascals.
 */
bmp280_error_t bmp280_read(bmp280_t *dev,
                           float *temperature_c,
                           float *pressure_pa);

/**
 * Perform one forced measurement and return altitude in meters above
 * BMP280_SEA_LEVEL_PA.
 */
bmp280_error_t bmp280_read_altitude(bmp280_t *dev,
                                    float *altitude_m);

/**
 * Convert pressure in pascals to altitude in meters.
 * Uses the same barometric formula as the original driver.
 */
float bmp280_pressure_to_altitude(float pressure_pa);

/* --------------------------------------------------------------------------
 * ARIES v3 SPI2 board binding
 *
 * The BMP280 is wired to the ARIES v3 SPI2 port:
 *
 *   BMP280 VIN  -> 3V3
 *   BMP280 GND  -> GND
 *   BMP280 SCK  -> SPI2CLK
 *   BMP280 SDI  -> SPI2MOSI
 *   BMP280 SDO  -> SPI2MISO
 *   BMP280 CSB  -> SPI2CSN (hardware chip select 0)
 *
 * SPI2 is configured for mode 3, MSB first, 8 bits per transfer, at
 * 100 MHz / 32 = 3.125 MHz (the BMP280 maximum is 10 MHz). CSB stays
 * asserted for the whole register access through the controller's CSAAT bit.
 * -------------------------------------------------------------------------- */
#define BMP280_SPI2_BUS     2U      /* ARIES v3 SPI2 controller        */
#define BMP280_SPI2_CS      0U      /* SPI2CSN, fixed peripheral CS0   */

/**
 * Configure SPI2, bind its hooks to dev->port and initialise the sensor.
 * Use this instead of bmp280_init() on the ARIES v3 board.
 */
bmp280_error_t bmp280_spi2_init(bmp280_t *dev);

/**
 * Bring up UART0 and the BMP280 on SPI2, then print temperature, pressure
 * and altitude forever. This never returns.
 */
void baro_run(void);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */