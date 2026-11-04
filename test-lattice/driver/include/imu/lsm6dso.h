/**
 * @file    lsm6dso.h
 * @brief   Minimal, allocation-free LSM6DSO driver for a RISC-V flight controller.
 *
 * Only the registers required for accel/gyro configuration and a single
 * 12-byte burst read are exposed. FIFO, sensor hub, FSM, pedometer, tap,
 * OIS, interrupt routing and self-test are intentionally omitted.
 */

#ifndef LSM6DSO_H
#define LSM6DSO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Register map (main page, only what this driver uses)
 * ---------------------------------------------------------------------- */

#define LSM6DSO_REG_WHO_AM_I    0x0FU
#define LSM6DSO_REG_CTRL1_XL    0x10U
#define LSM6DSO_REG_CTRL2_G     0x11U
#define LSM6DSO_REG_CTRL3_C     0x12U
#define LSM6DSO_REG_STATUS      0x1EU

#define LSM6DSO_REG_OUTX_L_G    0x22U
#define LSM6DSO_REG_OUTX_H_G    0x23U
#define LSM6DSO_REG_OUTY_L_G    0x24U
#define LSM6DSO_REG_OUTY_H_G    0x25U
#define LSM6DSO_REG_OUTZ_L_G    0x26U
#define LSM6DSO_REG_OUTZ_H_G    0x27U

#define LSM6DSO_REG_OUTX_L_A    0x28U
#define LSM6DSO_REG_OUTX_H_A    0x29U
#define LSM6DSO_REG_OUTY_L_A    0x2AU
#define LSM6DSO_REG_OUTY_H_A    0x2BU
#define LSM6DSO_REG_OUTZ_L_A    0x2CU
#define LSM6DSO_REG_OUTZ_H_A    0x2DU

/* -------------------------------------------------------------------------
 * Device constants and bit fields
 * ---------------------------------------------------------------------- */

#define LSM6DSO_WHO_AM_I_VALUE      0x6CU

#define LSM6DSO_SPI_READ_BIT        0x80U

#define LSM6DSO_STATUS_XLDA         0x01U   /* accel new data available */
#define LSM6DSO_STATUS_GDA          0x02U   /* gyro new data available  */

#define LSM6DSO_CTRL3_SW_RESET      0x01U
#define LSM6DSO_CTRL3_IF_INC        0x04U   /* auto-increment for bursts */
#define LSM6DSO_CTRL3_BDU           0x40U   /* block data update         */

/* -------------------------------------------------------------------------
 * Configuration: 1.666 kHz, +/-16 g, +/-2000 dps, high-performance mode.
 * High-performance mode is the reset default in CTRL6_C/CTRL7_G, and
 * CTRL8_XL.XL_FS_MODE resets to 0 so FS_XL=01 really selects +/-16 g.
 * ---------------------------------------------------------------------- */

#define LSM6DSO_ODR_1666HZ          0x08U
#define LSM6DSO_XL_FS_16G           0x01U
#define LSM6DSO_G_FS_2000DPS        0x03U

/* CTRL1_XL: FS_XL[1:0] is at bits 3:2, CTRL2_G: FS_G[1:0] at bits 3:2. */
#define LSM6DSO_CTRL1_XL_VALUE \
    ((uint8_t)((LSM6DSO_ODR_1666HZ << 4) | (LSM6DSO_XL_FS_16G << 2)))

#define LSM6DSO_CTRL2_G_VALUE \
    ((uint8_t)((LSM6DSO_ODR_1666HZ << 4) | (LSM6DSO_G_FS_2000DPS << 2)))

/* -------------------------------------------------------------------------
 * Sensitivity conversion macros.
 *
 * These values MUST match the full-scale settings above. Keep the unused
 * ranges here so changing the FS field and the matching macro is a one-line
 * edit.
 * ---------------------------------------------------------------------- */

/* Gyro sensitivity in dps/LSB. */
#define LSM6DSO_GYRO_SENS_125DPS    0.004375f
#define LSM6DSO_GYRO_SENS_250DPS    0.008750f
#define LSM6DSO_GYRO_SENS_500DPS    0.017500f
#define LSM6DSO_GYRO_SENS_1000DPS   0.035000f
#define LSM6DSO_GYRO_SENS_2000DPS   0.070000f

/* Accel sensitivity in g/LSB. */
#define LSM6DSO_ACCEL_SENS_2G       0.000061f
#define LSM6DSO_ACCEL_SENS_4G       0.000122f
#define LSM6DSO_ACCEL_SENS_8G       0.000244f
#define LSM6DSO_ACCEL_SENS_16G      0.000488f

/* Selected conversion factors. */
#define LSM6DSO_GYRO_DPS_PER_LSB    LSM6DSO_GYRO_SENS_2000DPS
#define LSM6DSO_ACCEL_G_PER_LSB     LSM6DSO_ACCEL_SENS_16G

/* Gyro XYZ followed by accel XYZ = 12 bytes starting at OUTX_L_G. */
#define LSM6DSO_DATA_BURST_LEN      12U

/* -------------------------------------------------------------------------
 * Public data type
 * ---------------------------------------------------------------------- */

typedef struct
{
    float gyro_x;       /* dps */
    float gyro_y;
    float gyro_z;
    float accel_x;      /* g */
    float accel_y;
    float accel_z;
} imu_data_t;

/* -------------------------------------------------------------------------
 * API
 *
 * imu_init()       0 on success, negative on SPI/identity error.
 * imu_read()       0 on success; fills *data with converted values.
 * imu_data_ready() 1 when both accel and gyro data are new, 0 otherwise,
 *                  negative on SPI error.
 * ---------------------------------------------------------------------- */

int imu_init(void);
int imu_read(imu_data_t *data);
int imu_data_ready(void);

/*
 * Read WHO_AM_I without configuring the device. Returns 0 and stores the byte
 * on success, negative if the SPI transaction failed.
 */
int imu_read_who_am_i(uint8_t *value);

/*
 * Initialize the ARIES v3 SPI1/UART hardware and run the IMU print loop
 * forever. This never returns.
 */
void imu_run(void);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSO_H */
