/**
- bmp280.c - Minimal BMP280 SPI driver
 *
- Based on:
- ebrezadev/BMP280-Barometric-Pressure-and-Temperature-Sensor-C-Driver
- MIT License, Copyright (c) 2025 Reza G. Ebrahimi
 */

#include "barometer.h"
#include "stdlib.h"
#include "config.h"
#include "spi.h"

#include <math.h>
#include <stddef.h>

/* --------------------------------------------------------------------------
- Low-level SPI helpers
- -------------------------------------------------------------------------- */

static uint8_t bmp280_xfer(bmp280_t *dev, uint8_t tx)
{
    return dev->port.transfer(tx);
}

static void bmp280_read_regs(bmp280_t *dev,
                             uint8_t reg,
                             uint8_t *buf,
                             uint8_t len)
{
    uint8_t i;

    dev->port.cs_low();
    (void)bmp280_xfer(dev, (uint8_t)(reg | BMP280_SPI_READ));

    for (i = 0; i < len; i++)
    {
        buf[i] = bmp280_xfer(dev, 0xFF);
    }

    dev->port.cs_high();
}

static void bmp280_write_reg(bmp280_t *dev, uint8_t reg, uint8_t value)
{
    dev->port.cs_low();
    (void)bmp280_xfer(dev, (uint8_t)(reg & BMP280_SPI_WRITE_MASK));
    (void)bmp280_xfer(dev, value);
    dev->port.cs_high();
}

static uint16_t bmp280_le16u(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t bmp280_le16s(const uint8_t *p)
{
    return (int16_t)bmp280_le16u(p);
}

static void bmp280_parse_calib(bmp280_t *dev, const uint8_t *calib)
{
    dev->calib.T1 = bmp280_le16u(&calib[0]);
    dev->calib.T2 = bmp280_le16s(&calib[2]);
    dev->calib.T3 = bmp280_le16s(&calib[4]);

    dev->calib.P1 = bmp280_le16u(&calib[6]);
    dev->calib.P2 = bmp280_le16s(&calib[8]);
    dev->calib.P3 = bmp280_le16s(&calib[10]);
    dev->calib.P4 = bmp280_le16s(&calib[12]);
    dev->calib.P5 = bmp280_le16s(&calib[14]);
    dev->calib.P6 = bmp280_le16s(&calib[16]);
    dev->calib.P7 = bmp280_le16s(&calib[18]);
    dev->calib.P8 = bmp280_le16s(&calib[20]);
    dev->calib.P9 = bmp280_le16s(&calib[22]);
}

static int bmp280_calib_valid(const bmp280_t *dev)
{
    /*
- A real BMP280 always has non-zero trim coefficients.  All-zero bytes
- (bus returned 0x00) or all-ones (bus returned 0xFF) mean the NVM burst
- was not read correctly, and the compensation output would be 0.
     */
    if ((dev->calib.T1 == 0U) || (dev->calib.P1 == 0U))
    {
        return 0;
    }

    if ((dev->calib.T1 == 0xFFFFU) || (dev->calib.P1 == 0xFFFFU))
    {
        return 0;
    }

    /*
- A partly-read calibration block (T1 valid, the rest zero) would make
- the temperature compensation collapse to exactly 0.00 C.  Real parts
- always have non-zero T2/T3 trim values.
     */
    if ((dev->calib.T2 == 0) && (dev->calib.T3 == 0))
    {
        return 0;
    }

    return 1;
}

/* --------------------------------------------------------------------------
- Initialization
- -------------------------------------------------------------------------- */

bmp280_error_t bmp280_init(bmp280_t *dev)
{
    uint8_t id;
    uint8_t calib[24];
    uint8_t i;

    if (dev == NULL)
    {
        return BMP280_ERR_NULL_HANDLE;
    }

    if (dev->port.cs_low   == NULL ||
        dev->port.cs_high  == NULL ||
        dev->port.transfer == NULL ||
        dev->port.delay_ms == NULL)
    {
        return BMP280_ERR_NULL_PORT;
    }

    /* Make sure CS is idle high before the first transaction */
    dev->port.cs_high();
    dev->port.delay_ms(BMP280_RESET_DELAY_MS);

    /* Verify chip ID */
    bmp280_read_regs(dev, BMP280_REG_ID, &id, 1);
    if (id != BMP280_CHIP_ID)
    {
        return BMP280_ERR_SENSOR_ID;
    }

    /* Soft reset */
    bmp280_write_reg(dev, BMP280_REG_RESET, BMP280_RESET_VALUE);
    dev->port.delay_ms(BMP280_RESET_DELAY_MS);

    /*
- Read the 24 calibration bytes one at a time.  Some BMP280 modules /
- SPI adapters do not return a long auto-increment burst correctly; the
- first bytes come back but the rest read as 0x00, which makes every
- compensated result collapse to exactly zero.  Addressing each byte
- separately avoids relying on auto-increment at all.
     */
    for (i = 0U; i < 24U; ++i)
    {
        bmp280_read_regs(dev, (uint8_t)(BMP280_REG_CALIB + i), &calib[i], 1);
    }
    bmp280_parse_calib(dev, calib);

    /*
- The first read can come back empty if the reset has not fully
- completed or if CSB was not released between accesses.  Retry once
- after the sensor wake-up time before giving up.
     */
    if (!bmp280_calib_valid(dev))
    {
        dev->port.delay_ms(BMP280_RESET_DELAY_MS);

        for (i = 0U; i < 24U; ++i)
        {
            bmp280_read_regs(dev, (uint8_t)(BMP280_REG_CALIB + i),
                             &calib[i], 1);
        }
        bmp280_parse_calib(dev, calib);

        if (!bmp280_calib_valid(dev))
        {
            return BMP280_ERR_CALIB;
        }
    }

    /* IIR filter. Standby time is irrelevant in forced mode. */
    bmp280_write_reg(dev, BMP280_REG_CONFIG,
                     (uint8_t)(BMP280_IIR_FILTER << 2));

    /* Configure oversampling while staying in sleep mode */
    bmp280_write_reg(dev, BMP280_REG_CTRL_MEAS,
                     (uint8_t)((BMP280_OSRS_T << 5) |
                               (BMP280_OSRS_P << 2) |
                               0x00));

    return BMP280_OK;
}

/* --------------------------------------------------------------------------
- Measurement
- -------------------------------------------------------------------------- */

bmp280_error_t bmp280_read(bmp280_t *dev,
                           float *temperature_c,
                           float *pressure_pa)
{
    uint8_t  pbuf[3];
    uint8_t  tbuf[3];
    uint8_t  status;
    uint8_t  i;
    uint32_t timeout;
    int32_t  adc_P;
    int32_t  adc_T;
    int32_t  var1;
    int32_t  var2;
    int32_t  temp;
    uint32_t p;

    if (dev == NULL)
    {
        return BMP280_ERR_NULL_HANDLE;
    }

    /* Trigger a single forced measurement */
    bmp280_write_reg(dev, BMP280_REG_CTRL_MEAS,
                     (uint8_t)((BMP280_OSRS_T << 5) |
                               (BMP280_OSRS_P << 2) |
                               0x01));

    /*
- Give the conversion time to finish even if the status bit cannot be
- read correctly.  With pressure x16 this is about 43 ms.
     */
    dev->port.delay_ms(BMP280_MEASURE_DELAY_MS);

    /* Wait until the measurement is finished */
    timeout = BMP280_POLL_TIMEOUT_MS;

    for (;;)
    {
        bmp280_read_regs(dev, BMP280_REG_STATUS, &status, 1);

        if ((status & BMP280_STATUS_MEASURING) == 0)
        {
            break;
        }

        if (timeout < BMP280_POLL_PERIOD_MS)
        {
            return BMP280_ERR_TIMEOUT;
        }

        dev->port.delay_ms(BMP280_POLL_PERIOD_MS);
        timeout -= BMP280_POLL_PERIOD_MS;
    }

    /* Read pressure and temperature bytes individually. */
    for (i = 0U; i < 3U; ++i)
    {
        bmp280_read_regs(dev, (uint8_t)(BMP280_REG_PRESS_MSB + i),
                         &pbuf[i], 1);
        bmp280_read_regs(dev, (uint8_t)(BMP280_REG_TEMP_MSB + i),
                         &tbuf[i], 1);
    }

    adc_P = ((int32_t)pbuf[0] << 12) |
            ((int32_t)pbuf[1] << 4)  |
            ((int32_t)pbuf[2] >> 4);

    adc_T = ((int32_t)tbuf[0] << 12) |
            ((int32_t)tbuf[1] << 4)  |
            ((int32_t)tbuf[2] >> 4);

    dev->last_adc_p = adc_P;
    dev->last_adc_t = adc_T;

    /*
- A dead bus reads back all 0x00 or all 0xFF.  Reject both raw values
- being at an extreme rather than reporting a bogus 0.00 measurement.
     */
    if (((adc_P == 0) || (adc_P == 0xFFFFF)) &&
        ((adc_T == 0) || (adc_T == 0xFFFFF)))
    {
        return BMP280_ERR_DATA;
    }

    /* Temperature compensation. Also sets t_fine for pressure.
       Use 64-bit intermediates so out-of-range raw/trim values cannot
       overflow into a nonsense temperature. */
    {
        int64_t v1_64;
        int64_t v2_64;
        int64_t tf_64;
        int64_t temp_64;

        v1_64 = (((int64_t)(adc_T >> 3) -
                  ((int64_t)dev->calib.T1 << 1)) *
                 (int64_t)dev->calib.T2) >> 11;

        v2_64 = (((((int64_t)(adc_T >> 4) - (int64_t)dev->calib.T1) *
                   ((int64_t)(adc_T >> 4) - (int64_t)dev->calib.T1)) >> 12) *
                 (int64_t)dev->calib.T3) >> 14;

        tf_64 = v1_64 + v2_64;
        if (tf_64 > INT32_MAX)
        {
            tf_64 = INT32_MAX;
        }
        else if (tf_64 < INT32_MIN)
        {
            tf_64 = INT32_MIN;
        }

        dev->t_fine = (int32_t)tf_64;

        temp_64 = (tf_64 * 5 + 128) >> 8;
        if (temp_64 > INT32_MAX)
        {
            temp_64 = INT32_MAX;
        }
        else if (temp_64 < INT32_MIN)
        {
            temp_64 = INT32_MIN;
        }

        temp = (int32_t)temp_64;
    }
    dev->last_temp_centi = temp;

    if (temperature_c != NULL)
    {
        *temperature_c = (float)temp / 100.0f;
    }

    /* Pressure compensation */
    var1 = (((int32_t)dev->t_fine) / 2) - (int32_t)64000;

    var2 = (((var1 / 4) * (var1 / 4)) / 2048) *
           ((int32_t)dev->calib.P6);

    var2 = var2 + ((var1 * ((int32_t)dev->calib.P5)) * 2);
    var2 = (var2 / 4) + (((int32_t)dev->calib.P4) * 65536);

    var1 = ((((int32_t)dev->calib.P3 *
              (((var1 / 4) * (var1 / 4)) / 8192)) / 8) +
            ((((int32_t)dev->calib.P2) * var1) / 2)) / 262144;

    var1 = ((((32768 + var1)) * ((int32_t)dev->calib.P1)) / 32768);

    if (var1 == 0)
    {
        p = 0;
    }
    else
    {
        p = (uint32_t)(((int32_t)(1048576 - adc_P) -
                        (var2 / 4096)) * 3125);

        if (p < 0x80000000U)
        {
            p = (p << 1) / (uint32_t)var1;
        }
        else
        {
            p = (p / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)dev->calib.P9) *
                ((int32_t)(((p / 8) * (p / 8)) / 8192))) / 4096;

        var2 = (((int32_t)(p / 4)) *
                ((int32_t)dev->calib.P8)) / 8192;

        p = (uint32_t)((int32_t)p +
                       ((var1 + var2 + dev->calib.P7) / 16));
    }

    if (pressure_pa != NULL)
    {
        /*
         * Cast through int32_t on purpose.  On this toolchain the unsigned
         * uint32_t -> float conversion (__floatunsisf) is unreliable: it can
         * add 2^31 to the result, so 101251 Pa comes out as ~2147584899.
         * Valid barometric pressures always fit in int32_t.
         */
        *pressure_pa = (float)(int32_t)p;
    }

    dev->last_pressure_pa = p;

    return BMP280_OK;
}

/* --------------------------------------------------------------------------
- Altitude
- -------------------------------------------------------------------------- */

float bmp280_pressure_to_altitude(float pressure_pa)
{
    float r;
    float z;
    float z2;
    float ln_r;
    float term;
    float y;
    float sum;
    int i;

    if (pressure_pa <= 0.0f)
    {
        return 0.0f;
    }

    r = pressure_pa / 101325.0f;

    if (r >= 1.0f)
    {
        return 0.0f;
    }

    /*
- Compute (r ^ 0.190284) without powf().  Some nano-libm builds return
- 1.0 for this call, which makes the altitude come out as exactly zero.
     *
-   ln(r) = 2 * (z + z^3/3 + z^5/5 + ...),  z = (r - 1) / (r + 1)
-   exp(y) = 1 + y + y^2/2! + y^3/3! + ...
     */
    z = (r - 1.0f) / (r + 1.0f);
    z2 = z * z;
    term = z;
    ln_r = z;

    for (i = 3; i <= 19; i += 2)
    {
        term *= z2;
        ln_r += term / (float)i;
    }

    ln_r *= 2.0f;
    y = 0.190284f * ln_r;

    term = 1.0f;
    sum = 1.0f;

    for (i = 1; i <= 12; ++i)
    {
        term *= y / (float)i;
        sum += term;
    }

    return 44307.69396f * (1.0f - sum);
}

/*
 * Altitude in centimetres, using 32-bit integer maths only.
 *
 * This toolchain's soft-float is broken (powf() returns 1.0, uint32_t -> float
 * can add 2^31) and its 64-bit division helper (__divdi3) also misbehaves on
 * the THEJAS32, which made the old Q24 series return values like -1507234053.
 *
 * Instead, a pre-computed table of the real barometric formula
 *
 *     h = 44307.69396 * (1 - (P / 101325) ^ 0.190284)
 *
 * is stored for 30000..110000 Pa in 250 Pa steps and linearly interpolated.
 * Every intermediate fits in 32 bits (never more than ~1.3e6), so neither
 * libm nor any 64-bit helper is used.  Worst-case interpolation error is
 * about 6 cm over the full range.
 */
#define BMP280_ALT_BASE_PA   (30000U)
#define BMP280_ALT_STEP_PA   (250U)

static int32_t bmp280_pressure_to_altitude_cm(uint32_t pressure_pa)
{
    /* Pressure 30000..110000 Pa in 250 Pa steps, altitude in cm. */
    static const int32_t altitude_cm_table[] = {
          916007,  910452,  904935,  899454,  894009,  888599,  883224,  877884,
          872577,  867304,  862064,  856857,  851682,  846538,  841425,  836344,
          831293,  826272,  821280,  816317,  811384,  806478,  801602,  796752,
          791930,  787135,  782367,  777625,  772909,  768219,  763554,  758915,
          754300,  749710,  745144,  740602,  736083,  731588,  727117,  722667,
          718241,  713837,  709455,  705095,  700756,  696439,  692143,  687868,
          683613,  679379,  675166,  670972,  666798,  662644,  658509,  654393,
          650297,  646218,  642159,  638118,  634095,  630091,  626104,  622135,
          618183,  614250,  610332,  606432,  602549,  598683,  594833,  591000,
          587182,  583381,  579596,  575826,  572072,  568334,  564611,  560903,
          557210,  553532,  549869,  546220,  542586,  538967,  535361,  531770,
          528193,  524630,  521080,  517544,  514022,  510514,  507018,  503536,
          500067,  496611,  493167,  489736,  486318,  482913,  479520,  476139,
          472771,  469415,  466071,  462739,  459418,  456110,  452813,  449527,
          446254,  442991,  439740,  436501,  433272,  430055,  426848,  423652,
          420467,  417293,  414129,  410976,  407834,  404702,  401580,  398468,
          395367,  392276,  389195,  386123,  383062,  380010,  376969,  373937,
          370914,  367901,  364898,  361904,  358919,  355944,  352977,  350020,
          347073,  344134,  341204,  338282,  335371,  332467,  329572,  326686,
          323809,  320940,  318080,  315228,  312384,  309549,  306722,  303903,
          301093,  298291,  295496,  292710,  289931,  287161,  284399,  281643,
          278897,  276157,  273425,  270702,  267985,  265276,  262575,  259881,
          257194,  254514,  251842,  249178,  246520,  243870,  241226,  238590,
          235961,  233339,  230724,  228115,  225514,  222919,  220332,  217750,
          215176,  212608,  210047,  207493,  204945,  202403,  199868,  197340,
          194818,  192302,  189793,  187290,  184793,  182303,  179818,  177340,
          174868,  172402,  169942,  167488,  165041,  162599,  160163,  157733,
          155308,  152890,  150478,  148071,  145670,  143274,  140885,  138501,
          136122,  133750,  131382,  129021,  126664,  124314,  121968,  119628,
          117294,  114965,  112641,  110323,  108010,  105702,  103399,  101101,
           98809,   96522,   94240,   91963,   89691,   87424,   85163,   82906,
           80654,   78407,   76165,   73928,   71696,   69469,   67246,   65028,
           62815,   60607,   58404,   56205,   54011,   51822,   49637,   47457,
           45282,   43111,   40945,   38783,   36626,   34473,   32325,   30181,
           28041,   25907,   23776,   21650,   19528,   17410,   15297,   13188,
           11084,    8983,    6887,    4795,    2707,     624,   -1455,   -3530,
           -5601,   -7668,   -9731,  -11790,  -13845,  -15895,  -17942,  -19985,
          -22024,  -24058,  -26089,  -28116,  -30139,  -32158,  -34173,  -36185,
          -38192,  -40196,  -42196,  -44192,  -46184,  -48173,  -50157,  -52139,
          -54116,  -56089,  -58059,  -60025,  -61988,  -63947,  -65902,  -67854,
          -69802,
    };

    const uint32_t entry_count =
        (uint32_t)(sizeof(altitude_cm_table) / sizeof(altitude_cm_table[0]));
    uint32_t offset;
    uint32_t index;
    uint32_t remainder;
    int32_t delta;

    /* Clamp to the table range (30000..110000 Pa). */
    if (pressure_pa <= BMP280_ALT_BASE_PA)
    {
        return altitude_cm_table[0];
    }

    if (pressure_pa >= (BMP280_ALT_BASE_PA +
                        ((entry_count - 1U) * BMP280_ALT_STEP_PA)))
    {
        return altitude_cm_table[entry_count - 1U];
    }

    offset = pressure_pa - BMP280_ALT_BASE_PA;
    index = offset / BMP280_ALT_STEP_PA;
    remainder = offset % BMP280_ALT_STEP_PA;

    /* delta * remainder is at most ~1.3e6, so it stays inside int32_t. */
    delta = altitude_cm_table[index + 1U] - altitude_cm_table[index];

    return altitude_cm_table[index] +
           ((delta * (int32_t)remainder) / (int32_t)BMP280_ALT_STEP_PA);
}

bmp280_error_t bmp280_read_altitude(bmp280_t *dev, float *altitude_m)
{
    bmp280_error_t err;

    if (altitude_m == NULL)
    {
        return BMP280_ERR_NULL_ARG;
    }

    /* Read only: bmp280_read() stores the pressure in the device struct. */
    err = bmp280_read(dev, NULL, NULL);
    if (err != BMP280_OK)
    {
        return err;
    }

    *altitude_m = bmp280_pressure_to_altitude((float)dev->last_pressure_pa);

    return BMP280_OK;
}

/* --------------------------------------------------------------------------
- ARIES v3 platform binding: SPI2 + UART0 console
 *
- The BMP280 CSB pin is driven by the SPI2 controller's hardware chip select
- (CS0). CSAAT is raised for the duration of a register access so CSB stays
- low across the address byte and the following data bytes.
- -------------------------------------------------------------------------- */

#define BARO_UART_BAUD          115200U
#define BARO_SAMPLE_PERIOD_MS   100U
#define BARO_PRINT_EVERY        5U

/* THEJAS32 machine timer: 100 MHz / 4000 = 25 kHz. */
#define BARO_MTIME_ADDR         0x20010580UL
#define BARO_MTIME_HZ           25000UL

typedef struct
{
    volatile uint32_t dr;       /* 0x00 data register          */
    volatile uint32_t ie;       /* 0x04 interrupt enable       */
    volatile uint32_t iir_fcr;  /* 0x08 intr id / FIFO control */
    volatile uint32_t lcr;      /* 0x0C line control           */
    volatile uint32_t reserved; /* 0x10                        */
    volatile uint32_t lsr;      /* 0x14 line status            */
} baro_uart_regs_t;

#define BARO_UART0_REGS  ((baro_uart_regs_t *)0x10000100UL)
#define BARO_UART_LSR_THRE  (1U << 5)

/* Upper bound on the busy-wait, so a stalled timer can never wedge the app. */
#define BARO_DELAY_SPIN_LIMIT   2000000UL

static void baro_delay_ms(uint32_t ms)
{
    /*
     * Use only the low 32 bits of mtime.  At 25 kHz that wraps every
     * ~47.7 hours, and the unsigned subtraction below stays correct across
     * the wrap for any sane delay.  Keeping it 32-bit also avoids the
     * 64-bit __muldi3/__udivdi3 helpers on the critical init path.
     */
    volatile uint32_t *mtime_lo = (volatile uint32_t *)BARO_MTIME_ADDR;
    uint32_t ticks  = ms * (BARO_MTIME_HZ / 1000U);
    uint32_t start  = *mtime_lo;
    uint32_t spin   = 0U;

    while ((uint32_t)(*mtime_lo - start) < ticks)
    {
        if (++spin > BARO_DELAY_SPIN_LIMIT)
        {
            break;
        }
    }
}

static void baro_uart_init(void)
{
    uint32_t divisor = SYSTEM_FREQUENCY / (BARO_UART_BAUD * 16U);

    BARO_UART0_REGS->lcr = 0x83U;                 /* DLAB = 1 */
    BARO_UART0_REGS->dr = divisor & 0xFFU;        /* divisor low byte  */
    BARO_UART0_REGS->ie = (divisor >> 8) & 0xFFU; /* divisor high byte */
    BARO_UART0_REGS->lcr = 0x03U;                 /* 8N1, DLAB = 0 */
    BARO_UART0_REGS->ie = 0x00U;
    BARO_UART0_REGS->iir_fcr = 0x00U;
}

static void baro_uart_putc(char c)
{
    while ((BARO_UART0_REGS->lsr & BARO_UART_LSR_THRE) == 0U)
    {
    }
    BARO_UART0_REGS->dr = (uint32_t)(uint8_t)c;
}

static void baro_uart_print(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while (*text != '\0')
    {
        baro_uart_putc(*text++);
    }
}

static void baro_uart_print_hex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    char text[3];

    text[0] = hex[(value >> 4) & 0x0FU];
    text[1] = hex[value & 0x0FU];
    text[2] = '\0';
    baro_uart_print(text);
}

static void baro_uart_print_u32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do
    {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (count != 0U)
    {
        baro_uart_putc(digits[--count]);
    }
}

/* Print a value already scaled by 100, without using floating point. */
static void baro_uart_print_centi(int32_t centi)
{
    uint32_t magnitude;

    if (centi < 0)
    {
        baro_uart_putc('-');
        magnitude = (uint32_t)(-(centi + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)centi;
    }

    baro_uart_print_u32(magnitude / 100U);
    baro_uart_putc('.');
    baro_uart_putc((char)('0' + ((magnitude % 100U) / 10U)));
    baro_uart_putc((char)('0' + (magnitude % 10U)));
}

/* Print a signed integer value, without printf(). */
static void baro_uart_print_i32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        baro_uart_putc('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    baro_uart_print_u32(magnitude);
}

static void baro_uart_print_error(bmp280_error_t err)
{
    switch (err)
    {
        case BMP280_OK:
            baro_uart_print("OK");
            break;
        case BMP280_ERR_NULL_HANDLE:
            baro_uart_print("null handle");
            break;
        case BMP280_ERR_NULL_PORT:
            baro_uart_print("null port");
            break;
        case BMP280_ERR_NULL_ARG:
            baro_uart_print("null argument");
            break;
        case BMP280_ERR_SENSOR_ID:
            baro_uart_print("unexpected chip ID");
            break;
        case BMP280_ERR_TIMEOUT:
            baro_uart_print("timeout");
            break;
        case BMP280_ERR_CALIB:
            baro_uart_print("invalid calibration data");
            break;
        case BMP280_ERR_DATA:
            baro_uart_print("no valid measurement data");
            break;
        default:
            baro_uart_print("unknown error");
            break;
    }
}

/* --------------------------------------------------------------------------
- SPI2 port hooks
- -------------------------------------------------------------------------- */

/*
- The BSP SPI helpers (SPI_wait_if_busy/SPI_transmit/SPI_receive) poll status
- bits with no timeout.  If SPI2 never completes a transfer they spin forever
- and the application appears to freeze right after its banner.  Poll with a
- bound instead so a bus problem is reported as an error.
 */
#define BARO_SPI_SPIN_LIMIT     200000UL

static uint8_t baro_spi2_failed;

static int baro_spi2_poll(uint32_t mask, int want_set)
{
    uint32_t spin;

    for (spin = 0U; spin < BARO_SPI_SPIN_LIMIT; ++spin)
    {
        uint32_t status = SPIreg(BMP280_SPI2_BUS).Status;

        if (((status & mask) != 0U) == (want_set != 0))
        {
            return 0;
        }
    }

    return -1;
}

static void baro_spi2_cs_low(void)
{
    if (baro_spi2_failed != 0U)
    {
        return;
    }

    if (baro_spi2_poll(SPI_BUSY_BIT, 0) != 0)
    {
        baro_spi2_failed = 1U;
        return;
    }

    SPI_set_CSAAT_pin(BMP280_SPI2_BUS, HIGH);
}

static void baro_spi2_cs_high(void)
{
    /* Always release CS, even after a failed transfer. */
    SPI_set_CSAAT_pin(BMP280_SPI2_BUS, LOW);
}

static uint8_t baro_spi2_transfer(uint8_t tx)
{
    if (baro_spi2_failed != 0U)
    {
        return 0xFFU;
    }

    /* Match the BSP SPI sequence: wait for the controller to be idle,
       then for room in the TX holding register. */
    if (baro_spi2_poll(SPI_BUSY_BIT, 0) != 0)
    {
        baro_spi2_failed = 1U;
        return 0xFFU;
    }

    if (baro_spi2_poll(SPI_TX_HOLD_EMPTY_BIT, 1) != 0)
    {
        baro_spi2_failed = 1U;
        return 0xFFU;
    }

    SPIreg(BMP280_SPI2_BUS).TxData = (US)tx;
    __asm__ __volatile__ ("fence" ::: "memory");

    if (baro_spi2_poll(SPI_RX_COMPLETE_BIT, 1) != 0)
    {
        baro_spi2_failed = 1U;
        return 0xFFU;
    }

    return (uint8_t)(SPIreg(BMP280_SPI2_BUS).RxData & 0xFFU);
}

static void baro_spi2_bus_init(void)
{
    static int initialized = 0;
    SPIcntrlRegType control;

    if (initialized != 0)
    {
        return;
    }

    SPI_init(BMP280_SPI2_BUS);

    /*
- 8 data bits, MSB first, SPI mode 3, fixed peripheral, hardware CS0.
- BMP280 supports mode 0 and mode 3; mode 3 matches the BSP default.
     */
    control.Value = 0U;
    control.Bits.PeriphChipSelect = (US)BMP280_SPI2_CS;
    control.Bits.PeriphType = SPI_FIXED_PERIPHERAL;
    control.Bits.DataOrder = SPI_MSB_FIRST;
    control.Bits.Mode = SPI_MODE_3;
    control.Bits.SPIrxTxIntr = RX_TX_INTR_DIS;
    control.Bits.CSAAT = LOW;
    control.Bits.Dbits = 0;                 /* 8 bits per transfer */

    SPI_config(BMP280_SPI2_BUS, control);

    /* 100 MHz / 32 = 3.125 MHz, below the BMP280 10 MHz limit. */
    SPI_set_baud(BMP280_SPI2_BUS, SPI_BAUD_CFD_32);

    initialized = 1;
}

static uint8_t baro_read_chip_id(void)
{
    uint8_t id;

    baro_spi2_cs_low();
    (void)baro_spi2_transfer((uint8_t)(BMP280_REG_ID | BMP280_SPI_READ));
    id = baro_spi2_transfer(0xFFU);
    baro_spi2_cs_high();

    return id;
}

bmp280_error_t bmp280_spi2_init(bmp280_t *dev)
{
    bmp280_error_t err;

    if (dev == NULL)
    {
        return BMP280_ERR_NULL_HANDLE;
    }

    baro_spi2_failed = 0U;
    baro_spi2_bus_init();

    dev->port.cs_low = baro_spi2_cs_low;
    dev->port.cs_high = baro_spi2_cs_high;
    dev->port.transfer = baro_spi2_transfer;
    dev->port.delay_ms = baro_delay_ms;

    err = bmp280_init(dev);

    /* A stuck status register means the bus, not the sensor, is the issue. */
    if (baro_spi2_failed != 0U)
    {
        return BMP280_ERR_TIMEOUT;
    }

    return err;
}

/* --------------------------------------------------------------------------
- Top-level barometer output loop
- -------------------------------------------------------------------------- */

void baro_run(void)
{
    bmp280_t dev;
    int32_t altitude_cm;
    uint32_t sample_count = 0U;
    bmp280_error_t err;
    

    baro_uart_init();
    baro_uart_print("\r\nBMP280 barometer (ARIES v3, SPI2)\r\n");

    err = bmp280_spi2_init(&dev);
    if (err != BMP280_OK)
    {
        baro_uart_print("ERROR: BMP280 init failed (");
        baro_uart_print_error(err);
        baro_uart_print(")\r\n");

        if (err == BMP280_ERR_TIMEOUT)
        {
            baro_uart_print("SPI2 never completed a transfer; "
                            "check that the BMP280 is on J4 and the "
                            "SPI2 pins are wired correctly\r\n");
        }
        else if (err == BMP280_ERR_CALIB)
        {
            baro_uart_print("Calibration registers 0x88..0x9F read as all "
                            "0x00 or all 0xFF; check BMP280 supply, CSB on "
                            "J4_6 and SPI2 wiring\r\n");
        }
        else
        {
            baro_uart_print("chip ID 0x");
            baro_uart_print_hex8(baro_read_chip_id());
            baro_uart_print(" (expected 0x58)\r\n");
            baro_uart_print("Check SPI2 wiring: SCK, MOSI, MISO and CSB\r\n");
        }

        for (;;)
        {
            baro_delay_ms(1000U);
        }
    }

    baro_uart_print("Ready: forced mode, temp x2, pressure x16, IIR x16\r\n");

    for (;;)
    {
        err = bmp280_read(&dev, NULL, NULL);
        if (err != BMP280_OK)
        {
            baro_uart_print("ERROR: BMP280 read failed (");
            baro_uart_print_error(err);
            baro_uart_print(")\r\n");
            baro_delay_ms(BARO_SAMPLE_PERIOD_MS);
            continue;
        }

        /*
         * Keep altitude on the integer path: the soft-float conversions on
         * this target are not dependable (see the note in bmp280_read()).
         */
        altitude_cm = bmp280_pressure_to_altitude_cm(dev.last_pressure_pa);

        if (++sample_count >= BARO_PRINT_EVERY)
        {
            sample_count = 0U;

            baro_uart_print("temp=");
            baro_uart_print_centi(dev.last_temp_centi);
            baro_uart_print(" C  pressure=");
            baro_uart_print_u32(dev.last_pressure_pa);
            baro_uart_print(" Pa  altitude=");
            baro_uart_print_i32(altitude_cm);
            baro_uart_print(" cm\r\n");
            
            


        }

        baro_delay_ms(BARO_SAMPLE_PERIOD_MS);
    }
}
