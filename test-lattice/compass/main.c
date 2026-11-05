/**
 * @file    main.c
 * @brief   RadioLink SE100 V2.0 (IST8310) compass test for the ARIES v3.
 *
 * The SE100's 4-pin compass port is connected to the board I2C bus.  Readings
 * are printed on UART0, the USB-C console, at 115200 8N1 so the output can be
 * viewed directly with:
 *
 *     picocom -b 115200 /dev/ttyUSB0
 */

#include <stddef.h>

#include "config.h"
#include "uart.h"
#include "ist8310.h"

/*
 * Declination and mounting offset are stored in tenths of a degree.
 * Example: -12.3 degrees of declination is -123.
 *
 * The heading maths is integer-only.  The soft-float atan2f() in the
 * newlib-nano build was returning wrong quadrants on this RV32I target,
 * which made the heading stick at 0/270 and jump between quadrants.
 */
#define APP_DECLINATION_TENTHS    0     /* set to your local declination */
#define APP_HEADING_OFFSET_TENTHS 0     /* module mounting/axis offset  */

#define APP_UART            UART_0
#define APP_UART_BAUD       115200UL
#define APP_PRINT_PERIOD_MS 200U

/*
 * The SE100 is rotated through a full circle once at boot.  Set
 * APP_CALIBRATE_ON_BOOT to 0 only if you have a stored calibration.
 */
#define APP_CALIBRATE_ON_BOOT       1
#define APP_CAL_START_DELAY_MS      3000U
#define APP_CAL_SAMPLE_PERIOD_MS    20U
#define APP_CAL_SAMPLE_COUNT        500U  /* roughly 12-20 s including reads */

#define APP_MTIME_ADDR      0x20010580UL
#define APP_MTIME_HZ        25000UL

static void app_delay_ms(uint32_t ms)
{
    volatile uint64_t *mtime = (volatile uint64_t *)APP_MTIME_ADDR;
    uint64_t deadline = *mtime + (((uint64_t)ms * APP_MTIME_HZ) / 1000U);
    uint32_t spin = 0U;

    while (*mtime < deadline)
    {
        if (++spin > 2000000UL)
        {
            break;
        }
    }
}

static void app_uart_putc(char c)
{
    char error = UART_NO_ERROR;

    uart_putchar(APP_UART, (UC)c, &error);
}

static void app_uart_print(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while (*text != '\0')
    {
        app_uart_putc(*text++);
    }
}

static void app_uart_u32(uint32_t value)
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
        app_uart_putc(digits[--count]);
    }
}

static void app_uart_i32(int32_t value)
{
    if (value < 0)
    {
        app_uart_putc('-');
        app_uart_u32((uint32_t)(-(value + 1)) + 1U);
    }
    else
    {
        app_uart_u32((uint32_t)value);
    }
}

/* Print nanotesla as a signed value in microtesla with 3 decimals. */
static void app_uart_ut(int32_t nanotesla)
{
    uint32_t magnitude;
    uint32_t frac;

    if (nanotesla < 0)
    {
        app_uart_putc('-');
        magnitude = (uint32_t)(-(nanotesla + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)nanotesla;
    }

    frac = magnitude % 1000U;
    app_uart_u32(magnitude / 1000U);
    app_uart_putc('.');
    app_uart_putc((char)('0' + (frac / 100U)));
    app_uart_putc((char)('0' + ((frac / 10U) % 10U)));
    app_uart_putc((char)('0' + (frac % 10U)));
}

static void app_uart_hex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    app_uart_putc(hex[(value >> 4) & 0x0FU]);
    app_uart_putc(hex[value & 0x0FU]);
}

#if APP_CALIBRATE_ON_BOOT
/* Print a Q12 scale factor as, for example, 1.023. */
static void app_uart_q12(uint16_t value)
{
    uint32_t thousandths =
        (((uint32_t)value * 1000U) + 2048U) / 4096U;

    app_uart_u32(thousandths / 1000U);
    app_uart_putc('.');
    app_uart_putc((char)('0' + ((thousandths / 100U) % 10U)));
    app_uart_putc((char)('0' + ((thousandths / 10U) % 10U)));
    app_uart_putc((char)('0' + (thousandths % 10U)));
}
#endif

/* atan(i / 64) in tenths of a degree, i = 0..64. */
static const uint16_t app_atan_table[65] =
{
      0,   9,  18,  27,  36,  45,  54,  62,
     71,  80,  89,  98, 106, 115, 123, 132,
    140, 149, 157, 165, 174, 182, 190, 198,
    206, 213, 221, 229, 236, 244, 251, 258,
    266, 273, 280, 287, 294, 300, 307, 314,
    320, 326, 333, 339, 345, 351, 357, 363,
    369, 374, 380, 386, 391, 396, 402, 407,
    412, 417, 422, 427, 432, 436, 441, 445,
    450
};

static uint32_t app_abs_i32(int32_t value)
{
    if (value < 0)
    {
        return (uint32_t)(-(value + 1)) + 1U;
    }

    return (uint32_t)value;
}

/*
 * Integer atan2(y, x) in tenths of a degree, 0..3599.
 *
 * The first octant is reduced to atan(min/max), interpolated from the
 * table above, then mirrored into the other octants and quadrants.  This
 * avoids the soft-float atan2f() implementation entirely.
 */
static int32_t app_atan2_tenths(int32_t y, int32_t x)
{
    uint32_t ax = app_abs_i32(x);
    uint32_t ay = app_abs_i32(y);
    uint32_t maximum;
    uint32_t minimum;
    uint32_t ratio_q15;
    uint32_t index;
    uint32_t fraction;
    uint32_t angle;
    int swapped = 0;

    if ((ax == 0U) && (ay == 0U))
    {
        return 0;
    }

    if (ax >= ay)
    {
        maximum = ax;
        minimum = ay;
    }
    else
    {
        maximum = ay;
        minimum = ax;
        swapped = 1;
    }

    /* Keep min << 15 and the division inside 32 bits. */
    while (maximum > 65535U)
    {
        maximum >>= 1;
        minimum >>= 1;
    }

    ratio_q15 = (minimum << 15) / maximum;
    index = ratio_q15 >> 9;                 /* 0..64 */
    fraction = ratio_q15 & 0x1FFU;          /* 0..511 */

    angle = app_atan_table[index];
    if (index < 64U)
    {
        uint32_t next = app_atan_table[index + 1U];
        angle += (((next - angle) * fraction) + 256U) >> 9;
    }

    if (swapped != 0)
    {
        angle = 900U - angle;
    }

    if (x < 0)
    {
        angle = 1800U - angle;
    }

    if (y < 0)
    {
        angle = 3600U - angle;
    }

    if (angle >= 3600U)
    {
        angle -= 3600U;
    }

    return (int32_t)angle;
}

/*
 * Magnetic heading for a level sensor, in tenths of a degree.
 *
 * X is the sensor's forward axis and Y its right-hand axis.  A field vector
 * that points at magnetic north therefore gives atan2(-Y, X) = 0 degrees.
 * Heading increases clockwise: 0 = N, 90 = E, 180 = S, 270 = W.
 */
static int32_t app_heading_tenths(const ist8310_sample_t *sample)
{
    int32_t heading = app_atan2_tenths(-sample->y_nt, sample->x_nt);

    heading += APP_DECLINATION_TENTHS + APP_HEADING_OFFSET_TENTHS;

    while (heading < 0)
    {
        heading += 3600;
    }

    while (heading >= 3600)
    {
        heading -= 3600;
    }

    return heading;
}

static void app_uart_heading(int32_t tenths)
{
    app_uart_u32((uint32_t)(tenths / 10));
    app_uart_putc('.');
    app_uart_putc((char)('0' + (tenths % 10)));
    app_uart_print(" deg");
}

#if APP_CALIBRATE_ON_BOOT
static void app_print_calibration(const ist8310_calibration_t *cal)
{
    app_uart_print("Calibration OK: hard-iron X=");
    app_uart_i32(cal->x_offset);
    app_uart_print(" Y=");
    app_uart_i32(cal->y_offset);
    app_uart_print(" counts; soft-iron X=");
    app_uart_q12(cal->x_scale_q12);
    app_uart_print(" Y=");
    app_uart_q12(cal->y_scale_q12);
    app_uart_print(" (samples=");
    app_uart_u32(cal->samples);
    app_uart_print(")\r\n");
}

/*
 * Collect X/Y extrema while the user rotates the module through a full
 * circle.  No calibration values are guessed: if the motion is insufficient,
 * finish() rejects the run and identity correction is used.
 */
static void app_run_calibration(ist8310_calibration_t *cal)
{
    ist8310_sample_t sample;
    uint32_t i;
    int result;

    app_uart_print("\r\nCALIBRATION: rotate the SE100 slowly through a full "
                   "360 degrees.\r\n");
    app_uart_print("Keep it level, away from metal/magnets. Starting in 3 s;\r\n");
    app_uart_print("keep rotating until all five dots have printed.\r\n");
    app_delay_ms(APP_CAL_START_DELAY_MS);

    if (ist8310_calibration_start(cal) != 0)
    {
        app_uart_print("Calibration could not start; using uncalibrated X/Y.\r\n");
        return;
    }

    for (i = 0U; i < APP_CAL_SAMPLE_COUNT; ++i)
    {
        if (ist8310_read(&sample) == 0)
        {
            (void)ist8310_calibration_add(cal, &sample);
        }

        if (((i + 1U) % 100U) == 0U)
        {
            app_uart_putc('.');
        }

        app_delay_ms(APP_CAL_SAMPLE_PERIOD_MS);
    }

    app_uart_print("\r\n");

    result = ist8310_calibration_finish(cal);
    if (result == 0)
    {
        app_print_calibration(cal);
    }
    else
    {
        app_uart_print("Calibration rejected (code ");
        app_uart_i32(result);
        app_uart_print("); using uncalibrated X/Y.\r\n");
    }
}
#endif /* APP_CALIBRATE_ON_BOOT */

/* A light vector filter; filtering X/Y avoids the 0/360-degree wrap problem
   that smoothing the angle itself would have. */
static int32_t app_smooth_x_nt = 0;
static int32_t app_smooth_y_nt = 0;
static uint8_t app_smooth_valid = 0U;

static void app_filter_xy(int32_t x_nt, int32_t y_nt,
                          int32_t *filtered_x_nt, int32_t *filtered_y_nt)
{
    if (app_smooth_valid == 0U)
    {
        app_smooth_x_nt = x_nt;
        app_smooth_y_nt = y_nt;
        app_smooth_valid = 1U;
    }
    else
    {
        app_smooth_x_nt = ((3 * app_smooth_x_nt) + x_nt) / 4;
        app_smooth_y_nt = ((3 * app_smooth_y_nt) + y_nt) / 4;
    }

    *filtered_x_nt = app_smooth_x_nt;
    *filtered_y_nt = app_smooth_y_nt;
}

static void app_print_sample(const ist8310_sample_t *sample)
{
    app_uart_print("mag[uT] X=");
    app_uart_ut(sample->x_nt);
    app_uart_print(" Y=");
    app_uart_ut(sample->y_nt);
    app_uart_print(" Z=");
    app_uart_ut(sample->z_nt);

    app_uart_print("  heading=");
    app_uart_heading(app_heading_tenths(sample));

    app_uart_print("  raw X=");
    app_uart_i32(sample->x_raw);
    app_uart_print(" Y=");
    app_uart_i32(sample->y_raw);
    app_uart_print(" Z=");
    app_uart_i32(sample->z_raw);
    app_uart_print("\r\n");
}

int main(void)
{
    ist8310_sample_t sample;
    ist8310_calibration_t calibration;
    int32_t calibrated_x;
    int32_t calibrated_y;
    int32_t filtered_x;
    int32_t filtered_y;
    uint8_t id = 0U;
    int init_result;

    uart_configure(APP_UART, APP_UART_BAUD, 0x83UL, SYSTEM_FREQUENCY);

    app_uart_print("\r\nRadioLink SE100 V2.0 - IST8310 compass\r\n");
    app_uart_print("ARIES v3 / THEJAS32, I2C @ 100 kHz, UART0 @ 115200 8N1\r\n");
    app_uart_print("Initializing IST8310 on I2C0 (address 0x0E)...\r\n");

    init_result = ist8310_init();
    if (init_result != 0)
    {
        app_uart_print("ERROR: IST8310 init failed (code ");
        app_uart_i32(init_result);
        app_uart_print(", I2C stage ");
        app_uart_i32(ist8310_last_error());
        app_uart_print(")\r\n");

        if (ist8310_read_who_am_i(&id) == 0)
        {
            app_uart_print("WHO_AM_I=0x");
            app_uart_hex8(id);
            app_uart_print(" (expected 0x10)\r\n");
        }
        else
        {
            app_uart_print("No response from 0x0E; check SDA/SCL/5V/GND.\r\n");
        }

        for (;;)
        {
            app_uart_print("Still waiting: fix wiring/power, then reset the board.\r\n");
            app_delay_ms(1000U);
        }
    }

    if (ist8310_read_who_am_i(&id) == 0)
    {
        app_uart_print("IST8310 detected, WHO_AM_I=0x");
        app_uart_hex8(id);
        app_uart_print("\r\n");
    }

    ist8310_calibration_reset(&calibration);

#if APP_CALIBRATE_ON_BOOT
    app_run_calibration(&calibration);
#else
    app_uart_print("Boot calibration disabled; using uncalibrated X/Y.\r\n");
#endif

    app_uart_print("Output: calibrated X/Y/Z in microtesla (uT), heading in "
                   "degrees (0=N, 90=E, clockwise)\r\n\r\n");

    for (;;)
    {
        if (ist8310_read(&sample) != 0)
        {
            app_uart_print("ERROR: magnetometer read failed\r\n");
            app_delay_ms(500U);
            continue;
        }

        /* Heading must use corrected X/Y, not the raw magnetometer counts. */
        ist8310_calibration_apply(&calibration, &sample,
                                  &calibrated_x, &calibrated_y);
        sample.x_nt = calibrated_x * IST8310_NT_PER_LSB;
        sample.y_nt = calibrated_y * IST8310_NT_PER_LSB;

        app_filter_xy(sample.x_nt, sample.y_nt, &filtered_x, &filtered_y);
        sample.x_nt = filtered_x;
        sample.y_nt = filtered_y;

        app_print_sample(&sample);
        app_delay_ms(APP_PRINT_PERIOD_MS);
    }
}
