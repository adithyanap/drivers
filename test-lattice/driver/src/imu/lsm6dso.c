/**
 * @file    lsm6dso.c
 * @brief   LSM6DSO accel/gyro driver and IMU output loop for ARIES v3.
 *
 * The IMU is wired to the ARIES v3 J7 header, which is the dedicated SPI1
 * port (SPI1MOSI / SPI1MISO / SPI1CLK / SPI1CSN).  Sensor values are printed
 * to UART0, the USB-C serial console, at 115200 baud.
 */

#include "lsm6dso.h"
#include "stdlib.h"
#include "config.h"
#include "spi.h"

#include <stddef.h>

/* -------------------------------------------------------------------------
 * Application configuration
 * ---------------------------------------------------------------------- */

#define IMU_SPI_BUS          ((UC)1)   /* ARIES v3 J7 = SPI1 */
#define IMU_SPI_CS           0U        /* SPI1CSN = hardware CS0 */
#define IMU_UART_BAUD        115200U
#define IMU_PRINT_EVERY      100U

/* Little-endian pair from the LSM6DSO output registers. */
#define LSM6DSO_MAKE_INT16(lo, hi) \
    ((int16_t)((uint16_t)(lo) | ((uint16_t)(hi) << 8)))

/* -------------------------------------------------------------------------
 * UART0 (USB-C serial console)
 * ---------------------------------------------------------------------- */

typedef struct
{
    volatile uint32_t dr;       /* 0x00 data register          */
    volatile uint32_t ie;       /* 0x04 interrupt enable       */
    volatile uint32_t iir_fcr;  /* 0x08 intr id / FIFO control */
    volatile uint32_t lcr;      /* 0x0C line control           */
    volatile uint32_t reserved; /* 0x10                        */
    volatile uint32_t lsr;      /* 0x14 line status            */
} uart_regs_t;

#define UART0_REGS  ((uart_regs_t *)0x10000100UL)

#define UART_LSR_THRE  (1U << 5)

/* THEJAS32 machine timer: 100 MHz / 4000 = 25 kHz. */
#define MTIME_ADDR  0x20010580UL
#define MTIME_HZ    25000UL

static void delay_ms(uint32_t ms)
{
    volatile uint64_t *mtime = (volatile uint64_t *)MTIME_ADDR;
    uint64_t deadline = *mtime + (((uint64_t)ms * MTIME_HZ) / 1000U);

    while (*mtime < deadline)
    {
    }
}

static void uart_init(void)
{
    uint32_t divisor = SYSTEM_FREQUENCY / (IMU_UART_BAUD * 16U);

    UART0_REGS->lcr = 0x83U;                 /* DLAB = 1 */
    UART0_REGS->dr = divisor & 0xFFU;        /* divisor low byte  */
    UART0_REGS->ie = (divisor >> 8) & 0xFFU; /* divisor high byte */
    UART0_REGS->lcr = 0x03U;                 /* 8N1, DLAB = 0 */
    UART0_REGS->ie = 0x00U;
    UART0_REGS->iir_fcr = 0x00U;
}

static void uart_putc(char c)
{
    while ((UART0_REGS->lsr & UART_LSR_THRE) == 0U)
    {
    }
    UART0_REGS->dr = (uint32_t)(uint8_t)c;
}

static void uart_print(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while (*text != '\0')
    {
        uart_putc(*text++);
    }
}

static void uart_print_hex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    char text[3];

    text[0] = hex[(value >> 4) & 0x0FU];
    text[1] = hex[value & 0x0FU];
    text[2] = '\0';
    uart_print(text);
}

/* Print a float with exactly two decimal places, without printf(). */
static void uart_print_fixed2(float value)
{
    char buffer[16];
    char digits[10];
    char *p = buffer;
    int32_t scaled;
    uint32_t magnitude;
    uint32_t whole;
    uint32_t fraction;
    uint8_t count = 0U;

    scaled = (int32_t)(value * 100.0f + (value >= 0.0f ? 0.5f : -0.5f));

    if (scaled < 0)
    {
        *p++ = '-';
        magnitude = (uint32_t)(-(scaled + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)scaled;
    }

    whole = magnitude / 100U;
    fraction = magnitude % 100U;

    do
    {
        digits[count++] = (char)('0' + (whole % 10U));
        whole /= 10U;
    } while (whole != 0U);

    while (count != 0U)
    {
        *p++ = digits[--count];
    }

    *p++ = '.';
    *p++ = (char)('0' + (fraction / 10U));
    *p++ = (char)('0' + (fraction % 10U));
    *p = '\0';

    uart_print(buffer);
}

static void uart_print_imu(const imu_data_t *data)
{
    uart_print("gyro[dps] X=");
    uart_print_fixed2(data->gyro_x);
    uart_print(" Y=");
    uart_print_fixed2(data->gyro_y);
    uart_print(" Z=");
    uart_print_fixed2(data->gyro_z);

    uart_print(" | accel[g] X=");
    uart_print_fixed2(data->accel_x);
    uart_print(" Y=");
    uart_print_fixed2(data->accel_y);
    uart_print(" Z=");
    uart_print_fixed2(data->accel_z);
    uart_print("\r\n");
}

/* -------------------------------------------------------------------------
 * SPI1 hardware
 * ---------------------------------------------------------------------- */

static void imu_hw_init(void)
{
    static int initialized = 0;
    SPIcntrlRegType control;

    if (initialized != 0)
    {
        return;
    }

    uart_init();

    /* ARIES v3 J7 uses SPI1 with its dedicated SPI1CSN (CS0). */
    SPI_init(IMU_SPI_BUS);

    /*
     * SPI_init() selects the BSP defaults; override them for the LSM6DSO:
     * 8 data bits, MSB first, SPI mode 3, fixed peripheral, CS0.
     */
    control.Value = 0U;
    control.Bits.PeriphChipSelect = (US)IMU_SPI_CS;
    control.Bits.PeriphType = SPI_FIXED_PERIPHERAL;
    control.Bits.DataOrder = SPI_MSB_FIRST;
    control.Bits.Mode = 3;
    control.Bits.SPIrxTxIntr = RX_TX_INTR_DIS;
    control.Bits.CSAAT = LOW;
    control.Bits.Dbits = 0;                /* DBITS_8 */

    SPI_config(IMU_SPI_BUS, control);
    /* 100 MHz / 32 = 3.125 MHz; well below the LSM6DSO 10 MHz limit and
       tolerant of the J7 header wiring. */
    SPI_set_baud(IMU_SPI_BUS, SPI_BAUD_CFD_32);

    initialized = 1;
}

static int imu_spi_write_read(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    uint16_t i;

    if ((tx == NULL) || (rx == NULL) || (len == 0U))
    {
        return -1;
    }

    SPI_wait_if_busy(IMU_SPI_BUS);

    /* Keep SPI1CSN asserted for the complete transfer. */
    SPI_set_CSAAT_pin(IMU_SPI_BUS, HIGH);

    for (i = 0U; i < len; ++i)
    {
        SPI_transmit(IMU_SPI_BUS, tx[i]);
        rx[i] = (uint8_t)SPI_receive(IMU_SPI_BUS);
    }

    SPI_set_CSAAT_pin(IMU_SPI_BUS, LOW);
    return 0;
}

/* -------------------------------------------------------------------------
 * Register access helpers
 * ---------------------------------------------------------------------- */

static int lsm6dso_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)(reg | LSM6DSO_SPI_READ_BIT);
    tx[1] = 0U;

    if (imu_spi_write_read(tx, rx, sizeof(tx)) != 0)
    {
        return -1;
    }

    *value = rx[1];
    return 0;
}

static int lsm6dso_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)(reg & 0x7FU);
    tx[1] = value;

    return (imu_spi_write_read(tx, rx, sizeof(tx)) == 0) ? 0 : -1;
}

/* -------------------------------------------------------------------------
 * Public driver API
 * ---------------------------------------------------------------------- */

int imu_read_who_am_i(uint8_t *value)
{
    if (value == NULL)
    {
        return -1;
    }
    return lsm6dso_read_reg(LSM6DSO_REG_WHO_AM_I, value);
}

int imu_init(void)
{
    uint8_t who_am_i;
    uint8_t ctrl3;

    imu_hw_init();

    /* Verify the part before changing any configuration. */
    if (lsm6dso_read_reg(LSM6DSO_REG_WHO_AM_I, &who_am_i) != 0)
    {
        return -1;
    }
    if (who_am_i != LSM6DSO_WHO_AM_I_VALUE)
    {
        return -2;
    }

    /* Soft reset; the reset bit self-clears after the device boot time. */
    if (lsm6dso_write_reg(LSM6DSO_REG_CTRL3_C, LSM6DSO_CTRL3_SW_RESET) != 0)
    {
        return -3;
    }
    delay_ms(10U);

    /* Coherent 16-bit outputs and automatic address increment for bursts. */
    ctrl3 = (uint8_t)(LSM6DSO_CTRL3_BDU | LSM6DSO_CTRL3_IF_INC);
    if (lsm6dso_write_reg(LSM6DSO_REG_CTRL3_C, ctrl3) != 0)
    {
        return -4;
    }

    /* Accel: 1.666 kHz, +/-16 g. */
    if (lsm6dso_write_reg(LSM6DSO_REG_CTRL1_XL, LSM6DSO_CTRL1_XL_VALUE) != 0)
    {
        return -5;
    }

    /* Gyro: 1.666 kHz, +/-2000 dps. */
    if (lsm6dso_write_reg(LSM6DSO_REG_CTRL2_G, LSM6DSO_CTRL2_G_VALUE) != 0)
    {
        return -6;
    }

    /* Allow the digital filters to settle before the first sample. */
    delay_ms(50U);
    return 0;
}

int imu_data_ready(void)
{
    uint8_t status;

    if (lsm6dso_read_reg(LSM6DSO_REG_STATUS, &status) != 0)
    {
        return -1;
    }

    return ((status & (LSM6DSO_STATUS_XLDA | LSM6DSO_STATUS_GDA)) ==
            (LSM6DSO_STATUS_XLDA | LSM6DSO_STATUS_GDA)) ? 1 : 0;
}

int imu_read(imu_data_t *data)
{
    /*
     * One SPI transaction:
     *   byte 0: read | OUTX_L_G (auto-increment enabled in CTRL3_C)
     *   bytes 1..12: gyro X/Y/Z then accel X/Y/Z
     */
    uint8_t tx[LSM6DSO_DATA_BURST_LEN + 1U] = {
        (uint8_t)(LSM6DSO_REG_OUTX_L_G | LSM6DSO_SPI_READ_BIT)
    };
    uint8_t rx[LSM6DSO_DATA_BURST_LEN + 1U];

    if (data == NULL)
    {
        return -1;
    }

    if (imu_spi_write_read(tx, rx, sizeof(tx)) != 0)
    {
        return -2;
    }

    data->gyro_x = (float)LSM6DSO_MAKE_INT16(rx[1], rx[2]) *
                   LSM6DSO_GYRO_DPS_PER_LSB;
    data->gyro_y = (float)LSM6DSO_MAKE_INT16(rx[3], rx[4]) *
                   LSM6DSO_GYRO_DPS_PER_LSB;
    data->gyro_z = (float)LSM6DSO_MAKE_INT16(rx[5], rx[6]) *
                   LSM6DSO_GYRO_DPS_PER_LSB;

    data->accel_x = (float)LSM6DSO_MAKE_INT16(rx[7], rx[8]) *
                    LSM6DSO_ACCEL_G_PER_LSB;
    data->accel_y = (float)LSM6DSO_MAKE_INT16(rx[9], rx[10]) *
                    LSM6DSO_ACCEL_G_PER_LSB;
    data->accel_z = (float)LSM6DSO_MAKE_INT16(rx[11], rx[12]) *
                    LSM6DSO_ACCEL_G_PER_LSB;

    return 0;
}

/* -------------------------------------------------------------------------
 * Top-level IMU output loop
 * ---------------------------------------------------------------------- */

void imu_run(void)
{
    imu_data_t data;
    uint32_t sample_count = 0U;

    uart_init();
    uart_print("\r\nLSM6DSO IMU (ARIES v3, SPI1/J7)\r\n");

    if (imu_init() != 0)
    {
        uint8_t who = 0U;

        if (imu_read_who_am_i(&who) == 0)
        {
            uart_print("WHO_AM_I returned 0x");
            uart_print_hex8(who);
            uart_print(" (expected 0x6C)\r\n");
        }

        uart_print("ERROR: LSM6DSO init failed (check SPI1 wiring and CS)\r\n");
        for (;;)
        {
            delay_ms(1000U);
        }
    }

    uart_print("Ready: 1666 Hz, +/-16 g, +/-2000 dps\r\n");

    for (;;)
    {
        int ready = imu_data_ready();

        if (ready < 0)
        {
            uart_print("ERROR: SPI status read failed\r\n");
            delay_ms(100U);
            continue;
        }

        if (ready == 0)
        {
            continue;
        }

        if (imu_read(&data) != 0)
        {
            uart_print("ERROR: SPI burst read failed\r\n");
            delay_ms(100U);
            continue;
        }

        if (++sample_count >= IMU_PRINT_EVERY)
        {
            sample_count = 0U;
            uart_print_imu(&data);
        }
    }
}
