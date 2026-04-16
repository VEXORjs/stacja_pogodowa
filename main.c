/*****************************************************************************
 *   Peripherals such as temp sensor, light sensor, accelerometer,
 *   and trim potentiometer are monitored and values are written to
 *   the OLED display.
 *
 *   Copyright(C) 2009, Embedded Artists AB
 *   All rights reserved.
 *
 ******************************************************************************/
/* clock wchodzi do pinu nr 4
 * data wchodzi do pinu nr 5*/

#include "mcu_regs.h"
#include "type.h"
#include "uart.h"
#include "stdio.h"
#include "timer32.h"
#include "i2c.h"
#include "gpio.h"
#include "ssp.h"
#include "adc.h"
#include "math.h"

#include "flash.h"
#include "ff.h"
#include "ffconf.h"
#include "diskio.h"
#include "light.h"
#include "oled.h"
#include "temp.h"
//#include "acc.h"

#define HTU21D_I2C_ADDRESS (0x40<<1)
#define CMD_TRIG_HUMD_NOHOLD 0xF5

#define BMP180_ADDRESS (0x77<<1)
#define OSS 3

#define SD_CARD 0

static uint32_t msTicks = 0;
static uint8_t buf[10];

static void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base)
{
    static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
    int pos = 0;
    int tmpValue = value;

    // the buffer must not be null and at least have a length of 2 to handle one
    // digit and null-terminator
    if (pBuf == NULL || len < 2)
    {
        return;
    }

    // a valid base cannot be less than 2 or larger than 36
    // a base value of 2 means binary representation. A value of 1 would mean only zeros
    // a base larger than 36 can only be used if a larger alphabet were used.
    if (base < 2 || base > 36)
    {
        return;
    }

    // negative value
    if (value < 0)
    {
        tmpValue = -tmpValue;
        value    = -value;
        pBuf[pos++] = '-';
    }

    // calculate the required length of the buffer
    do {
        pos++;
        tmpValue /= base;
    } while(tmpValue > 0);


    if (pos > len)
    {
        // the len parameter is invalid.
        return;
    }

    pBuf[pos] = '\0';

    do {
        pBuf[--pos] = pAscii[value % base];
        value /= base;
    } while(value > 0);

    return;

}

void SysTick_Handler(void) {
    msTicks++;
    disk_timerproc();
}

static uint32_t getTicks(void)
{
    return msTicks;
}

uint32_t read_humidity(void) {
	uint8_t cmd = CMD_TRIG_HUMD_NOHOLD;
	uint8_t rx_data[2] = {0};

	I2CWrite(HTU21D_I2C_ADDRESS, &cmd, 1);
	delay32Ms(0, 25);

	I2CRead(HTU21D_I2C_ADDRESS, rx_data, 2);
	uint16_t raw_humidity = (rx_data[0] << 8 | rx_data[1]);
	raw_humidity &= 0xFFFC;
	int32_t humidity_x10 = -60 + (1250 * (int32_t)raw_humidity / 65536);

	return humidity_x10;
}

typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
} BMP180_calib_t;

BMP180_calib_t cal;

/* read calibration */
void bmp180_read_calibration(void)
{
    uint8_t reg = 0xAA;
    uint8_t b[22];

    I2CWrite(BMP180_ADDRESS, &reg, 1);
    I2CRead(BMP180_ADDRESS, b, 22);

    cal.AC1 = (b[0]<<8) | b[1];
    cal.AC2 = (b[2]<<8) | b[3];
    cal.AC3 = (b[4]<<8) | b[5];
    cal.AC4 = (b[6]<<8) | b[7];
    cal.AC5 = (b[8]<<8) | b[9];
    cal.AC6 = (b[10]<<8) | b[11];
    cal.B1  = (b[12]<<8) | b[13];
    cal.B2  = (b[14]<<8) | b[15];
    cal.MB  = (b[16]<<8) | b[17];
    cal.MC  = (b[18]<<8) | b[19];
    cal.MD  = (b[20]<<8) | b[21];
}

/* UT */
uint32_t bmp180_read_ut(void)
{
    uint8_t cmd[2] = {0xF4, 0x2E};
    uint8_t data[2];

    I2CWrite(BMP180_ADDRESS, cmd, 2);
    delay32Ms(0, 5);

    uint8_t reg = 0xF6;
    I2CWrite(BMP180_ADDRESS, &reg, 1);
    I2CRead(BMP180_ADDRESS, data, 2);

    return ((uint32_t)data[0]<<8) | data[1];
}

/* UP (FIXED) */
uint32_t bmp180_read_up(void)
{
    uint8_t cmd[2];
    uint8_t data[3];

    cmd[0] = 0xF4;
    cmd[1] = 0x34 + (OSS<<6);

    I2CWrite(BMP180_ADDRESS, cmd, 2);

    switch(OSS)
    {
        case 0: delay32Ms(0,5); break;
        case 1: delay32Ms(0,8); break;
        case 2: delay32Ms(0,14); break;
        case 3: delay32Ms(0,26); break;
    }

    uint8_t reg = 0xF6;
    I2CWrite(BMP180_ADDRESS, &reg, 1);
    I2CRead(BMP180_ADDRESS, data, 3);

    uint32_t up = (((uint32_t)data[0]<<16) |
                   ((uint32_t)data[1]<<8)  |
                   ((uint32_t)data[2]));

    up >>= (8 - OSS);
    return up;
}

/* FULL COMPENSATION (LibDriver style) */
int32_t bmp180_get_pressure_pa_fixed(uint32_t ut, uint32_t up)
{
    int32_t x1, x2, b5, b6, x3, b3, p;
    uint32_t b4, b7;

    x1 = ((ut - cal.AC6) * cal.AC5) >> 15;
    x2 = ((int32_t)cal.MC << 11) / (x1 + cal.MD);
    b5 = x1 + x2;

    b6 = b5 - 4000;

    x1 = (cal.B2 * ((b6*b6)>>12)) >> 11;
    x2 = (cal.AC2 * b6) >> 11;
    x3 = x1 + x2;

    b3 = ((((int32_t)cal.AC1 * 4 + x3) << OSS) + 2) >> 2;

    x1 = (cal.AC3 * b6) >> 13;
    x2 = (cal.B1 * ((b6*b6)>>12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;

    b4 = (cal.AC4 * (uint32_t)(x3 + 32768)) >> 15;
    b7 = (up - b3) * (50000 >> OSS);

    if (b7 < 0x80000000)
        p = (b7<<1) / b4;
    else
        p = (b7 / b4) << 1;

    x1 = (p>>8)*(p>>8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;

    return p + ((x1 + x2 + 3791) >> 4);
}

/* optional: one-call */
int32_t bmp180_read_pressure_pa(void)
{
    uint32_t ut = bmp180_read_ut();
    uint32_t up = bmp180_read_up();
    return bmp180_get_pressure_pa_fixed(ut, up);
}

FATFS fs;
FIL file;
UINT bw;
FRESULT res;

DWORD get_fattime(void)
{
    return ((DWORD)(2024 - 1980) << 25)
         | ((DWORD)1 << 21)
         | ((DWORD)1 << 16);
}

int main (void)
{

    int32_t t = 0;
    int32_t tempK = 0;
    int32_t humidity = 0;
    int32_t light = 0;
    int32_t pressure = 0;

    GPIOInit();
    init_timer32(0, 10);

    UARTInit(115200);
    UARTSendString((uint8_t*)"OLED - Peripherals\r\n");

    I2CInit( (uint32_t)I2CMASTER, 0 );
    SSPInit();
    SysTick_Config(SystemCoreClock / 1000);

    ADCInit( ADC_CLK );

    oled_init();
    light_init();
    acc_init();
    temp_init (&getTicks);

    bmp180_read_calibration();

    DSTATUS sd_status = disk_initialize(0);

    if (sd_status & STA_NOINIT) {
    	UARTSendString((uint8_t*) "SD init FAILED\r\n");
    }
    else {
    	UARTSendString((uint8_t*) "SD init OK\r\n");
    }

    if (f_mount(0, &fs) == FR_OK) {
    	UARTSendString((uint8_t*) "FAT mount OK\r\n");
    }
    else {
    	UARTSendString((uint8_t*) "FAT mount FAILED\r\n");
    }

    if (f_open(&file, "log.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
          f_write(&file, "Start log\r\n", 11, &bw);
          f_close(&file);
      }
    else {
    	UARTSendString((uint8_t*) "f_open FAILED\r\n");
    }

    res = f_open(&file, "log.txt", FA_OPEN_ALWAYS | FA_WRITE);

    /* setup sys Tick. Elapsed time is e.g. needed by temperature sensor */

    if ( !(SysTick->CTRL & (1<<SysTick_CTRL_CLKSOURCE_Msk)) )
    {
      /* When external reference clock is used(CLKSOURCE in
      Systick Control and register bit 2 is set to 0), the
      SYSTICKCLKDIV must be a non-zero value and 2.5 times
      faster than the reference clock.
      When core clock, or system AHB clock, is used(CLKSOURCE
      in Systick Control and register bit 2 is set to 1), the
      SYSTICKCLKDIV has no effect to the SYSTICK frequency. See
      more on Systick clock and status register in Cortex-M3
      technical Reference Manual. */
      LPC_SYSCON->SYSTICKCLKDIV = 0x08;
    }

    /*
     * Assume base board in zero-g position when reading first value.
     */

    light_enable();
    light_setRange(LIGHT_RANGE_4000);

    oled_clearScreen(OLED_COLOR_BLACK);

    oled_putString(1,1,  (uint8_t*)"Temp(C): ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(1,10,  (uint8_t*)"Temp(K): ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(1,20,  (uint8_t*)"Humi(%): ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(1,30, (uint8_t*)"Lux(lx): ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(1,40, (uint8_t*)"Press: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    while(1) {

    	/* Light sensor*/
    	light = light_read();

        /* Temperature */
        t = temp_read();

        /* Humidity */
        humidity = read_humidity();

        /* Pressure */
        pressure = bmp180_read_pressure_pa();

        /* output values to OLED display */
        sprintf(buf,"%2d.%dC",t/10, t%10 );
        oled_putString((1+9*6),1, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        tempK = (t/10) + 273;
        sprintf(buf,"%3dK", tempK);
        oled_putString((1 + 9*6), 10, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(buf, "%d.%d%%", humidity/10, humidity%10);
        oled_putString((1 + 9*6), 20, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(buf, "%d", light);
        oled_putString((1 + 9*6), 30, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(buf, "%d.%d", pressure/100, pressure%100);
        oled_putString((1 + 9*6), 40, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        if (res == FR_OK)
        {
            f_lseek(&file, file.fsize);

            char logbuf[64];

            sprintf(logbuf,
                    "T=%d.%dC H=%d.%d%% L=%d\r\n",
                    t / 10, t % 10,
                    humidity / 10, humidity % 10,
                    light);

            UINT bw;
            res = f_write(&file, logbuf, strlen(logbuf), &bw);

            f_sync(&file);
        }

        /* delay */
        delay32Ms(0, 25);
    }

    f_close(&file);
}
