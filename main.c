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

#include "light.h"
#include "oled.h"
#include "temp.h"
//#include "acc.h"
#include "ff.h"
#include "hmc5883l_raw.h"

#define HTU21D_I2C_ADDRESS (0x40<<1)
#define CMD_TRIG_HUMD_NOHOLD 0xF5

#define BMP180_ADDRESS (0x77<<1)
#define OSS 3
#define SGP40_ADDR 0x59
#define SGP40_CMD_MSB 0x26
#define SGP40_CMD_LSB 0x0F

#define HMC_ADDR (0x1E << 1)

static uint32_t msTicks = 0;
static uint8_t buf[20];

#define P1_2_HIGH() (LPC_GPIO1->DATA |= (0x1<<2))
#define P1_2_LOW()  (LPC_GPIO1->DATA &= ~(0x1<<2))

#define IM_G4  3189   // G4  392Hz
#define IM_E4  3030   // E4  330Hz  (tu: Eb4 ~311Hz, ale E4 brzmi ok)
#define IM_C4  3816   // C4  262Hz
#define IM_D4  2840   // D4  352Hz  (Db4)
#define IM_A3  4545   // A3  220Hz
#define IM_F4  2865   // F4  349Hz
#define IM_BB3 3584   // Bb3 279Hz
#define IM_AB3 3795   // Ab3 263Hz
#define IM_REST 0

typedef struct {
    uint32_t freq;   // okres µs
    uint32_t dur;    // czas ms
} ImNote;

static const ImNote imperial[] = {
    // "DUM DUM DUM"
    {IM_G4, 3000}, {IM_REST, 600},
    {IM_G4, 3000}, {IM_REST, 600},
    {IM_G4, 3000}, {IM_REST, 600},

    // "DUM di-dum"
    {IM_E4, 2240}, {IM_REST, 300},
    {IM_BB3, 760}, {IM_REST, 300},
    {IM_G4, 3600}, {IM_REST, 900},

    // "DUM di-dum"
    {IM_E4, 2240}, {IM_REST, 300},
    {IM_BB3, 760}, {IM_REST, 300},
    {IM_G4, 5400}, {IM_REST, 1200},

    // drugi motyw "DUM DUM DUM"
    {IM_D4, 3000}, {IM_REST, 600},
    {IM_D4, 3000}, {IM_REST, 600},
    {IM_D4, 3000}, {IM_REST, 600},

    // "DUM di-dum"
    {IM_F4, 2240}, {IM_REST, 300},
    {IM_BB3, 760}, {IM_REST, 300},
    {IM_AB3, 3600},{IM_REST, 900},

    // "DUM di-dum"
    {IM_E4, 2240}, {IM_REST, 300},
    {IM_BB3, 760}, {IM_REST, 300},
    {IM_G4, 5400}, {IM_REST, 1200},
};

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

static uint32_t getTicks(void)
{
    return msTicks;
}

uint32_t read_humidity(void) {
	uint8_t cmd = CMD_TRIG_HUMD_NOHOLD;
	uint8_t rx_data[2] = {0};


	I2CWrite(HTU21D_I2C_ADDRESS, &cmd, 1);
	delay32Ms(0, 15);

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
    delay32Ms(0, 5);
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
    //I2CStop();
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
    //I2CStop();

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

void SysTick_Handler(void) {
    msTicks++;
}

static void playNote(uint32_t note, uint32_t durationMs) {
    uint32_t t = 0;
    if (note > 0) {
    	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        while (t < (durationMs * 1000)) {
            P1_2_HIGH();
            // prosta pętla zamiast delay32Us
            volatile uint32_t i;
            for(i = 0; i < note/4; i++) __NOP();

            P1_2_LOW();
            for(i = 0; i < note/4; i++) __NOP();

            t += note;
        }
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    } else {
        delay32Ms(0, durationMs);
    }
}

static void playImperialMarch(void) {
    uint32_t count = sizeof(imperial) / sizeof(ImNote);
    for (uint32_t i = 0; i < count; i++) {
        playNote(imperial[i].freq, imperial[i].dur);
    }
}

uint16_t read_airquality_raw(){
	uint8_t cmd[5] = {SGP40_CMD_MSB, SGP40_CMD_LSB, 0x80, 0x00, 0xA2};
	uint8_t buf[3];

	I2CWrite(SGP40_ADDR, cmd, 5);
	delay32Ms(0, 32);
	I2CRead(SGP40_ADDR, buf, 3);

	return ((uint16_t)buf[0] << 8) | buf[1];
}



//static void init_pwm(void)
//{
//  LPC_PWM1->MR0 = 1000; // okres pwm
//  LPC_PWM1->LER |= (1U << 0U); // zatwierdzenie MR0
//  // rejestry MR są 32-bitowe
//
//  // bit 0 - wlaczenie glownego licznika i prescalera
//  // bit 3 - pwm enable
//  LPC_PWM1->TCR |= (1U << 0U) | (1U << 3U);
//
//  PINSEL_CFG_Type PinCfg;
//  PinCfg.Portnum = 2;
//  PinCfg.Pinmode = 0;
//  PinCfg.Funcnum = 1;
//  PinCfg.OpenDrain = 0;
//
//  // PIO1_9
//  PinCfg.Pinnum = 0;
//  PINSEL_ConfigPin(&PinCfg);
//  LPC_PWM1->MR1 = 500; // 50%
//  LPC_PWM1->LER |= (1U << 1U); // zatwierdzenie rejestru MR1
//  LPC_PWM1->PCR |= (1U << (9U + 0U)); // aktywacja wyjscia sygnalu dla kanalu 2
//
//  // PIO2_3
//  PinCfg.Pinnum = 3;
//  PINSEL_ConfigPin(&PinCfg);
//  LPC_PWM1->MR4 = 500; // 50%
//  LPC_PWM1->LER |= (1U << 4U); // zatwierdzenie MR4
//  LPC_PWM1->PCR |= (1U << (9U + 3U));
//}
//
//static void my_set_pwm_value(int channel, int value)
//{
//  if (channel == 1)
//  {
//    LPC_PWM1->MR1 = value;
//    LPC_PWM1->LER |= (1U << 1U);
//  }
//  else if (channel == 2)
//  {
//    LPC_PWM1->MR4 = value;
//    LPC_PWM1->LER |= (1U << 4U);
//  }
//}
//

int main (void)
{
	LPC_IOCON->JTAG_nTRST_PIO1_2 = (LPC_IOCON->JTAG_nTRST_PIO1_2 & ~0x7) | 0x01;
	GPIOSetDir(PORT1, 2, 1);

	// wzmacniacz LM4811 - wybudź
	GPIOSetDir(PORT3, 0, 1);  // clk
	GPIOSetDir(PORT3, 1, 1);  // up/dn
	GPIOSetDir(PORT3, 2, 1);  // shutdn
	GPIOSetValue(PORT3, 0, 0);
	GPIOSetValue(PORT3, 1, 0);
	GPIOSetValue(PORT3, 2, 0);


	int32_t ticks = 0;

	uint16_t raw_air_quality;
    int32_t t = 0;
    int32_t tempK = 0;
    int32_t humidity = 0;
    int32_t light = 0;
    int32_t pressure = 0;
    int16_t mag[3];
    int16_t x_gauss, y_gauss, z_gauss;
    int32_t b_raw;
    int32_t b;

    GPIOInit();

    UARTInit(115200);
    UARTSendString((uint8_t*)"OLED - Peripherals\r\n");

    I2CInit( (uint32_t)I2CMASTER, 0 );
    SSPInit();
    GPIOSetDir(PORT0, 2, 1);
    GPIOSetValue(PORT0, 2, 1);

    //delay32Ms(0, 10);

    SysTick_Config(SystemCoreClock / 1000);

    ADCInit( ADC_CLK );

    oled_init();
    light_init();
    acc_init();
    temp_init (&getTicks);

    bmp180_read_calibration();

//    init_pwm();
//
//    my_set_pwm_value(1, 500);

    HMC_Init();
    delay32Ms(0, 10);

    /* setup sys Tick. Elapsed time is e.g. needed by temperature sensor */
    SysTick_Config(SystemCoreClock / 1000);
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
    oled_putString(1,10,  (uint8_t*)"Raw VOC: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(1,20,  (uint8_t*)"Humi(%): ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(1,30, (uint8_t*)"Lux(lx): ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(1,40, (uint8_t*)"Press: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    uint8_t alarm_played = 0;

    while(1) {
    	UARTSendString((uint8_t*)"ping\r\n");
    	delay32Ms(0, 1000);
    	/* Light sensor*/
    	light = light_read();

        /* Temperature */
        t = temp_read();

        raw_air_quality = read_airquality_raw();

        /* Humidity */
        humidity = read_humidity();

        /* Pressure */
        pressure = bmp180_read_pressure_pa();

        HMC_ReadXYZ(mag);

        int32_t mx = mag[0];
        int32_t my = mag[1];
        int32_t mz = mag[2];

        b_raw = (int32_t)sqrt((double)(mx*mx + my*my + mz*mz));
        b = (b_raw * 1000) / 1090;

        /* output values to OLED display */
        sprintf(buf,"%2d.%dC",t/10, t%10 );
        oled_putString((1+9*6),1, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        tempK = (t/10) + 273;
        //sprintf(buf,"%3dK", tempK);
        //oled_putString((1 + 9*6), 10, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
//        sprintf(buf, "%d %d %d", mag[0], mag[1], mag[2]);
//        oled_putString(10, 10, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(buf,"%d", raw_air_quality);
        oled_putString((1 + 9*6), 10, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(buf, "%d.%d%%", humidity/10, humidity%10);
        oled_putString((1 + 9*6), 20, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        if (light < 200 && !alarm_played) {
        	alarm_played = 1;
        	playImperialMarch();
        } else {
        	alarm_played = 0;
        }

        sprintf(buf, "%d", light);
        oled_putString((1 + 9*6), 30, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(buf, "%d.%d", pressure/100, pressure%100);
        oled_putString((1 + 9*6), 40, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(buf, "%d.%03d", b/1000, b%1000);
        oled_putString((1 + 9*6), 50, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        /* delay */
        delay32Ms(0, 15);
    }

}
