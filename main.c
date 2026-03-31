#include "mcu_regs.h"
#include "type.h"
#include "uart.h"
#include "stdio.h"
#include "timer32.h"
#include "i2c.h"
#include "gpio.h"
#include "ssp.h"
#include "adc.h"


#include "light.h"
#include "oled.h"
#include "temp.h"
#include "joystick.h"
#include "htu21d_i2c.h"
#include "htu21d_i2c_hal.h"


static uint32_t msTicks = 0;
static uint8_t buf[10];
static uint8_t bufK[10];
static uint8_t buf_humidity[20];

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
}

static uint32_t getTicks(void)
{
    return msTicks;
}

uint8_t checkNav(int8_t idx){
	 uint8_t move = joystick_read();

	 if (move == JOYSTICK_RIGHT){
		 idx++;
	 }

	 if (move == JOYSTICK_LEFT){
		 idx--;
	 }

	 return idx;
}

void renderMainScreen(void)
{
    oled_clearScreen(OLED_COLOR_BLACK);

    oled_putString(0, 0,  (uint8_t*)"STACJA", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 10, (uint8_t*)"POGODOWA", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    oled_putString(0, 25, (uint8_t*)"Jakub M", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 35, (uint8_t*)"Kacper A", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 45, (uint8_t*)"Jakub S", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    oled_putString(0, 55, (uint8_t*)"< > NAV", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

}

void renderTemperatureScreen(int32_t temperature) {

	oled_putString(0, 0,  (uint8_t*)"TEMPERATURA", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	oled_putString(1, 10, "Celc: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	sprintf(buf,"%2d.%dC",temperature/10, temperature%10 );

	oled_putString(40, 10, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	int32_t tempK = temperature + 2731;

	oled_putString(1, 25, (uint8_t*)"Kelv:", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	sprintf((char*)bufK,"%d.%dK",
		tempK/10,
		abs(tempK%10));

	oled_putString(40, 25, bufK, OLED_COLOR_WHITE, OLED_COLOR_BLACK);


	oled_putString(0, 55, (uint8_t*)"< > NAV", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

}

void renderHumidityScreen(int32_t humidity) {
	oled_putString(0, 0, "WILGOTNOSC", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	oled_putString(0, 0, humidity, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

int main (void)
{
	int8_t x = 0;
	int8_t y = 0;
	int8_t z = 0;

    int8_t index = 0;
    int8_t lastIndex = -1;

    int32_t t = 0;

    int32_t hum = 0;



    GPIOInit();
    init_timer32(0, 10);

    UARTInit(115200);
    UARTSendString((uint8_t*)"OLED - Peripherals\r\n");

    SSPInit();
    ADCInit( ADC_CLK );

    oled_init();
    light_init();
    acc_init();

    temp_init (&getTicks);

    htu21d_i2c_hal_init();

    htu21d_i2c_reset();
    htu21d_i2c_hal_ms_delay(50);

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
    acc_read(&x, &y, &z);

    light_enable();
    light_setRange(LIGHT_RANGE_4000);

    oled_clearScreen(OLED_COLOR_BLACK);

    while(1) {
        index = checkNav(index);

        if (index != lastIndex) {
            oled_clearScreen(OLED_COLOR_BLACK);

            if (index == 0) {
                renderMainScreen();
            }

            if (index == 1) {
                renderTemperatureScreen(t);
            }

            if (index == 2) {
            	renderHumidityScreen(hum);
            }

            lastIndex = index;
        }

        if (index == 1) {
            t = temp_read();
            renderTemperatureScreen(t);
        }

        if (index == 2) {
        	hum = htu21d_i2c_hum_read(&hum);
        	renderHumidityScreen(hum);
        }


        delay32Ms(0, 100);
    }


}
