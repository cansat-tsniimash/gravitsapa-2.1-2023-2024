#include <stdio.h>
#include <stm32f4xx_hal.h>

#include "Shift_Register/shift_reg.h"

extern SPI_HandleTypeDef hspi1;

struct spi_sr_bus
{
	int sr_pin;
	SPI_HandleTypeDef* spi;
	//Shift reg device
	shift_reg_t *sr;
};

//static const unsigned char diods = {0b10000000, 0b01000000, 0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010, 0b00000001};

int whoosh_diod(void)
{
	extern SPI_HandleTypeDef hspi1;


	//Настройка SR
	shift_reg_t sr_led = {};
	sr_led.bus = &hspi1;
	sr_led.latch_port = GPIOA;
	sr_led.latch_pin  = GPIO_PIN_4;
	sr_led.oe_port = GPIOA;
	sr_led.oe_pin = GPIO_PIN_4;
	sr_led.value = 0;
	shift_reg_init(&sr_led);




	return 0;

}
