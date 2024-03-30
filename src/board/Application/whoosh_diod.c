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

int whoosh_diod(void)
{
	extern SPI_HandleTypeDef hspi1;

	//Настройка SR
	shift_reg_t sr_sensor = {};
	sr_sensor.bus = &hspi1;
	sr_sensor.latch_port = GPIOA;
	sr_sensor.latch_pin  = GPIO_PIN_4;
	//sr_sensor.oe_port = GPIOB;
	//sr_sensor.oe_pin = GPIO_PIN_4;
	shift_reg_init(&sr_sensor);
	shift_reg_write_16(&sr_sensor, 0xffff);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_Delay(30);
	shift_reg_write_8(&sr_sensor, 0xFF);
	HAL_Delay(30);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);


	while(1)
	{

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_Delay(100);

	}
	return 0;

}
