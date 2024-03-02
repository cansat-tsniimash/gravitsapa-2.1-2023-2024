#include <stdio.h>
#include "includes.h"
#include <stm32f4xx_hal.h>


void lis_spi_cs_down(shift_reg_t *spi_sr_bus, uint8_t pin)
{

	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, false);

	shift_reg_oe(spi_sr_bus, false);
}

void lis_spi_cs_up(shift_reg_t *spi_sr_bus, uint8_t pin)
{
	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, true);

	shift_reg_oe(spi_sr_bus, false);
}

int32_t lis_spi_read(void * intf_ptr, uint8_t reg_addr, uint8_t * data, uint16_t data_len)
{
	struct spi_sr_bus * lis_spi_ptr = intf_ptr;

	lis_spi_cs_down(lis_spi_ptr->sr, lis_spi_ptr->sr_pin);

	reg_addr=reg_addr|(1<<7);
	reg_addr=reg_addr|(1<<6);
	HAL_SPI_Transmit(lis_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(lis_spi_ptr->spi, data, data_len, HAL_MAX_DELAY);

	lis_spi_cs_up(lis_spi_ptr->sr, lis_spi_ptr->sr_pin);

	return 0;
}

int32_t lis_spi_write(void * intf_ptr, uint8_t reg_addr, const uint8_t * data, uint16_t data_len)
{
	struct spi_sr_bus * lis_spi_ptr = intf_ptr;
	lis_spi_cs_down(lis_spi_ptr->sr, lis_spi_ptr->sr_pin);
	reg_addr=reg_addr&~(1<<7);
	reg_addr=reg_addr|(1<<6);
	HAL_SPI_Transmit(lis_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(lis_spi_ptr->spi, (uint8_t*)data, data_len, HAL_MAX_DELAY);
	lis_spi_cs_up(lis_spi_ptr->sr, lis_spi_ptr->sr_pin);

	return 0;
}
