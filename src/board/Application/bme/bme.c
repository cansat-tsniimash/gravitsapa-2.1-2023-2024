#include <stdio.h>
#include "includes.h"
#include <stm32f4xx_hal.h>




void bme_spi_cs_down(shift_reg_t *spi_sr_bus, uint8_t pin)
{

	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, false);

	shift_reg_oe(spi_sr_bus, false);
}


void bme_spi_cs_up(shift_reg_t *spi_sr_bus, uint8_t pin)
{
	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, true);

	shift_reg_oe(spi_sr_bus, false);
}


BME280_INTF_RET_TYPE bme_spi_read(uint8_t reg_addr, uint8_t * data, uint32_t data_len, void *intf_ptr)
{
	struct spi_sr_bus * bme_spi_ptr = intf_ptr;

	bme_spi_cs_down(bme_spi_ptr->sr, bme_spi_ptr->sr_pin);

	reg_addr |= (1 << 7);
	HAL_SPI_Transmit(bme_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(bme_spi_ptr->spi, data, data_len, HAL_MAX_DELAY);

	bme_spi_cs_up(bme_spi_ptr->sr, bme_spi_ptr->sr_pin);

	return 0;
}


BME280_INTF_RET_TYPE bme_spi_write(uint8_t reg_addr, const uint8_t * data, uint32_t data_len, void *intf_ptr)
{
	struct spi_sr_bus * bme_spi_ptr = intf_ptr;
	bme_spi_cs_down(bme_spi_ptr->sr, bme_spi_ptr->sr_pin);
	reg_addr &= ~(1 << 7);
	HAL_SPI_Transmit(bme_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(bme_spi_ptr->spi, (uint8_t*)data, data_len, HAL_MAX_DELAY);
	bme_spi_cs_up(bme_spi_ptr->sr, bme_spi_ptr->sr_pin);

	return 0;
}


void bme_delay_us(uint32_t period, void *intf_ptr)
{
	if (period < 1000)
		period = 1;
	else
		period = period / 1000;

	HAL_Delay(period);
}
