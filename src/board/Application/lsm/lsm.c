#include <stdio.h>
#include <includes.h>
#include <stm32f4xx_hal.h>


void lsm_spi_cs_down(shift_reg_t *spi_sr_bus, uint8_t pin)
{

	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, false);

	shift_reg_oe(spi_sr_bus, false);
}

void lsm_spi_cs_up(shift_reg_t *spi_sr_bus, uint8_t pin)
{
	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, true);

	shift_reg_oe(spi_sr_bus, false);
}

int32_t lsm_spi_read(void * intf_ptr, uint8_t reg_addr, uint8_t * data, uint16_t data_len)
{
	struct spi_sr_bus * lsm_spi_ptr = intf_ptr;

	lsm_spi_cs_down(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);

	reg_addr=reg_addr|(1<<7);
	HAL_SPI_Transmit(lsm_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(lsm_spi_ptr->spi, data, data_len, HAL_MAX_DELAY);

	lsm_spi_cs_up(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);

	return 0;
}

int32_t lsm_spi_write(void * intf_ptr, uint8_t reg_addr, const uint8_t * data, uint16_t data_len)
{
	struct spi_sr_bus * lsm_spi_ptr = intf_ptr;
	lsm_spi_cs_down(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);
	reg_addr=reg_addr&~(1<<7);
	HAL_SPI_Transmit(lsm_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(lsm_spi_ptr->spi, (uint8_t*)data, data_len, HAL_MAX_DELAY);
	lsm_spi_cs_up(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);

	return 0;
}
