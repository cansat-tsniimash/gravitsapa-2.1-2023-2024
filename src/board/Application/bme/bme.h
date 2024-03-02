#ifndef BME_H_
#define BME_H_

#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>


void bme_spi_cs_down(shift_reg_t *spi_sr_bus, uint8_t pin);
void bme_spi_cs_up(shift_reg_t *spi_sr_bus, uint8_t pin);

BME280_INTF_RET_TYPE bme_spi_read(uint8_t reg_addr, uint8_t * data, uint32_t data_len, void *intf_ptr);
BME280_INTF_RET_TYPE bme_spi_write(uint8_t reg_addr, const uint8_t * data, uint32_t data_len, void *intf_ptr);

void bme_delay_us(uint32_t period, void *intf_ptr);

#endif

