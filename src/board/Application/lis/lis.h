#ifndef LIS_H_
#define LIS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>



void lis_spi_cs_down(shift_reg_t *spi_sr_bus, uint8_t pin);
void lis_spi_cs_up(shift_reg_t *spi_sr_bus, uint8_t pin);

int32_t lis_spi_read(void * intf_ptr, uint8_t reg_addr, uint8_t * data, uint16_t data_len);
int32_t lis_spi_write(void * intf_ptr, uint8_t reg_addr, const uint8_t * data, uint16_t data_len);

#endif
