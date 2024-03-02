#ifndef LSM_H_
#define LSM_H_

#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>


void lsm_spi_cs_down(shift_reg_t *spi_sr_bus, uint8_t pin);
void lsm_spi_cs_up(shift_reg_t *spi_sr_bus, uint8_t pin);

int32_t lsm_spi_read(void * intf_ptr, uint8_t reg_addr, uint8_t * data, uint16_t data_len);
int32_t lsm_spi_write(void * intf_ptr, uint8_t reg_addr, const uint8_t * data, uint16_t data_len);

#endif
