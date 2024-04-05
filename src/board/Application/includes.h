#ifndef INCLUDES_H_
#define INCLUDES_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include <fatfs.h>
#include <Shift_Register/shift_reg.h>
#include <ATGM336H/nmea_gps.h>
#include <DS18B20/one_wire.h>
#include "nRF/nrf24_upper_api.h"
#include "nRF/nrf24_lower_api.h"
#include "nRF/nrf24_lower_api_stm32.h"
#include "nRF/nrf24_defs.h"
#include "BME280/bme280_defs.h"
#include "bme/bme.h"
#include "lsm/lsm.h"
#include "lis/lis.h"
#include "structs.h"
#include "csv_file.h"

struct spi_sr_bus
{
	int sr_pin;
	SPI_HandleTypeDef* spi;
	//Shift reg device
	shift_reg_t *sr;
};

#endif /* INCLUDES_H_ */
