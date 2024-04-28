/*
 * radio.h
 *
 *  Created on: Apr 28, 2024
 *      Author: kamepiula
 */

#ifndef RADIO_H_
#define RADIO_H_


#include <nRF/nrf24_lower_api.h>
#include <Shift_Register/shift_reg.h>


typedef struct radio_task_t
{
	nrf24_lower_api_config_t radio;
	nrf24_spi_pins_sr_t nrf_pins_sr;
} radio_task_t;


void radio_task_init(radio_task_t * task, shift_reg_t * radio_sr);
void radio_dump_regs(radio_task_t * task);

#endif /* RADIO_H_ */
