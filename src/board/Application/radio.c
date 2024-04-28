/*
 * radio.c
 *
 *  Created on: Apr 28, 2024
 *      Author: kamepiula
 */

#include "radio.h"

#include <stdio.h>
#include <string.h>

#include "nRF/nrf24_upper_api.h"
#include "nRF/nrf24_defs.h"


struct reg_def{
	const char * reg_name;
	int reg_addr;
	int reg_size;
};


#define MAKE_REG_DEF(value, size) { #value, value, size}


static struct reg_def reg_defs[] = {
	MAKE_REG_DEF(NRF24_REGADDR_CONFIG, 1),
	MAKE_REG_DEF(NRF24_REGADDR_EN_AA, 1),
	MAKE_REG_DEF(NRF24_REGADDR_EN_RXADDR, 1),
	MAKE_REG_DEF(NRF24_REGADDR_SETUP_AW, 1),
	MAKE_REG_DEF(NRF24_REGADDR_SETUP_RETR, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RF_CH, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RF_SETUP, 1),
	MAKE_REG_DEF(NRF24_REGADDR_STATUS, 1),
	MAKE_REG_DEF(NRF24_REGADDR_OBSERVE_TX, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RPD, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RX_ADDR_P0, 5),
	MAKE_REG_DEF(NRF24_REGADDR_RX_ADDR_P1, 5),
	MAKE_REG_DEF(NRF24_REGADDR_RX_ADDR_P2, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RX_ADDR_P3, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RX_ADDR_P4, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RX_ADDR_P5, 1),
	MAKE_REG_DEF(NRF24_REGADDR_TX_ADDR, 5),
	MAKE_REG_DEF(NRF24_REGADDR_RX_PW_P0, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RX_PW_P1, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RX_PW_P2, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RX_PW_P3, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RX_PW_P4, 1),
	MAKE_REG_DEF(NRF24_REGADDR_RX_PW_P5, 1),
	MAKE_REG_DEF(NRF24_REGADDR_FIFO_STATUS, 1),
	MAKE_REG_DEF(NRF24_REGADDR_DYNPD, 1),
	MAKE_REG_DEF(NRF24_REGADDR_FEATURE, 1),
};

static void print_reg(nrf24_lower_api_config_t * lower, const char * name, int addr, int size)
{
	uint8_t data[5] = {};
	nrf24_read_register(lower, addr, data, size);

	printf("%s: ", name);
	printf("0x");
	for (int i = 0; i < size; i++)
		printf("%02X", (int)data[i]);

	if (1 == size)
	{
		printf(" (0b");
		for (int i = 0; i < 8; i++)
		{
			int bit = ((int)data[0] << i) & 0x80;
			if (bit)
				printf("1");
			else
				printf("0");
		}
		printf(")");
	}

	printf("\n");
}


void radio_dump_regs(radio_task_t * task)
{
	for (int i = 0; i < sizeof(reg_defs)/sizeof(reg_defs[0]); i++)
		print_reg(&task->radio, reg_defs[i].reg_name, reg_defs[i].reg_addr, reg_defs[i].reg_size);
}


void radio_task_init(radio_task_t * task, shift_reg_t * radio_sr)
{
	extern SPI_HandleTypeDef hspi2;
	memset(task, 0x00, sizeof(*task));

	task->nrf_pins_sr.this = radio_sr;
	task->nrf_pins_sr.pos_CE = 0;
	task->nrf_pins_sr.pos_CS = 1;
	nrf24_lower_api_config_t * nrf24 = &task->radio;
	nrf24_spi_init_sr(nrf24, &hspi2, &task->nrf_pins_sr);
	nrf24_mode_standby(nrf24);

	nrf24_rf_config_t nrf_config;
	nrf_config.data_rate = NRF24_DATARATE_250_KBIT;
	nrf_config.tx_power = NRF24_TXPOWER_MINUS_0_DBM;
	nrf_config.rf_channel = 88;
	nrf24_setup_rf(nrf24, &nrf_config);

	nrf24_protocol_config_t nrf_protocol_config;
	nrf_protocol_config.crc_size = NRF24_CRCSIZE_1BYTE;
	nrf_protocol_config.address_width = NRF24_ADDRES_WIDTH_5_BYTES;
	nrf_protocol_config.en_dyn_payload_size = true;
	nrf_protocol_config.en_ack_payload = true;
	nrf_protocol_config.en_dyn_ack = true;
	nrf_protocol_config.auto_retransmit_count = 0;
	nrf_protocol_config.auto_retransmit_delay = 0;
	nrf24_setup_protocol(nrf24, &nrf_protocol_config);
	nrf24_pipe_set_tx_addr(nrf24, 0x123456789a);

	nrf24_pipe_config_t pipe_config;
	for (int i = 1; i < 6; i++)
	{
		pipe_config.address = 0xcfcfcfcfcf;
		pipe_config.address = (pipe_config.address & ~((uint64_t)0xff << 32)) | ((uint64_t)(i + 7) << 32);
		pipe_config.enable_auto_ack = true;
		pipe_config.payload_size = -1;
		nrf24_pipe_rx_start(nrf24, i, &pipe_config);
	}

	pipe_config.address = 0xafafafaf01;
	pipe_config.enable_auto_ack = false;
	pipe_config.payload_size = -1;
	nrf24_pipe_rx_start(nrf24, 0, &pipe_config);

	nrf24_irq_mask_set(nrf24, NRF24_IRQ_TX_DR | NRF24_IRQ_MAX_RT);

	nrf24_mode_standby(nrf24);
	nrf24_mode_tx(nrf24);
}
