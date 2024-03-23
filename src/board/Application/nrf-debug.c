#include <stdint.h>
#include <stdio.h>

#include "nRF/nrf24_lower_api.h"
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


void nrf_dump_regs(nrf24_lower_api_config_t * lower)
{
	for (int i = 0; i < sizeof(reg_defs)/sizeof(reg_defs[0]); i++)
		print_reg(lower, reg_defs[i].reg_name, reg_defs[i].reg_addr, reg_defs[i].reg_size);

}
