#include <stdio.h>

#include <stm32f4xx_hal.h>
#include "Shift_Register/shift_reg.h"
#include "LSM6DS3/lsm6ds3_reg.h"
#include "BME280/bme280.h"
#include "LIS3MDL/lis3mdl_reg.h"
#include "DS18B20/one_wire.h"
#include "timers.h"
#include "includes.h"

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;







struct spi_sr_bus
{
	int sr_pin;
	SPI_HandleTypeDef* spi;
	//Shift reg device
	shift_reg_t *sr;
};

static void bme_spi_cs_down(shift_reg_t *spi_sr_bus, uint8_t pin)
{

	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, false);

	shift_reg_oe(spi_sr_bus, false);
}


static void bme_spi_cs_up(shift_reg_t *spi_sr_bus, uint8_t pin)
{
	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, true);

	shift_reg_oe(spi_sr_bus, false);
}


static BME280_INTF_RET_TYPE bme_spi_read(uint8_t reg_addr, uint8_t * data, uint32_t data_len, void *intf_ptr)
{
	struct spi_sr_bus * bme_spi_ptr = intf_ptr;

	bme_spi_cs_down(bme_spi_ptr->sr, bme_spi_ptr->sr_pin);

	reg_addr |= (1 << 7);
	HAL_SPI_Transmit(bme_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(bme_spi_ptr->spi, data, data_len, HAL_MAX_DELAY);

	bme_spi_cs_up(bme_spi_ptr->sr, bme_spi_ptr->sr_pin);

	return 0;
}


static BME280_INTF_RET_TYPE bme_spi_write(uint8_t reg_addr, const uint8_t * data, uint32_t data_len, void *intf_ptr)
{
	struct spi_sr_bus * bme_spi_ptr = intf_ptr;
	bme_spi_cs_down(bme_spi_ptr->sr, bme_spi_ptr->sr_pin);
	reg_addr &= ~(1 << 7);
	HAL_SPI_Transmit(bme_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(bme_spi_ptr->spi, (uint8_t*)data, data_len, HAL_MAX_DELAY);
	bme_spi_cs_up(bme_spi_ptr->sr, bme_spi_ptr->sr_pin);

	return 0;
}


static void bme_delay_us(uint32_t period, void *intf_ptr)
{
	if (period < 1000)
		period = 1;
	else
		period = period / 1000;

	HAL_Delay(period);
}




static void lsm_spi_cs_down(shift_reg_t *spi_sr_bus, uint8_t pin)
{

	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, false);

	shift_reg_oe(spi_sr_bus, false);
}

static void lsm_spi_cs_up(shift_reg_t *spi_sr_bus, uint8_t pin)
{
	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, true);

	shift_reg_oe(spi_sr_bus, false);
}

static int32_t lsm_spi_read(void * intf_ptr, uint8_t reg_addr, uint8_t * data, uint16_t data_len)
{
	struct spi_sr_bus * lsm_spi_ptr = intf_ptr;

	lsm_spi_cs_down(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);

	reg_addr=reg_addr|(1<<7);
	HAL_SPI_Transmit(lsm_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(lsm_spi_ptr->spi, data, data_len, HAL_MAX_DELAY);

	lsm_spi_cs_up(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);

	return 0;
}

static int32_t lsm_spi_write(void * intf_ptr, uint8_t reg_addr, const uint8_t * data, uint16_t data_len)
{
	struct spi_sr_bus * lsm_spi_ptr = intf_ptr;
	lsm_spi_cs_down(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);
	reg_addr=reg_addr&~(1<<7);
	HAL_SPI_Transmit(lsm_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(lsm_spi_ptr->spi, (uint8_t*)data, data_len, HAL_MAX_DELAY);
	lsm_spi_cs_up(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);

	return 0;
}


static void lis_spi_cs_down(shift_reg_t *spi_sr_bus, uint8_t pin)
{

	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, false);

	shift_reg_oe(spi_sr_bus, false);
}

static void lis_spi_cs_up(shift_reg_t *spi_sr_bus, uint8_t pin)
{
	shift_reg_oe(spi_sr_bus, true);

	shift_reg_write_bit_16(spi_sr_bus, pin, true);

	shift_reg_oe(spi_sr_bus, false);
}

static int32_t lis_spi_read(void * intf_ptr, uint8_t reg_addr, uint8_t * data, uint16_t data_len)
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

static int32_t lis_spi_write(void * intf_ptr, uint8_t reg_addr, const uint8_t * data, uint16_t data_len)
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

//static void lsm_delay_us(uint32_t period, void *intf_ptr)
//{
//	if (period < 1000)
//		period = 1;
//	else
//		period = period / 1000;
//
//	HAL_Delay(period);
//}

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
  return 0;
}



int app_main(void)
{
	FATFS fileSystem;
	FIL File_bin;
	FIL File_csv;
	FRESULT res_bin = 255;
	FRESULT res_csv = 255;
	FRESULT megares = 255;

	const char path4[] = "packet_csv.csv";
	const char path_bin[] = "packet.bin";

	memset(&fileSystem, 0x00, sizeof(fileSystem));
	FRESULT is_mount = 0;
	extern Disk_drvTypeDef disk;
	disk.is_initialized[0] = 0;
	is_mount = f_mount(&fileSystem, "", 1);
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res1 = f_open(&File1, (char*)path1, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
		f_puts("flag; num; time_ms; accl1; accl2; accl3; gyro1; gyro2; gyro3; mag1; mag2; mag3; crc\n", &File1);
		res1 = f_sync(&File1);
	}

	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res_bin = f_open(&File_bin, (char*)path_bin, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
	}

	//переменные

	bool mount = false;

	float temperature_celsius_gyro = 0.0;
	float acc_g[3] = {0};
	float gyro_dps[3] = {0};
	float temperature_celsius_mag = 0.0;
	float mag[3] = {0};
	float lat;
	float lon;
	float alt;
	uint16_t temp_ds;
	bool crc_ok_ds = false;
	uint32_t start_time_ds = HAL_GetTick();
	uint32_t start_time_nrf = HAL_GetTick();
	uint32_t start_time_sd = HAL_GetTick();
	nrf24_fifo_status_t rx_status = NRF24_FIFO_EMPTY;
	nrf24_fifo_status_t tx_status = NRF24_FIFO_EMPTY;
	float limit_lux;
	int counter = 0;
	UINT Bytes;
	int comp = 0;
	int fast_count = 0;


	extern SPI_HandleTypeDef hspi2;

	//Настройка SR
	shift_reg_t sr_sensor = {};
	sr_sensor.bus = &hspi2;
	sr_sensor.latch_port = GPIOC;
	sr_sensor.latch_pin  = GPIO_PIN_1;
	sr_sensor.oe_port = GPIOC;
	sr_sensor.oe_pin = GPIO_PIN_13;
	shift_reg_init(&sr_sensor);
	shift_reg_write_16(&sr_sensor, 0xffff);


	shift_reg_t shift_reg_r = {};
	shift_reg_r.latch_port = GPIOC;
	shift_reg_r.latch_pin = GPIO_PIN_4;
	shift_reg_r.oe_port = GPIOC;
	shift_reg_r.oe_pin = GPIO_PIN_5;
	shift_reg_r.value = 0;
	shift_reg_init(&shift_reg_r);
	shift_reg_write_8(&shift_reg_r, 0xFF);

	//НАстройка DS18B20
	ds18b20_t ds;
	ds.onewire_port = GPIOA;
	ds.onewire_pin = GPIO_PIN_1;

	onewire_init(&ds);
	ds18b20_set_config(&ds, 100, -100, DS18B20_RESOLUTION_12_BIT);
	uint16_t temp_ds;
	uint32_t start_time_ds = HAL_GetTick();
	uint32_t start_time_nrf = HAL_GetTick();
	bool crc_ok_ds = false;
	float ds_temp;
	ds18b20_start_conversion(&ds);

	// Настройка bme280 =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	struct spi_sr_bus bme_spi_ptr = {};
	bme_spi_ptr.spi = &hspi2;
	bme_spi_ptr.sr = &sr_sensor;
	bme_spi_ptr.sr_pin = 2;

	struct bme280_dev bme = {};
	bme.intf = BME280_SPI_INTF;
	bme.intf_ptr = &bme_spi_ptr;
	bme.read = bme_spi_read;
	bme.write = bme_spi_write;
	bme.delay_us = bme_delay_us;

	int rc = bme280_soft_reset(&bme);
	printf("bme280 reset rc = %d\n", (int)rc);

	rc = bme280_init(&bme);
	printf("bme280 init rc = %d\n", (int)rc);

	bme.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme.settings.filter = BME280_FILTER_COEFF_16;
	bme.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;



	uint8_t settings_sel;
	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_FILTER_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	rc = bme280_set_sensor_settings(settings_sel, &bme);
	printf("bme280 settings set rc = %d\n", rc);
	rc = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme);
	printf("bme280 set sensor mode rc = %d\n", rc);


	// Настройка lsm6ds3 =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	struct spi_sr_bus lsm_spi_ptr = {};
	lsm_spi_ptr.spi = &hspi2;
	lsm_spi_ptr.sr = &sr_sensor;
	lsm_spi_ptr.sr_pin = 4;

	stmdev_ctx_t ctx = {0};
	ctx.handle = &lsm_spi_ptr;
	ctx.read_reg = lsm_spi_read;
	ctx.write_reg = lsm_spi_write;

	uint8_t whoami = 0x00;
	lsm6ds3_device_id_get(&ctx, &whoami);
//	printf("got lsm6ds3 whoami 0x%02X, expected 0x%02X\n", (int)whoami, (int)LSM6DS3_ID);

	lsm6ds3_reset_set(&ctx, PROPERTY_ENABLE);
	HAL_Delay(100);

	lsm6ds3_xl_full_scale_set(&ctx, LSM6DS3_16g);
	lsm6ds3_xl_data_rate_set(&ctx, LSM6DS3_XL_ODR_104Hz);

	lsm6ds3_gy_full_scale_set(&ctx, LSM6DS3_2000dps);
	lsm6ds3_gy_data_rate_set(&ctx, LSM6DS3_GY_ODR_104Hz);


	//Настройка lis3mdl =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	struct spi_sr_bus lis_spi_ptr = {};
	lis_spi_ptr.spi = &hspi2;
	lis_spi_ptr.sr = &sr_sensor;
	lis_spi_ptr.sr_pin = 3;
	stmdev_ctx_lis_t handle = {};
	handle.handle = &lis_spi_ptr;
	handle.read_reg = lis_spi_read;
	handle.write_reg = lis_spi_write;

	uint8_t whoami_lis = 0x00;
	lis3mdl_device_id_get(&handle, &whoami_lis);

	lis3mdl_reset_set(&handle, PROPERTY_ENABLE);
	HAL_Delay(100);

	lis3mdl_block_data_update_set(&handle, PROPERTY_ENABLE);
	lis3mdl_fast_low_power_set(&handle, PROPERTY_DISABLE);
	lis3mdl_full_scale_set(&handle, LIS3MDL_16_GAUSS);
	lis3mdl_data_rate_set(&handle, LIS3MDL_UHP_80Hz);
	lis3mdl_temperature_meas_set(&handle, PROPERTY_ENABLE);
	lis3mdl_operating_mode_set(&handle, LIS3MDL_CONTINUOUS_MODE);
	//Настройка GPS
	int64_t cookie;
	int fix_;
	float lon, lat, alt;
	uint64_t gps_buf;
	gps_init();
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
	uint64_t gps_time_s;
	uint32_t gps_time_us;





	nrf24_spi_pins_sr_t nrf_pins_sr;
	nrf_pins_sr.this = &shift_reg_r;
	nrf_pins_sr.pos_CE = 0;
	nrf_pins_sr.pos_CS = 1;
	nrf24_lower_api_config_t nrf24;
	nrf24_spi_init_sr(&nrf24, &hspi2, &nrf_pins_sr);

	nrf24_mode_power_down(&nrf24);
	nrf24_rf_config_t nrf_config;
	nrf_config.data_rate = NRF24_DATARATE_250_KBIT;
	nrf_config.tx_power = NRF24_TXPOWER_MINUS_0_DBM;
	nrf_config.rf_channel = 30;
	nrf24_setup_rf(&nrf24, &nrf_config);
	nrf24_protocol_config_t nrf_protocol_config;
	nrf_protocol_config.crc_size = NRF24_CRCSIZE_1BYTE;
	nrf_protocol_config.address_width = NRF24_ADDRES_WIDTH_5_BYTES;
	nrf_protocol_config.en_dyn_payload_size = true;
	nrf_protocol_config.en_ack_payload = true;
	nrf_protocol_config.en_dyn_ack = true;
	nrf_protocol_config.auto_retransmit_count = 0;
	nrf_protocol_config.auto_retransmit_delay = 0;
	nrf24_setup_protocol(&nrf24, &nrf_protocol_config);
	nrf24_pipe_set_tx_addr(&nrf24, 0x123456789a);

	nrf24_pipe_config_t pipe_config;
		for (int i = 1; i < 6; i++)
		{
			pipe_config.address = 0xcfcfcfcfcf;
			pipe_config.address = (pipe_config.address & ~((uint64_t)0xff << 32)) | ((uint64_t)(i + 7) << 32);
			pipe_config.enable_auto_ack = false;
			pipe_config.payload_size = -1;
			nrf24_pipe_rx_start(&nrf24, i, &pipe_config);
		}

	pipe_config.address = 0xafafafaf01;
	pipe_config.enable_auto_ack = false;
	pipe_config.payload_size = -1;
	nrf24_pipe_rx_start(&nrf24, 0, &pipe_config);

	nrf24_mode_standby(&nrf24);
	nrf24_mode_tx(&nrf24);

	while(1)
	{
		// Чтение данных из lsm6ds3
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		int16_t temperature_raw;
		int16_t acc_raw[3];
		int16_t gyro_raw[3];
		lsm6ds3_temperature_raw_get(&ctx, &temperature_raw);
		lsm6ds3_acceleration_raw_get(&ctx, acc_raw);
		lsm6ds3_angular_rate_raw_get(&ctx, gyro_raw);

		// Пересчет из попугаев в человеческие величины
		float temperature_celsius;
		float acc_g[3];
		float gyro_dps[3];
		temperature_celsius = lsm6ds3_from_lsb_to_celsius(temperature_raw);
		for (int i = 0; i < 3; i++)
		{
			acc_g[i] = lsm6ds3_from_fs16g_to_mg(acc_raw[i]) / 1000;
			gyro_dps[i] = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[i]) / 1000;
		}

//		 Вывод
//		printf(
//			"t = %8.4f; acc = %10.4f,%10.4f,%10.4f; gyro=%10.4f,%10.4f,%10.4f" " ||| ", //\n",
//			temperature_celsius,
//			acc_g[0], acc_g[1], acc_g[2],
//			gyro_dps[0], gyro_dps[1], gyro_dps[2]
//		);


		// Чтение данные из bme280
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-



		nrf24_pipe_config_t pipe_config;

		int16_t temperataure_raw_mag;
		int16_t mag_raw[3];
		float temperature_celsius_mag;
		float mag[3];
		lis3mdl_magnetic_raw_get(&handle, mag_raw);
		lis3mdl_temperature_raw_get(&handle, &temperataure_raw_mag);
		temperature_celsius_mag = lis3mdl_from_lsb_to_celsius(temperataure_raw_mag);
		for (int i = 0; i < 3; i++){
			(mag)[i] = lis3mdl_from_fs16_to_gauss(mag_raw[i]);
		};

		//ds reading
		if (HAL_GetTick()-start_time_ds >= 750)
				{
					uint8_t buf[8];
			        onewire_read_rom(&ds, buf);
					ds18b20_read_raw_temperature(&ds, &temp_ds, &crc_ok_ds);
					ds18b20_start_conversion(&ds);
					start_time_ds = HAL_GetTick();
					ds_temp = ((float)temp_ds) / 16;
				}





		struct bme280_data comp_data;
		rc = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);
//
		if (comp_data.temperature != 0){
			shift_reg_write_bit_16(&sr_sensor, 9, true);
			HAL_Delay(300);
			shift_reg_write_bit_16(&sr_sensor, 9, false);
			HAL_Delay(300);
		};

		if (acc_g != 0){
			shift_reg_write_bit_16(&sr_sensor, 10, true);
			HAL_Delay(300);
			shift_reg_write_bit_16(&sr_sensor, 10, false);
			HAL_Delay(300);
		};


		if (mag != 0){
			shift_reg_write_bit_16(&sr_sensor, 11, true);
			HAL_Delay(300);
			shift_reg_write_bit_16(&sr_sensor, 11, false);
			HAL_Delay(300);
		};





		//nrf24_fifo_flush_tx(&nrf24);
		//nrf24_fifo_write(&nrf24, (uint8_t *)buf, sizeof(buf), false);//32



		gps_work();
		gps_get_coords(&cookie, &lat, &lon, &alt, &fix_);
		gps_get_time(&cookie, &gps_time_s, &gps_time_us);
		if (fix_<3){
			shift_reg_write_bit_16(&sr_sensor, 8, true);
			HAL_Delay(300);
			shift_reg_write_bit_16(&sr_sensor, 8, false);
			HAL_Delay(300);
			};
	    // Печать
		  printf("FIX = %1d ,lat = %2.8f; lon = %2.8f; alt = %2.8f\r\n", (int)fix_, (float)lat, (float)lon, (float)alt);




		//HAL_Delay(100);

	}

	return 0;
}
