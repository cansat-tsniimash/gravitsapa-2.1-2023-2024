#include <stdio.h>

#include <stm32f4xx_hal.h>
#include "Shift_Register/shift_reg.h"
#include "LSM6DS3/lsm6ds3_reg.h"
#include "BME280/bme280.h"
#include "LIS3MDL/lis3mdl_reg.h"
#include "DS18B20/one_wire.h"
#include "timers.h"
#include "includes.h"
#include "ssd1306/ssd1306.h"
#include "ssd1306/ssd1306_conf.h"

#include "sdcard.h"
#include "radio.h"

#define DS18_PERIOD 800

#define PACK1_PERIOD 10
#define PACK1_RF_DELIMITER 1

#define PACK2_PERIOD 10
#define PACK2_RF_DELIMITER 1

#define PACK3_PERIOD 10
#define PACK3_RF_DELIMITER 1


#define GPS_PACKET 50
#define ORIENT_PACKE 5
#define TASK2_PERIOD 40
#define TASK3_PERIOD 5

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;


int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
  return 0;
}

////Настройка экарана
//void oled_draw1(){
//	ssd1306_Init(); // initialize the display
//	ssd1306_WriteString("CanSat", Font_11x18, White);
//	ssd1306_UpdateScreen();
//}
//void oled_draw2(){
//	ssd1306_Init(); // initialize the display
//	ssd1306_Reset();
//	ssd1306_Fill(White);
//	ssd1306_UpdateScreen();
//	ssd1306_WriteString("Gravitsapa", Font_11x18, Black);
//	ssd1306_UpdateScreen();
//}

void oled_circle()
{
	ssd1306_Init();
	for (int i = 1; i < 32; i++)
	{
		ssd1306_FillCircle(64, 32, i, White);
		ssd1306_UpdateScreen();

	}

	HAL_Delay(500);
	ssd1306_Reset();
	ssd1306_Fill(White);
	ssd1306_UpdateScreen();
	ssd1306_WriteString("Gravitsapa", Font_11x18, Black);
	ssd1306_UpdateScreen();
	HAL_Delay(900);
}

//crc count
uint16_t Crc16(uint8_t *buf, uint16_t len) {
uint16_t crc = 0xFFFF;
	while (len--) {
		crc ^= *buf++ << 8;
		for (uint8_t i = 0; i < 8; i++)
			crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}



int app_main(void)
{
	int ds18_error = 0;
	int bme_error = 0;
	int lsm_error = 0;
	int lis_error = 0;
	int gps_error = 0;
	int sd_error = 0;

	//Настройка SR
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	shift_reg_t sr_sensor = {};
	sr_sensor.bus = &hspi2;
	sr_sensor.latch_port = GPIOC;
	sr_sensor.latch_pin  = GPIO_PIN_1;
	sr_sensor.oe_port = GPIOC;
	sr_sensor.oe_pin = GPIO_PIN_13;
	sr_sensor.value = 0;
	shift_reg_init(&sr_sensor);
	shift_reg_write_16(&sr_sensor, 0xffff);

	shift_reg_t shift_reg_r = {};
	shift_reg_r.bus = &hspi2;
	shift_reg_r.latch_port = GPIOC;
	shift_reg_r.latch_pin = GPIO_PIN_4;
	shift_reg_r.oe_port = GPIOC;
	shift_reg_r.oe_pin = GPIO_PIN_5;
	shift_reg_r.value = 0;
	shift_reg_init(&shift_reg_r);
	shift_reg_write_8(&shift_reg_r, 0xFF);

	// DS18B20
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	ds18b20_t ds;
	ds.onewire_port = GPIOA;
	ds.onewire_pin = GPIO_PIN_1;
	ds18_error = onewire_init(&ds);
	ds18_error = ds18b20_set_config(&ds, 100, -100, DS18B20_RESOLUTION_12_BIT);

	// Настройка bme280
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
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

	bme_error = bme280_soft_reset(&bme);
	bme_error = bme280_init(&bme);
	bme.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme.settings.filter = BME280_FILTER_COEFF_16;
	bme.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;
	{
		uint8_t settings_sel;
		settings_sel = BME280_OSR_PRESS_SEL;
		settings_sel |= BME280_OSR_TEMP_SEL;
		settings_sel |= BME280_OSR_HUM_SEL;
		settings_sel |= BME280_FILTER_SEL;
		settings_sel |= BME280_STANDBY_SEL;
		bme_error = bme280_set_sensor_settings(settings_sel, &bme);
		printf("bme280 settings set rc = %d\n", bme_error);
		bme_error = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme);
		printf("bme280 set sensor mode rc = %d\n", bme_error);
	}

	// Настройка lsm6ds3 =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	struct spi_sr_bus lsm_spi_ptr = {};
	lsm_spi_ptr.spi = &hspi2;
	lsm_spi_ptr.sr = &sr_sensor;
	lsm_spi_ptr.sr_pin = 4;

	stmdev_ctx_t lsm_ctx = {};
	lsm_ctx.handle = &lsm_spi_ptr;
	lsm_ctx.read_reg = lsm_spi_read;
	lsm_ctx.write_reg = lsm_spi_write;
	{
		uint8_t whoami = 0x00;
		lsm6ds3_device_id_get(&lsm_ctx, &whoami);
	//	printf("got lsm6ds3 whoami 0x%02X, expected 0x%02X\n", (int)whoami, (int)LSM6DS3_ID);
		lsm6ds3_reset_set(&lsm_ctx, PROPERTY_ENABLE);
		HAL_Delay(2);
		lsm6ds3_xl_full_scale_set(&lsm_ctx, LSM6DS3_16g);
		lsm6ds3_xl_data_rate_set(&lsm_ctx, LSM6DS3_XL_ODR_104Hz);
		lsm6ds3_gy_full_scale_set(&lsm_ctx, LSM6DS3_2000dps);
		lsm6ds3_gy_data_rate_set(&lsm_ctx, LSM6DS3_GY_ODR_104Hz);
	}

	//Настройка lis3mdl =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	struct spi_sr_bus lis_spi_ptr = {};
	lis_spi_ptr.spi = &hspi2;
	lis_spi_ptr.sr = &sr_sensor;
	lis_spi_ptr.sr_pin = 3;
	stmdev_ctx_lis_t lis_ctx = {};
	lis_ctx.handle = &lis_spi_ptr;
	lis_ctx.read_reg = lis_spi_read;
	lis_ctx.write_reg = lis_spi_write;
	{
		uint8_t whoami_lis = 0x00;
		lis3mdl_device_id_get(&lis_ctx, &whoami_lis);

		lis3mdl_reset_set(&lis_ctx, PROPERTY_ENABLE);
		HAL_Delay(2);

		lis3mdl_block_data_update_set(&lis_ctx, PROPERTY_ENABLE);
		lis3mdl_fast_low_power_set(&lis_ctx, PROPERTY_DISABLE);
		lis3mdl_full_scale_set(&lis_ctx, LIS3MDL_16_GAUSS);
		lis3mdl_data_rate_set(&lis_ctx, LIS3MDL_UHP_80Hz);
		lis3mdl_temperature_meas_set(&lis_ctx, PROPERTY_ENABLE);
		lis3mdl_operating_mode_set(&lis_ctx, LIS3MDL_CONTINUOUS_MODE);
	}

	//Настройка GPS
	gps_error = gps_init();
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);

	// SD Карта
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	sdcard_task_t sdcard;
	sd_error = sdcard_task_init(&sdcard);

	// Радио
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	radio_task_t radio;
	radio_task_init(&radio, &shift_reg_r);

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// НАСТРОЙКА ВСЕГО ЗАВЕРШЕНА
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	// Запускаем ds18
	ds18_error = ds18b20_start_conversion(&ds);
	uint32_t ds_deadline = HAL_GetTick() + DS18_PERIOD;

	int64_t gps_coord_cookie = -42;
	int64_t gps_time_cookie = -43;

	//Пакеты
	pack1_t gps_pack = { .num = 0, .flag = 0x20 };
	pack2_t status_pack = { .num = 0, .flag = 0x01 };
	pack3_t orient_pack = { .num = 0, .flag = 0x30 };
	uint32_t pack1_deadline = HAL_GetTick() + PACK1_PERIOD;
	uint32_t pack2_deadline = HAL_GetTick() + PACK2_PERIOD;
	uint32_t pack3_deadline = HAL_GetTick() + PACK3_PERIOD;

	while(1)
	{
		// Чтение данных из lsm6ds3
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		int16_t acc_raw[3];
		int16_t gyro_raw[3];
		lsm6ds3_acceleration_raw_get(&lsm_ctx, acc_raw);
		lsm6ds3_angular_rate_raw_get(&lsm_ctx, gyro_raw);

		// Пересчет из попугаев в человеческие величины
		float acc_g[3];
		float gyro_dps[3];
		for (int i = 0; i < 3; i++)
		{
			acc_g[i] = lsm6ds3_from_fs16g_to_mg(acc_raw[i]) / 1000.0;//  / 1000
			gyro_dps[i] = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[i] / 1000.0); //  / 1000
			orient_pack.accl[i] = acc_g[i];
			orient_pack.gyro[i] = gyro_dps[i];
		}

		// Чтение данные из lis3mdl
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		int16_t mag_raw[3];
		lis3mdl_magnetic_raw_get(&lis_ctx, mag_raw);
		int16_t mag[3];
		for (int i = 0; i < 3; i++)
		{
			mag[i] = lis3mdl_from_fs16_to_gauss(mag_raw[i]);
			orient_pack.mag[i] = mag[i]*1000;
		};

		// Опрос ds18
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		if (HAL_GetTick() >= ds_deadline)
		{
			uint16_t temp_ds;
			bool crc_ok_ds = false;
			uint8_t buf[8];
			onewire_read_rom(&ds, buf);
			ds18b20_read_raw_temperature(&ds, &temp_ds, &crc_ok_ds);

			ds18b20_start_conversion(&ds);
			ds_deadline = HAL_GetTick() + DS18_PERIOD;

			ds18_error = crc_ok_ds ? 0 : 142;
			status_pack.ds_temp = ((float)temp_ds * 10) / 16;
		}

		// Опрос bme280
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		struct bme280_data comp_data;
		bme_error = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);
		status_pack.bmp_temp = comp_data.temperature * 100;
		status_pack.bmp_press = comp_data.pressure;

		// Опрос фоторезистора
		//status_pack.fhotorez = lux;

		// Опрос gps
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		float gps_lon, gps_lat, gps_alt;
		int gps_fix;
		uint64_t gps_time_s;
		uint32_t gps_time_us;
		gps_get_coords(&gps_coord_cookie, &gps_lat, &gps_lon, &gps_alt, &gps_fix);
		gps_get_time(&gps_time_cookie, &gps_time_s, &gps_time_us);
		gps_pack.lat = gps_lat;
		gps_pack.lon = gps_lon;
		gps_pack.alt = gps_alt;
		gps_pack.gps_time_s = gps_time_s;
		gps_pack.gps_time_us = gps_time_us;
		gps_pack.fix = gps_fix;
		gps_error = gps_fix != 0;

		if (HAL_GetTick() >= pack1_deadline)
		{
			pack1_deadline = HAL_GetTick() + PACK1_PERIOD;

			gps_pack.time_ms = HAL_GetTick();
			gps_pack.num += 1;
			gps_pack.crc = Crc16((uint8_t *)&gps_pack, sizeof(gps_pack) - 2);
			sd_error = sdcard_write_packet1(&sdcard, &gps_pack);

			if (gps_pack.num % PACK1_RF_DELIMITER == 0)
				nrf24_fifo_write(&radio.radio, (uint8_t*)&gps_pack, sizeof(gps_pack), false);

		}

		if (HAL_GetTick() >= pack2_deadline)
		{
			pack2_deadline = HAL_GetTick() + PACK2_PERIOD;

			status_pack.time_ms = HAL_GetTick();
			status_pack.num += 1;
			status_pack.crc = Crc16((uint8_t *)&status_pack, sizeof(status_pack) - 2);
			sd_error = sdcard_write_packet2(&sdcard, &status_pack);

			if (status_pack.num % PACK2_RF_DELIMITER == 0)
				nrf24_fifo_write(&radio.radio, (uint8_t *)&status_pack, sizeof(status_pack), false);
		}

		if (HAL_GetTick() >= pack3_deadline)
		{
			pack3_deadline = HAL_GetTick() + PACK3_PERIOD;

			orient_pack.time_ms = HAL_GetTick();
			orient_pack.num += 1;
			orient_pack.crc = Crc16((uint8_t *)&orient_pack, sizeof(orient_pack) - 2);// <<------pack
			sd_error = sdcard_write_packet3(&sdcard, &orient_pack);

			if (orient_pack.num % PACK3_RF_DELIMITER == 0)
				nrf24_fifo_write(&radio.radio, (uint8_t *)&orient_pack, sizeof(orient_pack), false);

		}

		// Чистим прерывания для RF24
		int nrf_irq;
		nrf24_irq_get(&radio.radio, &nrf_irq);
		nrf24_irq_clear(&radio.radio, nrf_irq);

		// Зажигаем разные лампочки по статусу аппаратуры
		if (bme_error == 0)
			shift_reg_write_bit_16(&sr_sensor, 9, true);
		if (lsm_error == 0)
			shift_reg_write_bit_16(&sr_sensor, 10, true);
		if (lis_error == 0)
			shift_reg_write_bit_16(&sr_sensor, 11, true);
		if (gps_error == 0)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);


		gps_work();
		sdcard_task_work(&sdcard);
	}

	return 0;
}
