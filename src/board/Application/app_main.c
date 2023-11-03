#include <stdio.h>

#include <stm32f4xx_hal.h>
#include "shift_reg.h"
#include "lsm6ds3_reg.h"
#include "bme280.h"




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

	reg_addr |= (1 << 7);
	HAL_SPI_Transmit(lsm_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(lsm_spi_ptr->spi, data, data_len, HAL_MAX_DELAY);

	lsm_spi_cs_up(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);

	return 0;
}

static int32_t lsm_spi_write(void * intf_ptr, uint8_t reg_addr, const uint8_t * data, uint16_t data_len)
{
	struct spi_sr_bus * lsm_spi_ptr = intf_ptr;
	lsm_spi_cs_down(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);
	reg_addr &= ~(1 << 7);
	HAL_SPI_Transmit(lsm_spi_ptr->spi, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(lsm_spi_ptr->spi, (uint8_t*)data, data_len, HAL_MAX_DELAY);
	lsm_spi_cs_up(lsm_spi_ptr->sr, lsm_spi_ptr->sr_pin);

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



int app_main(void)
{
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

		 //Вывод
//		printf(
//			"t = %8.4f; acc = %10.4f,%10.4f,%10.4f; gyro=%10.4f,%10.4f,%10.4f" " ||| ", //\n",
//			temperature_celsius,
//			acc_g[0], acc_g[1], acc_g[2],
//			gyro_dps[0], gyro_dps[1], gyro_dps[2]
//		);


		// Чтение данные из bme280
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

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
//
//		if (gyro_dps.gyro_dps[0] & gyro_dps.gyro_dps[1] & gyro_dps.gyro_dps[2]  != 1){
//			shift_reg_write_bit_16(&sr, 11, true);
//			HAL_Delay(300);
//			shift_reg_write_bit_16(&sr, 11, false);
//			HAL_Delay(300);
//		};
//
//		if (comp_data.temperature != 1){
//			shift_reg_write_bit_16(&sr, 12, true);
//			HAL_Delay(300);
//			shift_reg_write_bit_16(&sr, 12, false);
//			HAL_Delay(300);
//		};

	    // Печать

//		printf(
//			"temp = %8.4f; pressure = %10.4f; hum = %10.4f\n",
//			(float)comp_data.temperature,
//			(float)comp_data.pressure,
//			(float)comp_data.humidity
//		);


		//HAL_Delay(100);
	}

	return 0;
}
