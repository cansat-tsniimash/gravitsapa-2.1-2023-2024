/*
 * structs.h
 *
 *  Created on: 5 апр. 2023 г.
 *      Author: Install
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <stdint.h>

#pragma pack(push,1) //<-------
//структурки пакетиков
typedef struct{
	uint16_t flag;
	uint16_t id;
	uint32_t time;
	int16_t bmp_temp;
	uint32_t bmp_press;
	int16_t accl[3];
	uint16_t crc;
}pack_org_t;

// 13 byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time_ms;
	int16_t accl[3];
	int16_t gyro[3];
	int16_t mag[3];
	uint16_t crc;
}pack3_t;

//22byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time_ms;
	int16_t bmp_temp;
	uint32_t bmp_press;
	uint16_t fhotorez;
	int16_t ds_temp;
	uint16_t state_now;
	uint8_t find;
	uint16_t crc;
}pack2_t;

//30 byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time_ms;
	uint32_t gps_time_s;
	uint32_t gps_time_us;
	float lat;
	float lon;
	float alt;
	float angle;
	float lat_base;
	float lon_base;
	int8_t fix;
	uint16_t crc;
}pack1_t;


#pragma pack(pop)

#endif /* STRUCTS_H_ */
