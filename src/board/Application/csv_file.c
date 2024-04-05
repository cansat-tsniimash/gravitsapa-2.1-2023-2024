/*
 * csv_file.c
 *
 *  Created on: 5 апр. 2023 г.
 *      Author: Install
 */
#include <stdio.h>
#include <string.h>
#include <structs.h>

uint16_t sd_parse_to_bytes_pack3(char *buffer, pack3_t *pack3) {
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%ld;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\n",
			pack3->flag, pack3->num, pack3->time_ms, pack3->accl[0], pack3->accl[1], pack3->accl[2], pack3->gyro[0], pack3->gyro[1], pack3->gyro[2], pack3->mag[0], pack3->mag[1], pack3->mag[2], pack3->crc);
	return num_written;
}
uint16_t sd_parse_to_bytes_pack2(char *buffer, pack2_t *pack2) {
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%ld;%d;%ld;%d;%d;%d;%d;%d\n",
			pack2->flag, pack2->num, pack2->time_ms, pack2->bmp_temp, pack2->bmp_press, pack2->fhotorez, pack2->ds_temp, pack2->status, pack2->find, pack2->crc);
	return num_written;
}
uint16_t sd_parse_to_bytes_pack1(char *buffer, pack1_t *pack1) {
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%ld;%ld;%ld;%ld;%ld;%ld;%d;%d\n",
			pack1->flag, pack1->num, pack1->time_ms, pack1->gps_time_s, pack1->gps_time_us, pack1->lat, pack1->lon, pack1->alt, pack1->fix, pack1->crc);
	return num_written;
}



