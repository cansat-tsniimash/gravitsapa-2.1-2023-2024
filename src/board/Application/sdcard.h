/*
 * csv_file.h
 *
 *  Created on: 5 апр. 2023 г.
 *      Author: Install
 */

#ifndef SDCARD_H_
#define SDCARD_H_

#include <stdint.h>
#include <ff.h>

#include "structs.h"


typedef struct sdcard_task_t
{
	FATFS file_system; // переменная типа FATFS
	FIL file1; // хендлер файла
	FIL file2;
	FIL file3;
	FIL file_bin;

	int needs_remount;
	uint32_t sync_deadline;
} sdcard_task_t;


int sdcard_task_init(sdcard_task_t * task);
int sdcard_task_work(sdcard_task_t * task);

int sdcard_write_packet1(sdcard_task_t * task, const pack1_t * packet);
int sdcard_write_packet2(sdcard_task_t * task, const pack2_t * packet);
int sdcard_write_packet3(sdcard_task_t * task, const pack3_t * packet);


#endif /* SDCARD_H_ */
