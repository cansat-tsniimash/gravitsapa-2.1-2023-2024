/*
 * csv_file.c
 *
 *  Created on: 5 апр. 2023 г.
 *      Author: Install
 */
#include "sdcard.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ff_gen_drv.h"

#include "structs.h"


static int open_and_make_header(FIL * file, const char * file_path_template, const char * header)
{
	FRESULT rc;
	for (int i = 0; i < 100; i++)
	{
		char file_path_buffer[100] = {};
		snprintf(
			file_path_buffer, sizeof(file_path_buffer),
			file_path_template, i
		);

		rc = f_open(file, file_path_buffer, FA_WRITE | FA_CREATE_NEW); // открытие файла, обязательно для работы с ним
		if (rc == FR_EXIST)
			continue;
		else if (rc == FR_OK)
			break;
		else
			return rc;
	}

	if (header != NULL)
	{
		int rc2 = f_puts(header, file);
		if (rc2 < 0)
			return 100;
	}

	rc = f_sync(file);
	if (rc != FR_OK)
		return rc;

	return FR_OK;
}


static int sdcard_task_remount(sdcard_task_t * task)
{
	if (!task->needs_remount)
		return FR_OK;

	FIL * files[] = {&task->file1, &task->file2, &task->file3, &task->file_bin};
	for (size_t i = 0; i < sizeof(files)/sizeof(*files); i++)
	{
		f_close(files[i]);
		memset(files[i], 0x00, sizeof(FIL));
	}

	extern Disk_drvTypeDef disk;
	disk.is_initialized[0] = 0; // hach for cubemx
	FRESULT rc = f_mount(&task->file_system, "0", 1);
	if (rc != FR_OK)
		return rc;

	rc = open_and_make_header(
			&task->file1,
			"packet1_%d.csv",
			"flag; num; time_ms; gps_time_s; gps_time_us; lat; lon; alt; fix; crc\n"
	);
	if (rc != FR_OK)
		return rc;

	rc = open_and_make_header(
			&task->file2,
			"packet2_%d.csv",
			"flag; num; time_ms; bme_temp; bme_press; photorez; ds_temp; status; find; crc\n"
	);
	if (rc != FR_OK)
		return rc;


	rc = open_and_make_header(
			&task->file3,
			"packet3_%d.csv",
			"flag; num; time_ms; accl_x; accl_y; accl_z; gyro_x; gyro_y; gyro_z; mag_x; mag_y; mag_z; crc\n"
	);
	if (rc != FR_OK)
		return rc;


	rc = open_and_make_header(
			&task->file_bin,
			"packet_%d.bin",
			NULL
	);
	if (rc != FR_OK)
		return rc;

	return FR_OK;
}



int sdcard_task_init(sdcard_task_t * task)
{
	memset(task, 0x00, sizeof(*task));
	int rc = sdcard_task_remount(task);
	task->needs_remount = (rc != FR_OK);

	return rc;
}



int sdcard_task_work(sdcard_task_t * task)
{
	if (task->needs_remount)
	{
		FRESULT mount_rc = sdcard_task_remount(task);
		if (mount_rc != FR_OK)
			return mount_rc;

		task->needs_remount = 0;
	}


	if (HAL_GetTick() < task->sync_deadline)
		return FR_OK;

	FIL * files[] = {&task->file1, &task->file2, &task->file3, &task->file_bin};
	for (size_t i = 0; i < sizeof(files)/sizeof(*files); i++)
	{
		FRESULT sync_rc = f_sync(files[i]);
		if (sync_rc != FR_OK)
		{
			task->needs_remount = 1;
			return sync_rc;
		}
	}

	task->sync_deadline = HAL_GetTick() + 1000;
	return FR_OK;
}



int sdcard_write_packet1(sdcard_task_t * task, const pack1_t * packet)
{
	if (task->needs_remount)
		return 200;

	char buffer[300];
	UINT rsize;
	const int to_write = snprintf(
			buffer, sizeof(buffer),
			"%d;%"PRIu16";%"PRIu32";"
			"%"PRIu32";%"PRIu32";"
			"%f;%f;%f;%d;"
			"%"PRIu16";"
			"\n",
			(int)packet->flag, packet->num, packet->time_ms,
			packet->gps_time_s, packet->gps_time_us,
			packet->lat, packet->lon, packet->alt, (int)packet->fix,
			packet->crc
	);
	if (to_write < 0)
		return to_write;

	FRESULT rc = f_write(&task->file1, buffer, to_write, &rsize);
	if (rc != FR_OK)
	{
		task->needs_remount = 1;
		return rc;
	}

	rc = f_write(&task->file_bin, packet, sizeof(*packet), &rsize);
	if (rc != FR_OK)
	{
		task->needs_remount = 1;
		return rc;
	}

	return rc;
}


int sdcard_write_packet2(sdcard_task_t * task, const pack2_t * packet)
{
	if (task->needs_remount)
		return 200;

	char buffer[300];
	UINT rsize;
	const int to_write = snprintf(
			buffer, sizeof(buffer),
			"%d;%"PRIu16";%"PRIu32";"
			"%"PRId16";%"PRIu32";"
			"%"PRIu16";%"PRId16";"
			"%"PRIu16";%d;"
			"%"PRIu16""
			"\n",
			(int)packet->flag, packet->num, packet->time_ms,
			packet->bmp_temp, packet->bmp_press,
			packet->fhotorez, packet->ds_temp,
			packet->state_now, (int)packet->find,
			packet->crc
	);
	if (to_write < 0)
		return to_write;

	FRESULT rc = f_write(&task->file2, buffer, to_write, &rsize);
	if (rc != FR_OK)
	{
		task->needs_remount = 1;
		return rc;
	}

	rc = f_write(&task->file_bin, packet, sizeof(*packet), &rsize);
	if (rc != FR_OK)
	{
		task->needs_remount = 1;
		return rc;
	}

	return rc;
}


int sdcard_write_packet3(sdcard_task_t * task, const pack3_t * packet)
{
	if (task->needs_remount)
		return 200;

	char buffer[300];
	UINT rsize;
	const int to_write = snprintf(
			buffer, sizeof(buffer),
			"%d;%"PRIu16";%"PRIu32";"
			"%"PRId16";%"PRId16";%"PRId16";"
			"%"PRId16";%"PRId16";%"PRId16";"
			"%"PRId16";%"PRId16";%"PRId16";"
			"%"PRIu16""
			"\n",
			(int)packet->flag, packet->num, packet->time_ms,
			packet->accl[0], packet->accl[1], packet->accl[2],
			packet->gyro[0], packet->gyro[1], packet->gyro[2],
			packet->mag[0], packet->mag[1], packet->mag[2],
			packet->crc
	);
	if (to_write < 0)
		return to_write;

	FRESULT rc = f_write(&task->file3, buffer, to_write, &rsize);
	if (rc != FR_OK)
	{
		task->needs_remount = 1;
		return rc;
	}

	rc = f_write(&task->file_bin, packet, sizeof(*packet), &rsize);
	if (rc != FR_OK)
	{
		task->needs_remount = 1;
		return rc;
	}

	return rc;
}
