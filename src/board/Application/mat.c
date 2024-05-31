/*
 * mat.c
 *
 *  Created on: May 9, 2024
 *      Author: kamepiula
 */

#include "mat.h"
#include "math.h"
#include "ssd1306/ssd1306.h"
#include "ATGM336H/nmea_gps.h"
#include "aZimut.h"

extern float coord_base_lat;
extern float coord_base_lon;
extern float gps_lon, gps_lat, gps_alt;
extern float north_lat, north_lon;

void matrix_p_vector(const float m[2][2], const float v[2], float rv[2])
{
	rv[0] = m[0][0] * v[0] + m[0][1] * v[1];
	rv[1] = m[1][0] * v[0] + m[1][1] * v[1];
}


void build_rot3(const float alpha, float m[2][2])
{
	m[0][0] = cos(alpha); m[0][1] = - sin(alpha);
	m[1][0] = sin(alpha); m[1][1] =   cos(alpha);
}


void vector_rot3(const float v[2], const float alpha, float rv[2])
{
	float matrix[2][2];
	build_rot3(alpha, matrix);
	matrix_p_vector(matrix, v, rv);
}


void vector_add(float v1[2], const float v2[2])
{
	v1[0] += v2[0];
	v1[1] += v2[1];
}


void draw() //FIXME draw_arrow() - last name
{
	const double angle = get_azimuth_deg(coord_base_lon, coord_base_lat, gps_lon, gps_lat);
	const float line_size = 50;
	const float line_begin_[2] = {-line_size/2, 0};
	const float line_end_[2] = {line_size/2, 0};
	const float arrow_left_[2] = {line_size/2 - 12, -3};
	const float arrow_right_[2] = {line_size/2 - 12, +3};
	float line_begin[2];
	float line_end[2];
	float arrow_left[2];
	float arrow_right[2];

	vector_rot3(line_begin_, angle, line_begin);
	vector_rot3(line_end_, angle, line_end);
	vector_rot3(arrow_left_, angle, arrow_left);
	vector_rot3(arrow_right_, angle, arrow_right);

	const float offset[2] = {64, 32};
	vector_add(line_begin, offset);
	vector_add(line_end, offset);
	vector_add(arrow_left, offset);
	vector_add(arrow_right, offset);

	ssd1306_Line(line_begin[0]+0.5, line_begin[1]+0.5, line_end[0]+0.5, line_end[1]+0.5, White);
	ssd1306_Line(line_end[0]+0.5, line_end[1]+0.5, arrow_left[0]+0.5, arrow_left[1]+0.5, White);
	ssd1306_Line(line_end[0]+0.5, line_end[1]+0.5, arrow_right[0]+0.5, arrow_right[1]+0.5, White);
	//ssd1306_Line(arrow_left[0]+0.5, arrow_left[1]+0.5, arrow_right[0]+0.5, arrow_right[1]+0.5, White);

	ssd1306_Line(95, 7, 115, 7, White);
	ssd1306_Line(115, 7, 110, 4, White);
	ssd1306_Line(115, 7, 110, 10, White);

	ssd1306_SetCursor(116, 4);
	ssd1306_WriteStringVertical("N", Font_7x10, White);

	ssd1306_SetCursor(116, 20);
	int64_t cookie;
	struct minmea_sentence_gga gga;
	gps_get_gga(&cookie, &gga);
	char time_text_buffer[10] = {};
	char distance_text_buffer[10] = {};
	const char separator = (gga.time.seconds % 2 != 0 && gga.fix_quality > 0) ? ' ' : ':';
	snprintf(
			time_text_buffer, sizeof(time_text_buffer),
			"%02d%c%02dUTC", (int)gga.time.hours, separator, (int)gga.time.minutes
	);
	ssd1306_WriteStringVertical(time_text_buffer, Font_6x8, White);
	float dLat, dLon, a, c, R;
	float distance;

	//Вычисление расстояния от чела к штабу
	R = 6371;
	dLat = DEG_TO_RAD(coord_base_lat - gps_lat);
	dLon = DEG_TO_RAD(coord_base_lon - gps_lon);
	a = sin(dLat/2) * sin(dLat/2) + cos(DEG_TO_RAD(gps_lat)) * cos(DEG_TO_RAD(coord_base_lat)) * sin(dLon/2) * sin(dLon/2);
	c = 2 * atan2(sqrt(a), sqrt(1-a));

	distance = R * c;

 	//angle = acos(cos(distance1) - cos(distance) * cos(distance2) / sin(distance) * sin(distance2));
	snprintf(distance_text_buffer, sizeof(distance_text_buffer),
			"%.*f km", 1, (float)distance
	);
	ssd1306_SetCursor(0, 9);
	ssd1306_WriteStringVertical(distance_text_buffer, Font_6x8, White);


}
