	/*
	 * aZimut.c
	 *
	 *  Created on: May 31, 2024
	 *      Author: kamepiula
	 */

	#include "mat.h"
	#include "math.h"
	#include "ssd1306/ssd1306.h"
	#define RAD_TO_DEG(angle) ((angle)*180.0/M_PI)

	/* pi */

	//extern double gps_lat;
	//extern double gps_lon;
	//extern double coord_base_lat;
	//extern double coord_base_lon;

	double get_azimuth_deg(double base_lon, double base_lat, double my_lon, double my_lat)
	{
		double gps_lat = my_lat;
		double gps_lon = my_lon;
		double coord_base_lat = base_lat;
		double coord_base_lon = base_lon;
		double c = 90 - gps_lat;
		double b = 90 - coord_base_lat;
		double A = coord_base_lon - gps_lon;

		c = c/180*M_PI;
		b = b/180*M_PI;
		A = A/180*M_PI;

		double cosa = (cos(b) * cos(c) + sin(b) * sin(c) * cos(A));
		double sina = sqrt(1 - cosa * cosa);
		double cosB = (cos(b) - cosa * cos(c)) / (sina * sin(c));
		double B = acos(cosB);
		const double angle_deg = RAD_TO_DEG(B);
		//const double angle_rad = B;

		return angle_deg;
	}
