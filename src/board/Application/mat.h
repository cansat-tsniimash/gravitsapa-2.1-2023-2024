/*
 * mat.h
 *
 *  Created on: May 9, 2024
 *      Author: kamepiula
 */

#ifndef MAT_H_
#define MAT_H_


#define DEG_TO_RAD(angle) ((angle)*M_PI/180.0)
#define RAD_TO_DEG(angle) ((angle)*180.0/M_PI)


void matrix_p_vector(const float m[2][2], const float v[2], float rv[2]);

void build_rot3(const float alpha, float m[2][2]);

void vector_rot3(const float v[2], const float alpha, float rv[2]);

void draw_arrow(float angle);


#endif /* MAT_H_ */
