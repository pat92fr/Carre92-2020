/*
 * speed.h
 *
 *  Created on: 2 févr. 2016
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_MATH_SPEED_H_
#define APPLICATION_USER_MATH_SPEED_H_

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* APP Public Data ------------------------------------------------------------------*/

/* MATH Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

// evaluate next speed from current speed
void next_speed(float * current_speed_ms, float target_speed_ms, float acceleration_mss, float deceleration_mss, uint32_t delta_time_ms );

// evaluate the deceleration (m/s/s) from distance_left_mm (mm), current_speed_ms (m/s), end final_speed_ms (m/s)
float accelaration_until_distance_left(float current_speed_ms, float final_speed_ms, float distance_left_mm );

// evaluate the deceleration (deg/s) from angle_left_deg (deg), current_speed_dps (deg/s), end final_speed_dps (deg/s)
float accelaration_until_angle_left(float current_speed_dps, float final_speed_dps, float angle_left_deg );

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_MATH_SPEED_H_ */
