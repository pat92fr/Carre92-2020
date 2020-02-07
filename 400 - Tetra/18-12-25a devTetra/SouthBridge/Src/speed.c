/*
 * speed.c
 *
 *  Created on: 2 févr. 2016
 *      Author: Patrick
 */

#include "speed.h"

void next_speed(float * current_speed_ms, float target_speed_ms, float acceleration_mss, float deceleration_mss, uint32_t delta_time_ms )
{
	if(*current_speed_ms<target_speed_ms)
	{
		*current_speed_ms += (float)delta_time_ms * acceleration_mss / 1000.0;
		if(*current_speed_ms>target_speed_ms)
			*current_speed_ms = target_speed_ms;
	}
	else if(*current_speed_ms>target_speed_ms)
	{
		*current_speed_ms -= (float)delta_time_ms * deceleration_mss / 1000.0;
		if(*current_speed_ms<target_speed_ms)
			*current_speed_ms = target_speed_ms;
	}
}

// distance left (m), current speed(m/s), end speed (m/s) ==> acceleration (m/s/s)
float accelaration_until_distance_left(float current_speed_ms, float final_speed_ms, float distance_left_m )
{
    if(distance_left_m<=0)
        return -999.0;
    else
        return (final_speed_ms*final_speed_ms-current_speed_ms*current_speed_ms)/(2.0*distance_left_m);
}

// evaluate the deceleration (deg/s) from angle_left_deg (deg), current_speed_dps (deg/s), end final_speed_dps (deg/s)
float accelaration_until_angle_left(float current_speed_dps, float final_speed_dps, float angle_left_deg )
{
    if(angle_left_deg<=0)
        return -999.0;
    else
        return (final_speed_dps*final_speed_dps-current_speed_dps*current_speed_dps)/(2.0*angle_left_deg);
}
