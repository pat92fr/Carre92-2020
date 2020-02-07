/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATH_BASE_H
#define __MATH_BASE_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* APP Public Data ------------------------------------------------------------------*/

/* MATH Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

int32_t constrain(int32_t x, int32_t min, int32_t max);
float fconstrain(float x, float min, float max);
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);
//double fmap(double x, double in_min, double in_max, double out_min, double out_max);
float myfabs(float n);
//double myfabs(double n);

#define ToRad(x) ((x) * 0.01745329252)  // *pi/180.0
#define ToDeg(x) ((x) * 57.2957795131)  // *180.0/pi

#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f

float fastAtan2( float y, float x);

typedef struct
{
	float pitch;
	float roll;
	float yaw;
} eulers_double;

#ifdef __cplusplus
}
#endif

#endif /* __MATH_BASE_H */




