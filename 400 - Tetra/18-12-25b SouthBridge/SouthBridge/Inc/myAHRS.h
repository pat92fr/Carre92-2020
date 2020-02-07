/*
 * myAHRS.h
 *
 *  Created on: 3 mars 2018
 *      Author: Patrick
 */

#ifndef MYAHRS_H_
#define MYAHRS_H_

#include<stdbool.h>
#include "mymath.h"

typedef struct {

	bool initialized;

	float alpha_ga; // Coefficient du filtre compl�mentaire en Gyro et Acc�l�rom�tre
	float alpha_mh; // Coefficient du filtre compl�mentaire en Magn�tom�tre et Gyro

	eulers_double gyro_rate_tilt_compensated;	// DPS compens�s
	eulers_double angles_from_acc; 	// Angles calcul�s � partir de l'acc�l�rom�tre
	eulers_double angles_from_gyro;	// Angles calcul�s � partir du gyroscope
	eulers_double angles_filtered;	// Angles fusionn�s

	float heading;	// Cap magn�tique, compens� en tilt
	float yaw_from_gyro; // Cap calcul� � partir du gyro seul

	float gravity;	// Amplitude du champ gravitationnel
	bool high_g;	// Vrai si distorsion du champ gravitationnel
	float ratio_g;

	float magnetic_field; // Amplitude du champ magn�tique
	float magnetic_dip_angle;	// Angle d'incidence du champ magn�tique
	bool magnetic_distortion_m;	// Vrai si distorsion du champ magn�tique terrestre
} AHRS_HandleTypeDef;

void HAL_AHRS_Init(AHRS_HandleTypeDef * ahrs, float alpha_ga, float alpha_mh);
void HAL_AHRS_Process(AHRS_HandleTypeDef * ahrs, float period, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


#endif /* MYAHRS_H_ */
