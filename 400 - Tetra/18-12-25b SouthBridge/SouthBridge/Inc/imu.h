/*
 * imu.h
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_IMU_H_
#define APPLICATION_USER_HAL_IMU_H_

#include "stm32f4xx_hal.h"
#include "mymath.h"

//#define IMU_DO_SOFT_IRON_CORRECTION

enum
{
	GYR_X = 0,
	GYR_Y,
	GYR_Z,
	ACC_X,
	ACC_Y,
	ACC_Z,
	MAG_X,
	MAG_Y,
	MAG_Z,
	SENSOR_COUNT
};

typedef enum
{
	IMU_READ_CHECK,
	IMU_WRITE,
	IMU_WRITE_AND_CHECK,
} HAL_Imu_AccessTypeDef;


typedef struct
{
	int addr;
	int reg;
	uint8_t value;
	HAL_Imu_AccessTypeDef access;
	//! TODO pre-delay
	//! TODO post-delay
} HAL_Imu_ConfigurationElementTypeDef;

/**
  * @brief  IMU Handler Structure definition
  */
typedef struct {
	I2C_HandleTypeDef * hi2c;		/*!< Specifies the GPIO pin used to generate STEP/PULSE signal. */

	float sensor_offset[SENSOR_COUNT];
	float sensor_scale_factor[SENSOR_COUNT];
	float mag_softiron_matrix[3][3];

	uint16_t sensor_device_addr[3];
	uint16_t sensor_reg_addr[3];

	int16_t raw_sensor_data[SENSOR_COUNT];

	float scaled_sensor_data[SENSOR_COUNT];

	// Gyro Bias Correction
	eulers_double gyro_rate_mean; // Moyenne du gyroscope
	eulers_double gyro_rate_qmean;	// Moyenne quadratique du gyroscope
	eulers_double gyro_rate_deviation; // Variance du gyroscope
} IMU_HandleTypeDef;

#ifdef __cplusplus
 extern "C" {
#endif

void HAL_IMU_Init();

void HAL_IMU_Add(
		IMU_HandleTypeDef * himu,
		I2C_HandleTypeDef * hi2c,
		float const sensor_offset[SENSOR_COUNT],
		float const sensor_scale_factor[SENSOR_COUNT],
		float const mag_softiron_matrix[3][3],
		HAL_Imu_ConfigurationElementTypeDef const sensor_config[],
		uint16_t sensor_device_addr[3],
		uint16_t sensor_reg_addr[3]
);

void HAL_IMU_Read_Sensors(IMU_HandleTypeDef * himu);

#ifdef __cplusplus
}
#endif

 #endif /* APPLICATION_USER_HAL_IMU_H_ */
