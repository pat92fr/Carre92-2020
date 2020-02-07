/*
 * imu.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#include "imu.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include <math.h>

/// ADD to linker : -specs=rdimon.specs
/// ADD to linker : -specs=rdimon.specs
/// ADD to linker : -specs=rdimon.specs

#define HAL_IMU_max_handles 3
static IMU_HandleTypeDef * HAL_IMU_handles[HAL_IMU_max_handles];
static uint32_t HAL_IMU_handles_count = 0;

/* Private functions ----------------------------------------------------------*/

void HAL_IMU_Scale_Data(IMU_HandleTypeDef * himu)
{
	for(short index=0;index<SENSOR_COUNT;++index)
	{
		himu->scaled_sensor_data[index] = ((float)(himu->raw_sensor_data[index])-himu->sensor_offset[index])*himu->sensor_scale_factor[index];
	}
	// then soft iron
#ifdef IMU_DO_SOFT_IRON_CORRECTION
	float x = himu->scaled_sensor_data[MAG_X];
	float y = himu->scaled_sensor_data[MAG_Y];
	float z = himu->scaled_sensor_data[MAG_Z];
	himu->scaled_sensor_data[MAG_X] = x * himu->mag_softiron_matrix[0][0] + y * himu->mag_softiron_matrix[0][1] + z * himu->mag_softiron_matrix[0][2];
	himu->scaled_sensor_data[MAG_Y] = x * himu->mag_softiron_matrix[1][0] + y * himu->mag_softiron_matrix[1][1] + z * himu->mag_softiron_matrix[1][2];
	himu->scaled_sensor_data[MAG_Z] = x * himu->mag_softiron_matrix[2][0] + y * himu->mag_softiron_matrix[2][1] + z * himu->mag_softiron_matrix[2][2];
#endif
}

void HAL_IMU_Gyro_Bias_Correction(IMU_HandleTypeDef * himu)
{
	static uint32_t counter = 0;
	// Update Gyro Rate Variance
	static float const mean_alpha = 0.03; // EWMA
	himu->gyro_rate_mean.pitch = (1-mean_alpha)*himu->gyro_rate_mean.pitch + mean_alpha*himu->raw_sensor_data[GYR_X];
	himu->gyro_rate_mean.roll = (1-mean_alpha)*himu->gyro_rate_mean.roll + mean_alpha*himu->raw_sensor_data[GYR_Y];
	himu->gyro_rate_mean.yaw = (1-mean_alpha)*himu->gyro_rate_mean.yaw + mean_alpha*himu->raw_sensor_data[GYR_Z];

	himu->gyro_rate_qmean.pitch = (1-mean_alpha)*himu->gyro_rate_qmean.pitch + mean_alpha*himu->raw_sensor_data[GYR_X]*himu->raw_sensor_data[GYR_X];
	himu->gyro_rate_qmean.roll = (1-mean_alpha)*himu->gyro_rate_qmean.roll + mean_alpha*himu->raw_sensor_data[GYR_Y]*himu->raw_sensor_data[GYR_Y];
	himu->gyro_rate_qmean.yaw = (1-mean_alpha)*himu->gyro_rate_qmean.yaw + mean_alpha*himu->raw_sensor_data[GYR_Z]*himu->raw_sensor_data[GYR_Z];

	himu->gyro_rate_deviation.pitch = sqrt(himu->gyro_rate_qmean.pitch-himu->gyro_rate_mean.pitch*himu->gyro_rate_mean.pitch);
	himu->gyro_rate_deviation.roll = sqrt(himu->gyro_rate_qmean.roll-himu->gyro_rate_mean.roll*himu->gyro_rate_mean.roll);
	himu->gyro_rate_deviation.yaw = sqrt(himu->gyro_rate_qmean.yaw-himu->gyro_rate_mean.yaw*himu->gyro_rate_mean.yaw);

	// Update sensor offset for gyro x/y/z
	static float const variance_threshold_pitch = 15.0f; // raw unit
	static float const variance_threshold_roll = 15.0f; // raw unit
	static float const variance_threshold_yaw = 10.0f; // raw unit
	if(counter>208*10) // wait for 10 seconds before updating sensor offset init values
	{
		if(himu->gyro_rate_deviation.pitch<=variance_threshold_pitch)
		{
			himu->sensor_offset[GYR_X] = himu->sensor_offset[GYR_X]*0.9f + 0.1f*himu->gyro_rate_mean.pitch;
		}
		if(himu->gyro_rate_deviation.roll<=variance_threshold_roll)
		{
			himu->sensor_offset[GYR_Y] = himu->sensor_offset[GYR_Y]*0.9f + 0.1f*himu->gyro_rate_mean.roll;
		}
		if(himu->gyro_rate_deviation.yaw<=variance_threshold_yaw)
		{
			himu->sensor_offset[GYR_Z] = himu->sensor_offset[GYR_Z]*0.9f + 0.1f*himu->gyro_rate_mean.yaw;
		}
	}
	++counter;
}

/*****************************************************************************/

void HAL_IMU_Process_Failure(void)
{
	while(1);
}

/* Public functions ----------------------------------------------------------*/

void HAL_IMU_Init(void)
{
	HAL_Delay(500); // Boot time of MEMS
}

void HAL_IMU_Add(
		IMU_HandleTypeDef * himu,
		I2C_HandleTypeDef * hi2c,
		float const sensor_offset[SENSOR_COUNT],
		float const sensor_scale_factor[SENSOR_COUNT],
		float const mag_softiron_matrix[3][3],
		HAL_Imu_ConfigurationElementTypeDef const sensor_config[],
		uint16_t sensor_device_addr[3],
		uint16_t sensor_reg_addr[3]
)
{
	// register STEPPER handle
	HAL_IMU_handles[HAL_IMU_handles_count++]=himu;

	// init
	himu->hi2c = hi2c;

	memcpy(himu->sensor_offset,sensor_offset,SENSOR_COUNT*sizeof(float));
	memcpy(himu->sensor_scale_factor,sensor_scale_factor,SENSOR_COUNT*sizeof(float));
	memcpy(himu->mag_softiron_matrix,mag_softiron_matrix,3*3*sizeof(float));

	memcpy(himu->sensor_device_addr,sensor_device_addr,3*sizeof(uint16_t));
	memcpy(himu->sensor_reg_addr,sensor_reg_addr,3*sizeof(uint16_t));

	HAL_StatusTypeDef result = HAL_OK;
	/// configure
	{
		uint8_t donnee = 0x5A;
		HAL_Imu_ConfigurationElementTypeDef const * ptr = sensor_config;
		while(ptr->addr!=0)
		{
			if(ptr->access!=IMU_READ_CHECK)
			{
				result = HAL_I2C_Mem_Write(himu->hi2c, ptr->addr<<1, ptr->reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&(ptr->value), 1, 100);
				if(result!=HAL_OK)
					while(1);
			}
			if(ptr->access!=IMU_WRITE)
			{
				result = HAL_I2C_Mem_Read(himu->hi2c, ptr->addr<<1, ptr->reg, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 100);
				if(result!=HAL_OK)
					while(1);
				if(donnee != ptr->value)
					while(1);
			}
			++ptr;
		}
	}
	// Gyro Bias Correction Init
	himu->gyro_rate_mean = (eulers_double){ 0.0f, 0.0f, 0.0f };
	himu->gyro_rate_qmean = (eulers_double){ 0.0f, 0.0f, 0.0f };
	himu->gyro_rate_deviation = (eulers_double){ 0.0f, 0.0f, 0.0f };
	// First read
	HAL_IMU_Read_Sensors(himu);

}


void HAL_IMU_Read_Sensors(IMU_HandleTypeDef * himu)
{
	uint8_t buffer[6];
	HAL_StatusTypeDef result = HAL_OK;
	result = HAL_I2C_Mem_Read(himu->hi2c, (himu->sensor_device_addr[0])<<1, himu->sensor_reg_addr[0] , I2C_MEMADD_SIZE_8BIT, buffer, 6, 10);
	if(result!=HAL_OK)
	{
		HAL_IMU_Process_Failure();
	}
	himu->raw_sensor_data[GYR_X] = (int16_t)(((buffer[0] << 8) | buffer[1]))>> 0;
	himu->raw_sensor_data[GYR_Y] = (int16_t)(((buffer[2] << 8) | buffer[3]))>> 0;
	himu->raw_sensor_data[GYR_Z] = (int16_t)(((buffer[4] << 8) | buffer[5]))>> 0;


	result = HAL_I2C_Mem_Read(himu->hi2c, (himu->sensor_device_addr[1])<<1, himu->sensor_reg_addr[1] , I2C_MEMADD_SIZE_8BIT, buffer, 6, 10);
	if(result!=HAL_OK)
	{
		HAL_IMU_Process_Failure();
	}
	himu->raw_sensor_data[ACC_X] = (int16_t)(((buffer[0] << 8) | buffer[1]))>> 0; // 0 for LSM, 2 for NXP
	himu->raw_sensor_data[ACC_Y] = (int16_t)(((buffer[2] << 8) | buffer[3]))>> 0; // 0 for LSM, 2 for NXP
	himu->raw_sensor_data[ACC_Z] = (int16_t)(((buffer[4] << 8) | buffer[5]))>> 0; // 0 for LSM, 2 for NXP

	result = HAL_I2C_Mem_Read(himu->hi2c, (himu->sensor_device_addr[2])<<1, himu->sensor_reg_addr[2] , I2C_MEMADD_SIZE_8BIT, buffer, 6, 10);
	if(result!=HAL_OK)
	{
		HAL_IMU_Process_Failure();
	}
	himu->raw_sensor_data[MAG_X] = (int16_t)(((buffer[0] << 8) | buffer[1]))>> 0;
	himu->raw_sensor_data[MAG_Y] = (int16_t)(((buffer[2] << 8) | buffer[3]))>> 0;
	himu->raw_sensor_data[MAG_Z] = (int16_t)(((buffer[4] << 8) | buffer[5]))>> 0;

	HAL_IMU_Gyro_Bias_Correction(himu);
	HAL_IMU_Scale_Data(himu);

}
