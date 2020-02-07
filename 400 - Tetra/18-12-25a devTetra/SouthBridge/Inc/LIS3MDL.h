/*
 * LIS3MDL.h
 *
 *  Created on: 31 janv. 2018
 *      Author: Patrick
 */

// https://www.pololu.com/file/download/LIS3MDL.pdf

#ifndef LIS3MDL_H_
#define LIS3MDL_H_

#define IMU_LIS3MDL_ADDRESS (0x1E)
#define IMU_LIS3MDL_WHO_AM_I (0x3D)

typedef enum IMU_LIS3MDL_REGISTER
{
	IMU_LIS3MDL_REGISTER_WHO_AM_I        = 0x0F,
	IMU_LIS3MDL_REGISTER_CTRL1           = 0x20,
	IMU_LIS3MDL_REGISTER_CTRL2           = 0x21,
	IMU_LIS3MDL_REGISTER_CTRL3           = 0x22,
	IMU_LIS3MDL_REGISTER_CTRL4           = 0x23,
	IMU_LIS3MDL_REGISTER_CTRL5           = 0x24,
	IMU_LIS3MDL_REGISTER_STATUS_REG      = 0x27,

	IMU_LIS3MDL_REGISTER_OUT_X_L         = 0x28,
	IMU_LIS3MDL_REGISTER_OUT_X_H         = 0x29,
	IMU_LIS3MDL_REGISTER_OUT_Y_L         = 0x2A,
	IMU_LIS3MDL_REGISTER_OUT_Y_H         = 0x2B,
	IMU_LIS3MDL_REGISTER_OUT_Z_L         = 0x2C,
	IMU_LIS3MDL_REGISTER_OUT_Z_H         = 0x2D,

} t_imu_LIS3MDL_registers;

#endif /* LIS3MDL_H_ */
