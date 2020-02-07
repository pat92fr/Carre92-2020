/*
 * myAHRS.c
 *
 *  Created on: 3 mars 2018
 *      Author: Patrick
 */

#include "myAHRS.h"
#include <math.h>
#include "mymath.h"

#define AHRSIMU_PI              3.141592653f            /*!< PI definition */
#define AHRSIMU_RAD2DEG(x)      ((x) * 57.2957795f)     /*!< Radians to degrees converter */
#define AHRSIMU_DEG2RAD(x)      ((x) * 0.0174532925f)   /*!< Radians to degrees converter */

#define HIGH_G_MAX 1.2f
#define HIGH_G_MIN 0.8f
#define HIGH_G_NORM 1.0f
#define HIGH_G_MAX_AMPLITUDE 0.2f
#define HIGH_M_MAX 0.5f
#define HIGH_M_MIN 0.3f

void HAL_AHRS_Init(AHRS_HandleTypeDef * ahrs, float alpha_ga, float alpha_mh)
{
	ahrs->initialized = false;

	ahrs->alpha_ga = alpha_ga;
	ahrs->alpha_mh = alpha_mh;

	ahrs->gyro_rate_tilt_compensated = (eulers_double){ 0.0f, 0.0f, 0.0f };
	ahrs->angles_from_acc = (eulers_double){ 0.0f, 0.0f, 0.0f };
	ahrs->angles_from_gyro = (eulers_double){ 0.0f, 0.0f, 0.0f };
	ahrs->angles_filtered = (eulers_double){ 0.0f, 0.0f, 0.0f };

	ahrs->heading = 0.0f;

	ahrs->yaw_from_gyro = 0.0f;

	ahrs->gravity = 0.0f;
	ahrs->high_g = false;
	ahrs->ratio_g = 0.0f;

	ahrs->magnetic_field = 0.0f;
	ahrs->magnetic_dip_angle = 0.0f;
	ahrs->magnetic_distortion_m = false;
}

void HAL_AHRS_Process(AHRS_HandleTypeDef * ahrs, float period, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	if(ahrs->initialized)
	{
		// High-g detection
		ahrs->gravity = sqrt( ax * ax + ay * ay + az * az );
		ahrs->high_g = ahrs->gravity > HIGH_G_MAX || ahrs->gravity < HIGH_G_MIN;
		ahrs->ratio_g = fmap(myfabs(ahrs->gravity-HIGH_G_NORM), 0.0f, HIGH_G_MAX_AMPLITUDE, 0.0f, 1.0f);

		// Magnetic field distortion
		ahrs->magnetic_field = sqrt( mx * mx + my * my + mz * mz );
		ahrs->magnetic_dip_angle = AHRSIMU_PI/2 - acos((ax*mx+ay*my+az*mz)/(ahrs->gravity*ahrs->magnetic_field));
		ahrs->magnetic_distortion_m = ahrs->magnetic_field > HIGH_M_MAX || ahrs->magnetic_field < HIGH_G_MIN;
		// TODO Take in account Magnetic Field Dip angle into boolean expression
		// TODO Make it proportional as HIgh G

		//! ST DT0060 Rev 1

		// Step 1: Compute angle derivatives Phi’ / Theta’ / Psi’ based on current angles
		// Phi / Theta / Psi and on gyroscope data Wx / Wy / Wz

		float cos_phi = cos(ahrs->angles_filtered.roll);
		float sin_phi = sin(ahrs->angles_filtered.roll);
		float cos_theta = cos(ahrs->angles_filtered.pitch);
		float tan_theta = tan(ahrs->angles_filtered.pitch);

		ahrs->gyro_rate_tilt_compensated.roll = gx + gy * sin_phi * tan_theta + gz * cos_phi * tan_theta;
		ahrs->gyro_rate_tilt_compensated.pitch = gy * cos_phi - gz * sin_phi;
		ahrs->gyro_rate_tilt_compensated.yaw = gy * sin_phi / cos_theta + gz * cos_phi / cos_theta;

		// Step 2: Compute updated angles Phi / Theta / Psi based on angles derivatives:
		ahrs->angles_from_gyro.roll = ahrs->angles_filtered.roll + ToRad(ahrs->gyro_rate_tilt_compensated.roll*period);
		ahrs->angles_from_gyro.pitch = ahrs->angles_filtered.pitch + ToRad(ahrs->gyro_rate_tilt_compensated.pitch*period);
		ahrs->angles_from_gyro.yaw = ahrs->angles_filtered.yaw + ToRad(ahrs->gyro_rate_tilt_compensated.yaw*period);

		//! ST DT0058 Rev 1

		// Step 1: computation of Phi (roll angle)

		// Roll: Phi = Atan2( Gy, Gz )

		ahrs->angles_from_acc.roll = fastAtan2(ay,az+ay*0.01);

		// use ACC and GYRO according High-G condition (proportional)
		float roll_from_gyro = ahrs->angles_from_gyro.roll;
		float roll_from_gyro_and_acc = (1.0f-ahrs->alpha_ga)*ahrs->angles_from_gyro.roll + (ahrs->alpha_ga)*ahrs->angles_from_acc.roll;
		ahrs->angles_filtered.roll =  ahrs->ratio_g*roll_from_gyro + (1.0-ahrs->ratio_g)*roll_from_gyro_and_acc;

		float phi = ahrs->angles_filtered.roll;

		// Step 2: computation of Theta (pitch angle)

		// Gz2 = Gy * Sin( Phi ) + Gz * Cos( Phi )

		float az2 = ay * sin(phi) + az * cos(phi);

		// Pitch: Theta = Atan( -Gx / Gz2)

		ahrs->angles_from_acc.pitch = fastAtan2(-ax,az2);

		// use ACC and GYRO according High-G condition (proportional)
		float pitch_from_gyro = ahrs->angles_from_gyro.pitch;
		float pitch_from_gyro_and_acc = (1.0f-ahrs->alpha_ga)*ahrs->angles_from_gyro.pitch + (ahrs->alpha_ga)*ahrs->angles_from_acc.pitch;
		ahrs->angles_filtered.pitch =  ahrs->ratio_g*pitch_from_gyro + (1.0-ahrs->ratio_g)*pitch_from_gyro_and_acc;

		// Note: if Theta = +/-90 deg, then Gy and Gz are near-zero and Gz2 is also nearzero;
		// division by zero should not be performed, Theta = -90 deg if Gx>0, and Theta= +90 deg if Gx<0;
		// Gx cannot be zero

		float theta = ahrs->angles_filtered.pitch;

		// Step 3: computation of Psi (yaw angle)

		// By2 = Bz * Sin( Phi ) – By * Cos( Phi )

		float my2 = mz * sin(phi) - my * cos(phi);

		// Bz2 = By * Sin( Phi ) + Bz * Cos( Phi )

		float mz2 = my * sin(phi) + mz * cos(phi);

		// Bx3 = Bx * Cos( Theta ) + Bz2 * Sin( Theta )

		float mx3 = mx * cos(theta) + mz2 * sin(theta);

		// Yaw: Psi = Atan2( By2 , Bx3)

		float heading = fastAtan2(my2,mx3);

		// TODO : use magnetic distortion to invalidate MAG heading for yaw
		// TODO : change using proportion of both meas*ures between gyro yaw and heading

		// complementary filter between tilt compensated magnetometer heading and gyro
		if(heading < ahrs->angles_filtered.yaw - AHRSIMU_PI*1.5f)
		{
			ahrs->angles_filtered.yaw = (1.0f-ahrs->alpha_mh)*ahrs->angles_from_gyro.yaw + (ahrs->alpha_mh)*(heading+AHRSIMU_PI*2.0f);
		}
		else if (heading > ahrs->angles_filtered.yaw + AHRSIMU_PI*1.5f)
		{
			ahrs->angles_filtered.yaw = (1.0f-ahrs->alpha_mh)*ahrs->angles_from_gyro.yaw + (ahrs->alpha_mh)*(heading-AHRSIMU_PI*2.0f);
		}
		else
			ahrs->angles_filtered.yaw = (1.0f-ahrs->alpha_mh)*ahrs->angles_from_gyro.yaw + (ahrs->alpha_mh)*heading;

		if(ahrs->angles_filtered.yaw>AHRSIMU_PI)
			ahrs->angles_filtered.yaw -= AHRSIMU_PI*2.0f;
		if(ahrs->angles_filtered.yaw<-AHRSIMU_PI)
			ahrs->angles_filtered.yaw += AHRSIMU_PI*2.0f;

		// Note: if Theta = +/-90 deg and if Phi is unstable, Psi will also be unstable; if Phi is
		// made stable as mentioned above, then Psi will also be stable. Regardless of
		// stability, the sum Phi+Psi will always be stable. See paragraph on singularities and
		// gimbal-lock.

		// If Theta = +/-90 deg, Phi and Psi will describe a rotation around the same vertical
		// axis (one degree of freedom is lost, this is also known as gimbal lock); the sum
		// Phi+Psi will always be correct regardless of the in/stability of Phi and Psi.

		// Magnetometer data can only be used when the modulus is of near nominal earth
		// field value, which happens when there are no magnetic anomalies and hard/soft
		// iron effects are compensated: Bx2 + By2	+ Bz2 = B.
		// Gyroscope data can be used to update the e-compass output when the
		// magnetometer data cannot be used.

		ahrs->yaw_from_gyro += (ahrs->gyro_rate_tilt_compensated.yaw*period);

		if(ahrs->yaw_from_gyro>180.0f)
			ahrs->yaw_from_gyro -= 360.0f;
		if(ahrs->yaw_from_gyro<-180.0f)
			ahrs->yaw_from_gyro += 360.0f;

	}
	else
	{
		// not initialized :
		// value of gyro and previous value of angles not reliables.

		//! ST DT0058 Rev 1

		// Step 1: computation of Phi (roll angle)

		// Roll: Phi = Atan2( Gy, Gz )

		ahrs->angles_from_acc.roll = fastAtan2(ay,az+ay*0.01);

		ahrs->angles_filtered.roll = ahrs->angles_from_acc.roll;

		double phi = ahrs->angles_filtered.roll;

		// Step 2: computation of Theta (pitch angle)

		// Gz2 = Gy * Sin( Phi ) + Gz * Cos( Phi )

		double az2 = ay * sin(phi) + az * cos(phi);

		// Pitch: Theta = Atan( -Gx / Gz2)

		ahrs->angles_from_acc.pitch = fastAtan2(-ax,az2);

		ahrs->angles_filtered.pitch = ahrs->angles_from_acc.pitch;

		double theta = ahrs->angles_filtered.pitch;

		// Step 3: computation of Psi (yaw angle)

		// By2 = Bz * Sin( Phi ) – By * Cos( Phi )

		double my2 = mz * sin(phi) - my * cos(phi);

		// Bz2 = By * Sin( Phi ) + Bz * Cos( Phi )

		double mz2 = my * sin(phi) + mz * cos(phi);

		// Bx3 = Bx * Cos( Theta ) + Bz2 * Sin( Theta )

		double mx3 = mx * cos(theta) + mz2 * sin(theta);

		// Yaw: Psi = Atan2( By2 , Bx3)

		double heading = fastAtan2(my2,mx3);

		ahrs->angles_filtered.yaw = heading;

		ahrs->yaw_from_gyro = ToDeg(ahrs->angles_filtered.yaw);

		ahrs->initialized = true;
	}
}


// Reference
// http://www.st.com/content/ccc/resource/technical/document/design_tip/group0/56/9a/e4/04/4b/6c/44/ef/DM00269987/files/DM00269987.pdf/jcr:content/translations/en.DM00269987.pdf
// Computing tilt measurement and tilt-compensated e-compass

// Reference
// https://delta-iot.com/la-theorie-du-filtre-complementaire/

