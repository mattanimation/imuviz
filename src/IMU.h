#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "i2c-dev.h"
#include "LSM9DS0.h"
#include "LSM9DS1.h"
#include "LSM6DSL.h"
#include "LIS3MDL.h"

int file;
int BerryIMUversion = 99;

void  readBlock(uint8_t command, uint8_t size, uint8_t *data)
{
    int result = i2c_smbus_read_i2c_block_data(file, command, size, data);
    if (result != size){
		printf("Failed to read block from I2C.");
		exit(1);
	}
}

void selectDevice(int file, int addr)
{
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		 printf("Failed to select I2C device.");
	}
}


void readACC(int  a[])
{
	uint8_t block[6];
	if (BerryIMUversion == 1){
		selectDevice(file,LSM9DS0_ACC_ADDRESS);
		readBlock(0x80 |  LSM9DS0_OUT_X_L_A, sizeof(block), block);
	}
	else if (BerryIMUversion == 2){
		selectDevice(file,LSM9DS1_ACC_ADDRESS);
		readBlock(LSM9DS1_OUT_X_L_XL, sizeof(block), block);
	}
	else if (BerryIMUversion == 3){
		selectDevice(file,LSM6DSL_ADDRESS);
		readBlock(LSM6DSL_OUTX_L_XL, sizeof(block), block);
	}
	// Combine readings for each axis.
	a[0] = (int16_t)(block[0] | block[1] << 8);
	a[1] = (int16_t)(block[2] | block[3] << 8);
	a[2] = (int16_t)(block[4] | block[5] << 8);
}


void readMAG(int  m[])
{
	uint8_t block[6];
    if (BerryIMUversion == 1){
		selectDevice(file,LSM9DS0_MAG_ADDRESS);
		readBlock(0x80 |  LSM9DS0_OUT_X_L_M, sizeof(block), block);
	}
	else if (BerryIMUversion == 2){
		selectDevice(file,LSM9DS1_MAG_ADDRESS);
		readBlock(LSM9DS1_OUT_X_L_M, sizeof(block), block);
	}
	else if (BerryIMUversion == 3){
		selectDevice(file,LIS3MDL_ADDRESS);
		readBlock(LIS3MDL_OUT_X_L, sizeof(block), block);
	}

	// Combine readings for each axis.
	m[0] = (int16_t)(block[0] | block[1] << 8);
	m[1] = (int16_t)(block[2] | block[3] << 8);
	m[2] = (int16_t)(block[4] | block[5] << 8);

}

void readGYR(int g[])
{
	uint8_t block[6];
    if (BerryIMUversion == 1){
		selectDevice(file,LSM9DS0_GYR_ADDRESS);
		readBlock(0x80 |  LSM9DS0_OUT_X_L_G, sizeof(block), block);
	}
	else if (BerryIMUversion == 2){
		selectDevice(file,LSM9DS1_GYR_ADDRESS);
		readBlock(LSM9DS1_OUT_X_L_G, sizeof(block), block);
	}
	else if (BerryIMUversion == 3){
		selectDevice(file,LSM6DSL_ADDRESS);
		readBlock(LSM6DSL_OUTX_L_G, sizeof(block), block);
	}

	// Combine readings for each axis.
	g[0] = (int16_t)(block[0] | block[1] << 8);
	g[1] = (int16_t)(block[2] | block[3] << 8);
	g[2] = (int16_t)(block[4] | block[5] << 8);
}


void writeAccReg(uint8_t reg, uint8_t value)
{
	if (BerryIMUversion == 1)
		selectDevice(file,LSM9DS0_ACC_ADDRESS);
	else if (BerryIMUversion == 2)
		selectDevice(file,LSM9DS1_ACC_ADDRESS);
	else if (BerryIMUversion == 3)
		selectDevice(file,LSM6DSL_ADDRESS);

	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf ("Failed to write byte to I2C Acc.");
        exit(1);
    }
}

void writeMagReg(uint8_t reg, uint8_t value)
{
	if (BerryIMUversion == 1)
		selectDevice(file,LSM9DS0_MAG_ADDRESS);
	else if (BerryIMUversion == 2)
		selectDevice(file,LSM9DS1_MAG_ADDRESS);
	else if (BerryIMUversion == 3)
		selectDevice(file,LIS3MDL_ADDRESS);

	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf("Failed to write byte to I2C Mag.");
		exit(1);
	}
}


void writeGyrReg(uint8_t reg, uint8_t value)
{
	if (BerryIMUversion == 1)
		selectDevice(file,LSM9DS0_GYR_ADDRESS);
	else if (BerryIMUversion == 2)
		selectDevice(file,LSM9DS1_GYR_ADDRESS);
	else if (BerryIMUversion == 3)
		selectDevice(file,LSM6DSL_ADDRESS);

	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf("Failed to write byte to I2C Gyr.");
		exit(1);
	}
}


void detectIMU()
{
	//__u16 block[I2C_SMBUS_BLOCK_MAX];

	//int res, bus,  size;

	char filename[20];
	sprintf(filename, "/dev/i2c-%d", 1);
	file = open(filename, O_RDWR);
	if (file<0) {
		printf("\n####    Unable to open I2C bus!    ####\n");
		exit(1);
	}

	//Detect if BerryIMUv1 (Which uses a LSM9DS0) is connected
	selectDevice(file,LSM9DS0_ACC_ADDRESS);
	int LSM9DS0_WHO_XM_response = i2c_smbus_read_byte_data(file, LSM9DS0_WHO_AM_I_XM);

	selectDevice(file,LSM9DS0_GYR_ADDRESS);
	int LSM9DS0_WHO_G_response = i2c_smbus_read_byte_data(file, LSM9DS0_WHO_AM_I_G);

	if (LSM9DS0_WHO_G_response == 0xd4 && LSM9DS0_WHO_XM_response == 0x49){
		printf ("\n\n\n#####   BerryIMUv1/LSM9DS0  DETECTED    #####\n\n");
		BerryIMUversion = 1;
	}

	//Detect if BerryIMUv2 (Which uses a LSM9DS1) is connected
	selectDevice(file,LSM9DS1_MAG_ADDRESS);
	int LSM9DS1_WHO_M_response = i2c_smbus_read_byte_data(file, LSM9DS1_WHO_AM_I_M);

	selectDevice(file,LSM9DS1_GYR_ADDRESS);
	int LSM9DS1_WHO_XG_response = i2c_smbus_read_byte_data(file, LSM9DS1_WHO_AM_I_XG);

    if (LSM9DS1_WHO_XG_response == 0x68 && LSM9DS1_WHO_M_response == 0x3d){
		printf ("\n\n\n#####   BerryIMUv2/LSM9DS1  DETECTED    #####\n\n");
		BerryIMUversion = 2;
	}

	//Detect if BerryIMUv3 (Which uses a LSM6DSL and LIS3MDL) is connected
	selectDevice(file,LSM6DSL_ADDRESS);
	int LSM6DSL_WHO_M_response = i2c_smbus_read_byte_data(file, LSM6DSL_WHO_AM_I);

	selectDevice(file,LIS3MDL_ADDRESS);
	int LIS3MDL_WHO_XG_response = i2c_smbus_read_byte_data(file, LIS3MDL_WHO_AM_I);

	if ( LSM6DSL_WHO_M_response == 0x6A && LIS3MDL_WHO_XG_response == 0x3D){
		printf ("\n\n\n#####   BerryIMUv3  DETECTED    #####\n\n");
		BerryIMUversion = 3;
	}

	sleep(1);
	if (BerryIMUversion == 99){
		printf ("\n####    NO IMU DETECTED    ####\n");
		exit(1);
	}
}

void enableIMU()
{

	if (BerryIMUversion == 1){//For BerryIMUv1
		// Enable Gyroscope
		writeGyrReg(LSM9DS0_CTRL_REG1_G, 0b00001111); // Normal power mode, all axes enabled
		writeGyrReg(LSM9DS0_CTRL_REG4_G, 0b00110000); // Continuos update, 2000 dps full scale

		// Enable accelerometer.
		writeAccReg(LSM9DS0_CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuous update,  100Hz data rate
		writeAccReg(LSM9DS0_CTRL_REG2_XM, 0b00100000); // +/- 16G full scale

		//Enable  magnetometer
		writeMagReg(LSM9DS0_CTRL_REG5_XM, 0b11110000); // Temp enable, M data rate = 50Hz
		writeMagReg(LSM9DS0_CTRL_REG6_XM, 0b01100000); // +/-12gauss
		writeMagReg(LSM9DS0_CTRL_REG7_XM, 0b00000000); // Continuous-conversion mode
	}

	if (BerryIMUversion == 2){//For BerryIMUv2
		// Enable gyroscope
		writeGyrReg(LSM9DS1_CTRL_REG4,0b00111000);      // z, y, x axis enabled for gyro
		writeGyrReg(LSM9DS1_CTRL_REG1_G,0b10111000);    // Gyro ODR = 476Hz, 2000 dps
		writeGyrReg(LSM9DS1_ORIENT_CFG_G,0b10111000);   // Swap orientation

		// Enable the accelerometer
		writeAccReg(LSM9DS1_CTRL_REG5_XL,0b00111000);   // z, y, x axis enabled for accelerometer
		writeAccReg(LSM9DS1_CTRL_REG6_XL,0b00101000);   // +/- 16g

		//Enable the magnetometer
		writeMagReg(LSM9DS1_CTRL_REG1_M, 0b10011100);   // Temp compensation enabled,Low power mode mode,80Hz ODR
		writeMagReg(LSM9DS1_CTRL_REG2_M, 0b01000000);   // +/-12gauss
		writeMagReg(LSM9DS1_CTRL_REG3_M, 0b00000000);   // continuos update
		writeMagReg(LSM9DS1_CTRL_REG4_M, 0b00000000);   // lower power mode for Z axis
	}

	if (BerryIMUversion == 3){//For BerryIMUv3
		//Enable  gyroscope
		writeGyrReg(LSM6DSL_CTRL2_G,0b10011100);        // ODR 3.3 kHz, 2000 dps

		// Enable the accelerometer
		writeAccReg(LSM6DSL_CTRL1_XL,0b10011111);       // ODR 3.33 kHz, +/- 8g , BW = 400hz
		writeAccReg(LSM6DSL_CTRL8_XL,0b11001000);       // Low pass filter enabled, BW9, composite filter
		writeAccReg(LSM6DSL_CTRL3_C,0b01000100);        // Enable Block Data update, increment during multi byte read

		//Enable  magnetometer
		writeMagReg(LIS3MDL_CTRL_REG1, 0b11011100);     // Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
		writeMagReg(LIS3MDL_CTRL_REG2, 0b00100000);     // +/- 8 gauss
		writeMagReg(LIS3MDL_CTRL_REG3, 0b00000000);     // Continuous-conversion mode
	}
	printf ("\n####    IMU ENABLED    ####\n");
}

// ref: https://github.com/jremington/LSM9DS1-AHRS/tree/main/Mahony_AHRS
// VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto
//The filter will not produce sensible results if these are not correct

/*
static float q[4] = {1.0, 0.0, 0.0, 0.0};

#define Kp 50.0
#define Ki 0.0

//Gyro scale 245 dps convert to radians/sec and offsets
float Gscale = (M_PI / 180.0) * 0.00875; //245 dps scale sensitivity = 8.75 mdps/LSB
int G_offset[3] = {75, 31, 142};

//Accel scale 16457.0 to normalize
float A_B[3]
{ -133.33,   72.29, -291.92};

float A_Ainv[3][3]
{ {  1.00260,  0.00404,  0.00023},
  {  0.00404,  1.00708,  0.00263},
  {  0.00023,  0.00263,  0.99905}
};

//Mag scale 3746.0 to normalize
float M_B[3]
{ -922.31, 2199.41,  373.17};

//float M_Ainv[3][3]
{ {  1.04492,  0.03452, -0.01714},
  {  0.03452,  1.05168,  0.00644},
  { -0.01714,  0.00644,  1.07005}
};

// local magnetic declination in degrees
float declination = -14.84;

// vector math
float vector_dot(float a[3], float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
	float mag = sqrt(vector_dot(a, a));
	a[0] /= mag;
	a[1] /= mag;
	a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
	byte i;
	float temp[3];
	Gxyz[0] = Gscale * (imu.gx - G_offset[0]);
	Gxyz[1] = Gscale * (imu.gy - G_offset[1]);
	Gxyz[2] = Gscale * (imu.gz - G_offset[2]);

	Axyz[0] = imu.ax;
	Axyz[1] = imu.ay;
	Axyz[2] = imu.az;
	Mxyz[0] = imu.mx;
	Mxyz[1] = imu.my;
	Mxyz[2] = imu.mz;

	//apply accel offsets (bias) and scale factors from Magneto

	for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
	Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
	Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
	Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
	vector_normalize(Axyz);

	//apply mag offsets (bias) and scale factors from Magneto

	for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
	Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
	Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
	Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
	vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
	// Vector to hold integral error for Mahony method
	static float eInt[3] = {0.0, 0.0, 0.0};
	// short name local variable for readability
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;
	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of the reference vectors
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
		// Apply I feedback
		gx += Ki * eInt[0];
		gy += Ki * eInt[1];
		gz += Ki * eInt[2];
	}


	// Apply P feedback
	gx = gx + Kp * ex;
	gy = gy + Kp * ey;
	gz = gz + Kp * ez;


	//update quaternion with integrated contribution
	// small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
	gx = gx * (0.5*deltat); // pre-multiply common factors
	gy = gy * (0.5*deltat);
	gz = gz * (0.5*deltat);
	float qa = q1;
	float qb = q2;
	float qc = q3;
	q1 += (-qb * gx - qc * gy - q4 * gz);
	q2 += (qa * gx + qc * gz - q4 * gy);
	q3 += (qa * gy - qb * gz + q4 * gx);
	q4 += (qa * gz + qb * gy - qc * gx);
	// Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}
*/

#endif
