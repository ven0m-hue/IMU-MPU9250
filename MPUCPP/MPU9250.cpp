/*
 * MPU9250.cpp
 *
 *  Created on: 19-Mar-2022
 *      Author: Venom
 */

#include "MPU9250.h"

namespace IMU {


//TODO Constructor pending
//TODO Compile Fix Erros if any
//TODO Unit Testing
//TODO Organizing the code 

//Short hands for Ascale
typedef Ascale::AFS_2G   Ascale_2G;
typedef Ascale::AFS_4G   Ascale_4G;
typedef Ascale::AFS_8G   Ascale_8G;
typedef Ascale::AFS_16G  Ascale_16G;

//Short hands for Gscale
typedef Gscale::GFS_250DPS    Gscale_250;
typedef Gscale::GFS_500DPS    Gscale_500;
typedef Gscale::GFS_1000DPS   Gscale_1000;
typedef Gscale::GFS_2000DPS   Gscale_2000;

//Short hands for Ascale
typedef Mscale::MFS_14BITS		Mscale_14;
typedef Mscale::MFS_16BITS		Mscale_16;

MPU9250::MPU9250()
{
	// TODO call init function inside this constructor, probably define it in the main function
	Init(*this);

}

MPU9250::~MPU9250() {
	// TODO Auto-generated destructor stub
}

MPU9250::MPU9250(const MPU9250 &other)
{

	if(this != &other)
	{
		//Handle deep copy
		*I2Chandle = *(other.I2Chandle);
		*GPIO_INT_PIN = *(other.GPIO_INT_PIN);

		this->acc = other.acc;
		this->gyr = other.gyr;
		this->mag = other.mag;
		this->roll_offset = other.roll_offset;
		this->pitch_offset = other.pitch_offset;

	}

}

MPU9250& MPU9250::operator=(const MPU9250 &other) {
	if(this != &other)
	{
		//Handle deep copy
		*I2Chandle = *(other.I2Chandle);
		*GPIO_INT_PIN = *(other.GPIO_INT_PIN);

		this->acc = other.acc;
		this->gyr = other.gyr;
		this->mag = other.mag;
		this->roll_offset = other.roll_offset;
		this->pitch_offset = other.pitch_offset;
	}

}


bool MPU9250::Init(const MPU9250 &imu){

	/* 1.Reset all the sensors*/
	writeByte(imu.I2Chandle, MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
	HAL_Delay(10); // 1ms delay

	/*2. Power management and Crystal clock settings*/
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	/*3. Configure the accel and gyro
	 *   Set the sample rate and bandwidth  as 1KHz and 42 Hz @refer_datasheet pg 15
	 *   DLPF_CFG = b'11
	 *   Sample_rate = gyro_output_rate/(1 + SMPLRT_DIV)
	 */
	writeByte(imu.I2Chandle, MPU9250_ADDRESS, CONFIG, 0x03);
	// Using the 200Hz rate -> From the above calculation.
	writeByte(imu.I2Chandle, MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

	/*4.Gyro Scale selection
	 * 1. Clear the Gyro_config register
	 * 2. Set the Fchoice = b'11 aka f_choice_b = b'00 and clear the GFS
	 * 3. Select the gyro scale
	 *
	 * 5. Repeat the same sequence for the accel scale selection
	 */
	uint8_t rxData;

	//Gyro
	rxData = readByte(imu.I2Chandle, MPU9250_ADDRESS, GYRO_CONFIG);
	rxData &= ~(0x02); //celars fchoice
	rxData &= ~(0x18); //celars GFS
	rxData |= (Gscale << 3);
	writeByte(imu.I2Chandle, MPU9250_ADDRESS, GYRO_CONFIG, rxData);

	//Accel
	rxData = readByte(imu.I2Chandle, MPU9250_ADDRESS, ACCEL_CONFIG);
	rxData &= ~(0x18); //celars GFS
	rxData |= (Ascale << 3);
	writeByte(imu.I2Chandle, MPU9250_ADDRESS, ACCEL_CONFIG, rxData);

	/*
	 * Set accel sample rate @4KHz refer data_sheet pg 17
	 * Bw = 41Hz
	 */
	rxData = readByte(imu.I2Chandle, MPU9250_ADDRESS, ACCEL_CONFIG2);
	rxData &= ~(0x0F);
	rxData |= 0x03;
	writeByte(imu.I2Chandle, MPU9250_ADDRESS, ACCEL_CONFIG2, rxData);

	//Originally all the sensor are set to 1KHz, however refacotred using SMPLRT_DIV to 200hz

	/*6.Configure the interrupt pins
	 * Interrupt for rasing edge and clears on read
	 * @refer reference manual  pg 29
	 */
	writeByte(imu.I2Chandle, MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(imu.I2Chandle, MPU9250_ADDRESS, INT_ENABLE, 0x01);

	/*7.Configure the magnetometer*/
	AK8963_Init(imu.I2Chandle);
	this->AK8963_Init(imu.I2Chandle);
	//self_calibrate_accel_pressure(imu, 1000);

	return true;

}

void MPU9250::ReadAccel(MPU9250 &imu)
{
	uint8_t rawdata[6];
	HAL_I2C_Mem_Read(imu.I2Chandle, MPU9250_ADDRESS, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawdata, 6, MPU9250_I2C_TIMEOUT);

	int16_t accX = (int16_t)((int16_t)rawdata[0] << 8 | rawdata[1]);
	int16_t accY = (int16_t)((int16_t)rawdata[2] << 8 | rawdata[3]);
	int16_t accZ = (int16_t)((int16_t)rawdata[4] << 8 | rawdata[5]);

	//Follows the NED coordinate frame.
	imu.acc[0] =  accX * getScale(Ascale_2G);
	imu.acc[1] =  accY * getScale(Ascale_2G);
	imu.acc[2] = -accZ * getScale(Ascale_2G);

}


void MPU9250::ReadGyro(MPU9250 &imu)
{
	uint8_t rawdata[6];
	HAL_I2C_Mem_Read(imu->I2Chandle, MPU9250_ADDRESS, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawdata, 6, MPU9250_I2C_TIMEOUT);

	int16_t gyrX = (int16_t)((int16_t)rawdata[0] << 8 | rawdata[1]);
	int16_t gyrY = (int16_t)((int16_t)rawdata[2] << 8 | rawdata[3]);
	int16_t gyrZ = (int16_t)((int16_t)rawdata[4] << 8 | rawdata[5]);

	imu.gyr[0] =  gyrX * getScale(Gscale_250);
	imu.gyr[1] =  gyrY * getScale(Gscale_250);
	imu.gyr[2] =  gyrZ * getScale(Gscale_250);
}

void MPU9250::ReadMag(MPU9250 &imu)
{
	uint8_t rawdata[6];
	/*Wait for Mag to be ready*/
	if(readByte(imu->I2Chandle, MPU9250_ADDRESS, AK8963_ST1) & 0x01)
	{
		HAL_I2C_Mem_Read(imu->I2Chandle, MPU9250_ADDRESS, AK8963_XOUT_L, I2C_MEMADD_SIZE_8BIT, rawdata, 3, MPU9250_I2C_TIMEOUT);
		/*Check the Overflow flag in the SR of AK8963
		 * wait until it gets cleared
		 * refer @ reference manual pg 50*/
		if( !(readByte(imu->I2Chandle, MPU9250_ADDRESS, AK8963_ST2) & 0x08))
		{

			int16_t magX = (int16_t)((int16_t)rawdata[1] << 8 | rawdata[0]);
			int16_t magY = (int16_t)((int16_t)rawdata[3] << 8 | rawdata[2]);
			int16_t magZ = (int16_t)((int16_t)rawdata[5] << 8 | rawdata[4]);

			imu->mag[0] =  magX * getScale(Mscale_16);
			imu->mag[1] =  magY * getScale(Mscale_16);
			imu->mag[2] =  magZ * getScale(Mscale_16);

		}
	}

}

void MPU9250::writeByte(I2C_HandleTypeDef& hi2c, uint8_t Address, uint8_t subAddress, uint8_t data)
{
	uint8_t txData[] = {subAddress, data};
	HAL_I2C_Master_Transmit(hi2c, MPU9250_ADDRESS, txData, 2, MPU9250_I2C_TIMEOUT);
}

uint8_t MPU9250::readByte(I2C_HandleTypeDef& hi2c, uint8_t Address, uint8_t subAddress)
{
	uint8_t rxData[1];
	uint8_t txData[] = {subAddress};
	HAL_I2C_Master_Transmit(hi2c, MPU9250_ADDRESS, txData, 1, MPU9250_I2C_TIMEOUT);

	HAL_I2C_Master_Receive(hi2c, MPU9250_ADDRESS, rxData, 1, MPU9250_I2C_TIMEOUT);

	return rxData[0];
}


void MPU9250::AK8963_Init(I2C_HandleTypeDef& hi2c)
{
	/*1.Reset the Mag sensor*/
	writeByte(hi2c, MPU9250_ADDRESS, AK8963_CNTL, 0x00);
	HAL_Delay(1);
	/*2.Fuse rom access mode*/
	writeByte(hi2c, MPU9250_ADDRESS, AK8963_CNTL, 0x0F);
	HAL_Delay(1);
	/*3.Power doen Magnetometer*/
	writeByte(hi2c, MPU9250_ADDRESS, AK8963_CNTL, 0x00);
	HAL_Delay(1);
	/*4.Mscale enable the 16bit resolution mode
	 *  Enable continous mode data acquisition Mmode = b'0110 @refer data sheet
	 */
	writeByte(hi2c, MPU9250_ADDRESS, AK8963_CNTL, Mscale << (4 | Mmode));
	HAL_Delay(1);
}

double MPU9250::getScale(const uint8_t scale)
{
	double result = 0.0f;

	switch(scale)
	{

	/*
	 * Possible accelerometer scales (and their register bit settings) are:
	 * 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	 * From the data sheet, divide the raw value with the accurate scale factors to get the Ax, Ay, Az
	 * @refer datasheet pg 9
	 *
	 * Convert the acceleration value interms of acceleration due to gravity to make sense of the data.
	 */

	case Ascale::AFS_2G:
		  result = ((PHY_g)/(16384.0));
		  break;

	case Ascale::AFS_4G:
		result = ((PHY_g)/(8192));
		  break;

	case Ascale::AFS_8G:
		result = ((PHY_g)/(4096));
		  break;

	case Ascale::AFS_16G:
		result = ((PHY_g)/(2048));
		  break;

	/*
	 *  Possible gyro scales (and their register bit settings) are:
	 *  250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	 *  From the data sheet, divide the raw value with the accurate scale factors to get the Wx, Wy, Wz
	 *  @refer datasheet pg 8
	 *	The output unit is deg/sec
	 *  To get the output in rad/sec for future computation convert deg/s -> rad/s
	 */

	case Gscale::GFS_250DPS:
		result = ((PI)/( 180 * 131));
		break;

	case Gscale::GFS_500DPS:
		result = ((PI)/(180 * 65.5));
		break;

	case Gscale::GFS_1000DPS:
		result = ((PI)/(180 * 32.8));
		break;

	case Gscale::GFS_2000DPS:
		result = ((PI)/(180 * 16.4));
		break;


	/*
	 * Possible magnetometer scales (and their register bit settings) are:
	 * Mmode = 14 bit resolution (0) and 16 bit resolution (1)
	 */

	case Mscale::MFS_14BITS:
		result = ((10.0 * 4912.0)/(8190.0)); // Proper scale to return milliGauss
		break;

	case Mscale::MFS_16BITS:
		result = ((10.0 * 4912.0)/(32760.0)); // Proper scale to return milliGauss
		break;

	}

	return result;
}

} /* namespace IMU */
