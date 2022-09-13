#include <MPU6050.h>
#include "fsl_debug_console.h"

float gyroCorrectedArray[4] = {131.0, 65.5, 32.8, 16.4};
float axelCorrectedArray[4] = {16384.0, 8192.0, 4096.0, 2048.0};
float angleValue[3] = {0};

//---------------------------------------------------------------------------------

void MPU6050_wake_up(MPU6050_t * base)
{
	write_register8(base, PWR_MGMT_1_REG, 0x00);
}

void MPU6050_initial(MPU6050_t * base)
{
	write_register8(base, SMPLRT_DIV_REG, 0x00); //0
	write_register8(base, GYRO_CONFIG_REG, base->gyro_range << 3);
	write_register8(base, ACCEL_CONFIG_REG, base->acc_range << 3);
}

uint8_t MPU6050_connection(MPU6050_t * base)
{
	uint8_t buff;
	read_data(base, WHO_AM_I, &buff, 1);
	return buff;
}

void MPU6050_interupt(MPU6050_t * base)
{
	uint8_t buff;

	read_data(base, INTERUPT_CONFIG, &buff, 1);

	// [7 6 5 4 | oth oth oth x]

	//0 => INT pin is active high.
	//1 => INT pin is active low.
	CLR_BIT(buff, 7);

	//0 => INT pin is configured as push-pull. //more popular
	//1 => INT pin is configured as open drain.
	CLR_BIT(buff, 6);

	//0 => INT pin emits a 50us long pulse.
	//1 => INT pin is held high until the interrupt is cleared
	CLR_BIT(buff, 5);

	// 0, interrupt status bits are cleared only by reading INT_STATUS (Register 58)
	//1, interrupt status bits are cleared on any read operation.
	CLR_BIT(buff, 4);

	write_register8(base, INTERUPT_CONFIG, buff);

	// [x x x oth | oth x x 1]

	read_data(base, INTERUPT_ENABLE, &buff, 1);

	//1 this bit enables the Data Ready interrupt, which occurs each
	//time a write operation to all of the sensor registers has been completed.

	SET_BIT(buff, 0);

	write_register8(base, INTERUPT_ENABLE, buff);
}

void MPU6050_filter(MPU6050_t * base)
{
	uint8_t buff;

	// [x x oth oth | oth 2 1 0] filter from 0 to 7 where 0 and 7 8Khz for gyro

	read_data(base, DLPF_CONFIG, &buff, 1);

	//dec 6 => 0b110 max filter

	CLR_BIT(buff, 0);
	SET_BIT(buff, 1);
	SET_BIT(buff, 2);

	write_register8(base, DLPF_CONFIG, buff);
}

void MPU6050_complementaryFilter(float * axel, float * gyro, float * output)
{
	// Accelerometer/Gyroscope input data x y z
	// The X-axis shows rotation around the Y-axis, and
	// the angling of the Y-axis shows (negative) rotation around the X-axis.
	// Correct angle calculation  x = -y and y = x

	float swap = axel[0];
	axel[0] = (-1) * axel[1];
	axel[1] = swap;

	//- - - - - - - - - - - - - AXEL ANGLE CALCULATION - - - - - - - - - - - - - - - - -
	float rad2deg = (180.0 / M_PI); //RADIANS_TO_DEGREES

	float x2 = pow(axel[0], 2);
	float y2 = pow(axel[1], 2);
	float z2 = pow(axel[2], 2);

	// ax = atan(x / sqrt(y*y + z*z))
	float axel_x = atan(axel[0] / sqrt(y2 + z2)) * rad2deg;

	// ay = atan(y / sqrt(x*x + z*z))
	float axel_y = atan(axel[1] / sqrt(x2 + z2)) * rad2deg;

	// az = atan(sqrt(x*x + y*y) / z);
	float axel_z = atan(sqrt(x2 + y2) / axel[2]) * rad2deg;

	//- - - - - - - - - - - - - GYRO ANGLE CALCULATION - - - - - - - - - - - - - - - - -

	//(Gyroscope Angle) = (Last Measured Filtered Angle) + (New Measured Filtered Angle) x dt
	//angle value = previous angle value
	float gyro_x = angleValue[0] + gyro[0] * dt;
	float gyro_y = angleValue[1] + gyro[1] * dt;
	float gyro_z = angleValue[2] + gyro[2] * dt;

	//- - - - - - - - - - - - - filter ANGLE CALCULATION - - - - - - - - - - - - - - - - -

	if(((axel_x != axel_x) || (axel_y != axel_y) || (axel_z != axel_z)) || ((gyro_x != gyro_x) || (gyro_y != gyro_y) || (gyro_z != gyro_z)))
	{
		return;
	}

	//Filtered Angle = Alpha x (Gyroscope Angle) +  Beta x (Accelerometer Angle)
	angleValue[0] = alfa * gyro_x + beta * axel_x;
	angleValue[1] = alfa * gyro_y + beta * axel_y;
	angleValue[2] = alfa * gyro_z + beta * axel_z;


	for(int i = 0; i< 3; i++){
		output[i] = angleValue[i];
	}

}

//------------------------------ READ RAW DATA --------------------------------------------------

void MPU6050_readRawAxel(MPU6050_t * base, float * data)
{
	uint8_t buff[6];
	read_data(base, ACCEL_XOUT_H_REG, buff, 6);

	int16_t buff16 = 0;

	buff16 = ((int16_t)buff[0])<<8 | buff[1];
	data[0] = (float)buff16;

	buff16 = ((int16_t)buff[2])<<8 | buff[3];
	data[1] = (float)buff16;

	buff16 = ((int16_t)buff[4])<<8 | buff[5];
	data[2] = (float)buff16;
}

void MPU6050_readRawGyro(MPU6050_t * base, float * data)
{
	uint8_t buff[6];
	read_data(base, GYRO_XOUT_H_REG, buff, 6);

	int16_t buff16 = 0;

	buff16 = ((int16_t)buff[0])<<8 | buff[1];
	data[0] = (float)buff16;

	buff16 = ((int16_t)buff[2])<<8 | buff[3];
	data[1] = (float)buff16;

	buff16 = ((int16_t)buff[4])<<8 | buff[5];
	data[2] = (float)buff16;
}

void MPU6050_readRawTemp(MPU6050_t * base, float * data)
{
	uint8_t buff[2];
	read_data(base, TEMP_OUT_H , buff, 2);

	int16_t buff16 = 0;

	buff16 = ((int16_t)buff[0])<<8 | buff[1];
	data[0] = (float)buff16;

}

void MPU6050_readRawAll(MPU6050_t * base, float * data)
{
	uint8_t buff[14];
	read_data(base, ACCEL_XOUT_H_REG, buff, 14);

	int16_t buff16 = 0;

	//axel
	buff16 = ((int16_t)buff[0])<<8 | buff[1];
	data[0] = (float)buff16;

	buff16 = ((int16_t)buff[2])<<8 | buff[3];
	data[1] = (float)buff16;

	buff16 = ((int16_t)buff[4])<<8 | buff[5];
	data[2] = (float)buff16;

	//temp
	buff16 = ((int16_t)buff[6])<<8 | buff[7];
	data[3] = (float)buff16;

	//gyro
	buff16 = ((int16_t)buff[8])<<8 | buff[9];
	data[4] = (float)buff16;

	buff16 = ((int16_t)buff[10])<<8 | buff[11];
	data[5] = (float)buff16;

	buff16 = ((int16_t)buff[12])<<8 | buff[13];
	data[6] = (float)buff16;

}

//-------------------------------- READ CORECTED DATA ------------------------------------------------

void MPU6050_correctAxel(MPU6050_t * base, float * data)
{
	data[0] = (data[0] / axelCorrectedArray[base->acc_range]);
	data[1] = (data[1] / axelCorrectedArray[base->acc_range]);
	data[2] = (data[2] / axelCorrectedArray[base->acc_range]);
}

void MPU6050_correctGyro(MPU6050_t * base, float * data)
{
	data[0] = (data[0] / gyroCorrectedArray[base->gyro_range]);
	data[1] = (data[1] / gyroCorrectedArray[base->gyro_range]);
	data[2] = (data[2] / gyroCorrectedArray[base->gyro_range]);
}

void MPU6050_correctTemp(MPU6050_t * base, float * data)
{
	data[0] = (data[0]/340 + 36.53);
}

void MPU6050_correctAll(MPU6050_t * base, float * data)
{
	//accel
	data[0] = (data[0] / axelCorrectedArray[base->acc_range]);
	data[1] = (data[1] / axelCorrectedArray[base->acc_range]);
	data[2] = (data[2] / axelCorrectedArray[base->acc_range]);

	//temp
	data[3] = (data[3] / 340 + 36.53);

	//gyro
	data[4] = (data[4] / gyroCorrectedArray[base->gyro_range]);
	data[5] = (data[5] / gyroCorrectedArray[base->gyro_range]);
	data[6] = (data[6] / gyroCorrectedArray[base->gyro_range]);
}

//--------------------------------------- I2C READ ------------------------------------------

void read_data(MPU6050_t * base, uint8_t addr, uint8_t *value, uint8_t len)
{
	if (kStatus_Success == I2C_MasterStart(base->i2c, base->addr, kI2C_Write)) {

		I2C_MasterWriteBlocking(base->i2c, &addr, 1, kI2C_TransferNoStopFlag);
		I2C_MasterRepeatedStart(base->i2c, base->addr, kI2C_Read);
		I2C_MasterReadBlocking(base->i2c, value, len, kI2C_TransferDefaultFlag);
		I2C_MasterStop(base->i2c);
	}

}

void write_register8(MPU6050_t * base, uint8_t addr, uint8_t value) {

	uint8_t tx_buff[2]={addr, value};

	if (kStatus_Success == I2C_MasterStart(base->i2c, base->addr, kI2C_Write)) {

		I2C_MasterWriteBlocking(base->i2c, tx_buff, 2, kI2C_TransferDefaultFlag);
		I2C_MasterStop(base->i2c);

	}
}





