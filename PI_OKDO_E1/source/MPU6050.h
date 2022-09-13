#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#include "fsl_i2c.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#include <math.h>

//---------------------------------------------------------------------------------
//  struct
//---------------------------------------------------------------------------------

////dt and t in second
#define dt (0.001f)		//dt = sampling rate 1/100 * 1khz (main sample rate) => 10hz => f = 1/t => 10 = 1/t => 10*t = 1 => t = 0.1 [s] => 100 [ms]
#define t (1.0f)			//time constant greater than timescale of typical accelerometer noise (tau), czas pojawienie się zakóceniaw

#define alfa (t/(t+dt))			// 0.(90) alpha = t/(t + dt) "t = tau" gyro value filter =>  // 0.98
#define beta (1- alfa)			// 0.(09) beta = (1-alfa) axel value filter //0.02

typedef enum {
  MPU6050_ACC_RANGE_2G,  // +/- 2g (default)
  MPU6050_ACC_RANGE_4G,  // +/- 4g
  MPU6050_ACC_RANGE_8G,  // +/- 8g
  MPU6050_ACC_RANGE_16G // +/- 16g
} acc_range_t;


typedef enum {
  MPU6050_GYR_RANGE_250,  // +/- 250 deg/s (default)
  MPU6050_GYR_RANGE_500,  // +/- 500 deg/s
  MPU6050_GYR_RANGE_1000, // +/- 1000 deg/s
  MPU6050_GYR_RANGE_2000  // +/- 2000 deg/s
} gyro_range_t;

typedef struct {
    uint8_t addr;
    I2C_Type* i2c;
    acc_range_t acc_range;
	gyro_range_t gyro_range;
} MPU6050_t;

#define GET_BIT(k, n) (k & (1 << (n)))
#define SET_BIT(k, n) (k |= (1 << (n)))
#define CLR_BIT(k, n) (k &= ~(1 << (n)))

//---------------------------------------------------------------------------------
// macro register
//---------------------------------------------------------------------------------

#define WHO_AM_I 			(0x75) 			//WHO_AM_I register - address register
#define ADDRES 				(0x68)			//ADDRES hex - WHO_AM_I register response

#define PWR_MGMT_1_REG 		(0X6B)			//Power Management 1 Register Data = 0x0
#define SMPLRT_DIV_REG 		(0x19) 			//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
											//where Gyroscope Output Rate = 8kHz when the DLPF is disabled
											//(DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled

#define GYRO_CONFIG_REG 	(0x1B)			//Gyroscope Configuration Register Data range
#define ACCEL_CONFIG_REG 	(0x1C)			//Accelerometer Configuration Register Data range

#define ACCEL_XOUT_H_REG 	(0x3B)			//Accelerometer Measurements register address
#define GYRO_XOUT_H_REG 	(0x43)			//Temperature Measurements register address
#define TEMP_OUT_H 			(0x41)			//Gyroscope Measurements register address

#define INTERUPT_CONFIG		(0x37)			//INT Pin / Bypass Enable Configuration
#define	INTERUPT_ENABLE		(0x38)			//Interrupt Enable
#define INTERUPT_STATUS		(0x3A)			//Interrupt Status

#define DLPF_CONFIG			(0x1A)			//Digital Low Pass Filter

//---------------------------------------------------------------------------------
// function declaration
//---------------------------------------------------------------------------------

//wake up
void MPU6050_wake_up(MPU6050_t * base);

//initial all registers
void MPU6050_initial(MPU6050_t * base);

//test conection
uint8_t MPU6050_connection(MPU6050_t * base);

// seting up a interupt on data ready ~= 1khz
void MPU6050_interupt(MPU6050_t * base);

// seting filter
void MPU6050_filter(MPU6050_t * base);

//complementary filter

void MPU6050_complementaryFilter(float * axel, float * gyro, float * output);

//---------------------------------------------------------------------------------

//read axel
void MPU6050_readRawAxel(MPU6050_t * base, float * ptr);

//read temp
void MPU6050_readRawTemp(MPU6050_t * base, float * ptr);

//read gyro
void MPU6050_readRawGyro(MPU6050_t * base, float * ptr);

//read all data register in order
void MPU6050_readRawAll(MPU6050_t * base, float * ptr);

//---------------------------------------------------------------------------------

//read axel
void MPU6050_correctAxel(MPU6050_t * base, float * ptr);

//read temp
void MPU6050_correctTemp(MPU6050_t * base, float * ptr);

//read gyro
void MPU6050_correctGyro(MPU6050_t * base, float * ptr);

//read all data register in order
void MPU6050_correctAll(MPU6050_t * base, float * ptr);

//---------------------------------------------------------------------------------

//read register i2c
void read_data(MPU6050_t * base, uint8_t addr, uint8_t *value, uint8_t len);

//write register i2c
void write_register8(MPU6050_t * base, uint8_t addr, uint8_t value);

#endif /* MPU6050_H_ */
