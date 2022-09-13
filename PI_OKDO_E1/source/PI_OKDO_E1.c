#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC55S69_cm33_core0.h"
#include "fsl_debug_console.h"
#include "fsl_power.h"

#include "arm_math.h"
#include "lcd.h"
#include "MPU6050.h"

#define dataSize 4800
#define avarage (dataSize / LCD_WIDTH)

#define c_mpuInit 		rgb(128, 255, 255)
#define c_lcdInit 		rgb(255, 128, 255)

#define c_background	rgb(0, 0, 0)
#define c_info 			rgb(255, 255, 255)
#define c_angle 		rgb(255, 255, 0)

#define impulsePerRotation (90.f * 34.f)
#define intervalToSecond (1000.f * 60.f)

#define zero_angle 0

#define readENCA_ST (GPIO_PinRead(BOARD_INIT_ENC_ENCA_ST_GPIO, BOARD_INIT_ENC_ENCA_ST_PORT, BOARD_INIT_ENC_ENCA_ST_PIN))
#define readENCB_ST (GPIO_PinRead(BOARD_INIT_ENC_ENCR_ST_GPIO, BOARD_INIT_ENC_ENCR_ST_PORT, BOARD_INIT_ENC_ENCR_ST_PIN))
#define readBUTTON 	(GPIO_PinRead(BOARD_INIT_BUTTON_GPIO_GPIO, BOARD_INIT_BUTTON_GPIO_PORT, BOARD_INIT_BUTTON_GPIO_PIN))

void setEngineRight(bool front, bool back, uint8_t pwm);
void setEngineLeft(bool front, bool back, uint8_t pwm);

typedef struct{

	int32_t position;
	float32_t speed;

	bool status;
	float32_t setSpeed;
	float32_t calcPWM;

	bool front;
	bool back;
	float32_t pwmDuty;

	void (*set_ptr)(bool, bool, uint8_t);

	float32_t error;
	arm_pid_instance_f32 coef;
}Engine_t;

typedef struct{
	MPU6050_t config;
	float32_t angleX, angleY, angleZ, temp, angle_error, calcSpeed;
	arm_pid_instance_f32 coef;
}mpu6050_t;

mpu6050_t mpu6050 = {{ADDRES, MPU6050_I2C_PERIPHERAL, MPU6050_GYR_RANGE_2000, MPU6050_ACC_RANGE_16G}, 0,0,0,0,0,0};

Engine_t engineLeft = {0, 0, false, 0, 0, false, false, 0, &setEngineLeft, 0};
Engine_t engineRight= {0, 0, false, 0, 0, false, false, 0, &setEngineRight, 0};

lpadc_conv_result_t g_LpadcResultConfigStruct;
volatile uint16_t adcBattery = 0;

volatile int plotDataIndex = 0;
float dataBuffer[dataSize] = {0};
float plotBuffer[LCD_WIDTH] = {0};
volatile float dataMin = 0, dataMax = 0, dataAvg = 0, step = 0;
char textBuffer[50] = {0};

float mpuBuffer[7] = {0};

volatile float bat_v = 0, bat_adc = 0, bat_pre = 0;

void setEngineLeft(bool front, bool back, uint8_t pwm){

	GPIO_PinWrite(PBOARD_INIT_ENGINE_AIN_L_GPIO, PBOARD_INIT_ENGINE_AIN_L_PORT, PBOARD_INIT_ENGINE_AIN_L_PIN, front);
	GPIO_PinWrite(PBOARD_INIT_ENGINE_AIN_R_GPIO, PBOARD_INIT_ENGINE_AIN_R_PORT, PBOARD_INIT_ENGINE_AIN_R_PIN, back);

	CTIMER_UpdatePwmDutycycle(PWM_TIMER_PERIPHERAL, PWM_TIMER_PWM_PERIOD_CH, PWM_TIMER_PWM_1_CHANNEL, pwm);
}

void setEngineRight(bool front, bool back, uint8_t pwm){

	GPIO_PinWrite(PBOARD_INIT_ENGINE_BIN_L_GPIO, PBOARD_INIT_ENGINE_BIN_L_PORT, PBOARD_INIT_ENGINE_BIN_L_PIN, back);
	GPIO_PinWrite(PBOARD_INIT_ENGINE_BIN_R_GPIO, PBOARD_INIT_ENGINE_BIN_R_PORT, PBOARD_INIT_ENGINE_BIN_R_PIN, front);

	CTIMER_UpdatePwmDutycycle(PWM_TIMER_PERIPHERAL, PWM_TIMER_PWM_PERIOD_CH, PWM_TIMER_PWM_2_CHANNEL, pwm);
}

void simpleInitPID(arm_pid_instance_f32 * obj, float32_t p, float32_t i, float32_t d)
{
	obj->Kp = p;
	obj->Ki = i;
	obj->Kd = d;

	arm_pid_init_f32(obj, 1);
}

void CB_MPU6050(pint_pin_int_t pintr ,uint32_t pmatch_status)
{
	MPU6050_readRawAll(&mpu6050.config, mpuBuffer);
	MPU6050_correctAll(&mpu6050.config, mpuBuffer);
	MPU6050_complementaryFilter(&mpuBuffer[0], &mpuBuffer[4],  &mpu6050.angleX);
}

void CB_ENCA(pint_pin_int_t pintr ,uint32_t pmatch_status)
{
	if(readENCA_ST){
		engineLeft.position--;
	}
	else{
		engineLeft.position++;
	}
}

void CB_ENCB(pint_pin_int_t pintr ,uint32_t pmatch_status)
{
	if(readENCB_ST)
	{
		engineRight.position--;
	}else
	{
		engineRight.position++;
	}
}

void CB_BUTTON(pint_pin_int_t pintr ,uint32_t pmatch_status){

	for(volatile int i = 0; i < 100000; i++){};

	if(readBUTTON){

		engineLeft.status = false;
		setEngineLeft(0, 0, 0);

		engineRight.status = false;
		setEngineRight(0, 0, 0);


		if(plotDataIndex > dataSize)
		{
			for(int i = 0; i< LCD_WIDTH; i++){
				PRINTF("%.3f, ", plotBuffer[i]);
			}

			PRINTF("\r\n");
		}
	}
}

void CB_ADC(void)
{
	LPADC_GetConvResult(ADC0, &g_LpadcResultConfigStruct, 0U);
	adcBattery = g_LpadcResultConfigStruct.convValue;

	bat_v = (adcBattery / 65535.0) * 14.8;
	bat_adc = (adcBattery / 65535.0) * 3.3;
	bat_pre = (bat_adc / 3.3) * 100.00;

}

void engineAngleUpdate()
{
	mpu6050.angle_error = zero_angle - mpu6050.angleX;
	mpu6050.calcSpeed = arm_pid_f32(&mpu6050.coef, mpu6050.angle_error);

	engineLeft.setSpeed = 	mpu6050.calcSpeed;
	engineRight.setSpeed = 	mpu6050.calcSpeed;
}

void engineCalculationUpdate(Engine_t * engine)
{
	engine->speed = (engine->position / impulsePerRotation) * intervalToSecond;
	engine->position = 0.0;
}

void enginePowerUpdate(Engine_t * engine)
{
	engine->error = (engine->setSpeed  - engine->speed);
	engine->calcPWM = arm_pid_f32(&engine->coef, engine->error);

	engine->pwmDuty = 	(engine->status ? engine->calcPWM : 0.0);

	engine->pwmDuty = ((engine->pwmDuty 	> 100) ? (100) : (engine->pwmDuty ));
	engine->pwmDuty = ((engine->pwmDuty 	< -100) ? (-100) : (engine->pwmDuty ));

	engine->front = (engine->pwmDuty > 0);
	engine->back = (engine->pwmDuty < 0);

	engine->set_ptr(engine->front, engine->back, abs(engine->pwmDuty));
}

void CB_PWM(uint32_t flags)
{
	if (plotDataIndex < dataSize)
	{
		//dataBuffer[plotDataIndex] = engineRight.speed; 		// engine
		dataBuffer[plotDataIndex] = mpu6050.angleX; 		// angle
		plotDataIndex++;
	}

	engineAngleUpdate();

	engineCalculationUpdate(&engineLeft);
	engineCalculationUpdate(&engineRight);

	enginePowerUpdate(&engineLeft);
	enginePowerUpdate(&engineRight);
}


float findMax(float * data)
{
	float max = data[0];

	for (int i = 0; i < dataSize; i++) {
		if (data[i] > max) {
			max = data[i];
		}
	}

	return max;
}

float findMin(float * data) {
	float min = data[0];

	for (int i = 0; i < dataSize; i++) {
		if (data[i] < min) {
			min = data[i];
		}
	}

	return min;
}

float findAvg(float * data) {
	float avg = 0;
	for (int i = 0; i < dataSize; i++) {
		avg = avg + data[i];
	}

	avg = avg / dataSize;

	return avg;
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return ((((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min);
}

void plotDataCalculation(float * data, float setPoint)
{
	float avg = 0;

	for (int i = 0; i < LCD_WIDTH; i++){

		for(int j = 0; j < avarage; j++){
			avg = avg + data[i * avarage + j];
		}

		avg = avg / avarage;
		plotBuffer[i] =  avg;
		avg = 0;
	}

	float scaleMin = findMin(plotBuffer);
	scaleMin = ((scaleMin < setPoint) ? scaleMin : setPoint);

	float scaleMax = findMax(plotBuffer);
	scaleMax = ((scaleMax > setPoint) ? scaleMax : setPoint);

	for(int i = 0; i < LCD_WIDTH; i++){
		plotBuffer[i] = map(plotBuffer[i], scaleMin, scaleMax, 0, (LCD_HEIGHT - 1));
	}

	step = map(setPoint, scaleMin, scaleMax, 0, (LCD_HEIGHT - 1));

	dataMin = findMin(data);
	dataMax = findMax(data);
	dataAvg = findAvg(data);
}

void plotGraph(uint16_t plotColor, uint16_t targetColor)
{

	for(int i = 0; i < LCD_WIDTH - 1; i++){
		LCD_Draw_Line(i, (LCD_HEIGHT - 1) - plotBuffer[i], (i + 1), (LCD_HEIGHT - 1) - plotBuffer[i + 1], plotColor);
	}

	LCD_Draw_Line(0, (LCD_HEIGHT - 1) - step, (LCD_WIDTH - 1), (LCD_HEIGHT - 1) - step, targetColor);
}

void plotData()
{
	sprintf(textBuffer, "Bat:%.1fV(%.1fV) %.1f%%", bat_v, bat_adc, bat_pre);
	LCD_Puts(0, 0, textBuffer, c_info);

	//	sprintf(textBuffer, "P%.3f I%.3f D%.3f", engineRight.coef.Kp, engineRight.coef.Ki, engineRight.coef.Kd);
	//	LCD_Puts(0, 96, textBuffer, c_info);

	sprintf(textBuffer, "P%.3f I%.3f D%.3f", mpu6050.coef.Kp, mpu6050.coef.Ki, mpu6050.coef.Kd);
	LCD_Puts(0, 96, textBuffer, c_info);

	if (plotDataIndex == dataSize)
	{
		//plotDataCalculation(dataBuffer, engineRight.setSpeed);

		plotDataCalculation(dataBuffer, mpu6050.angleX);

		plotDataIndex++;
	}
	else if(plotDataIndex > dataSize)
	{
		plotGraph(c_angle, c_info);

		sprintf(textBuffer, "L%.3f A%.3f H%.3f", dataMin, dataAvg, dataMax);
		LCD_Puts(0, 104, textBuffer, c_info);
	}

	//	sprintf(textBuffer, "S:%.3f A%.3f", engineRight.setSpeed, engineRight.speed);
	//	LCD_Puts(0, 112, textBuffer, c_info);
	//
	//	sprintf(textBuffer, "E%.3f C%.3f", engineRight.error, engineRight.calcPWM);
	//	LCD_Puts(0, 120, textBuffer, c_info);

	sprintf(textBuffer, "Z:%.3f X%.3f", (float)zero_angle, mpu6050.angleX);
	LCD_Puts(0, 112, textBuffer, c_info);

	sprintf(textBuffer, "E%.3f C%.3f", mpu6050.angle_error, mpu6050.calcSpeed);
	LCD_Puts(0, 120, textBuffer, c_info);

	//	sprintf(textBuffer, "X%.3f Y%.3f Z%.3f", mpu6050.angleX, mpu6050.angleY, mpu6050.angleZ);
	//	LCD_Puts(0, 120, textBuffer, c_info);
	//
	//	sprintf(textBuffer, "L: %d, %.3f", engineLeft.position, engineLeft.speed);
	//	LCD_Puts(0, 112, textBuffer, c_info);
	//
	//	sprintf(textBuffer, "R: %d, %.3f", engineRight.position, engineRight.speed);
	//	LCD_Puts(0, 120, textBuffer, c_info);

}

int main(void) {

	/* Disable LDOGPADC power down */
	POWER_DisablePD(kPDRUNCFG_PD_LDOGPADC);

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
#endif

	__disable_irq();

	for(volatile int i = 0; i < 10000000; i++){};

	simpleInitPID(&engineLeft.coef, 0.8, 0.08, 0.001);
	engineLeft.status = true;

	simpleInitPID(&engineRight.coef, 0.8, 0.075, 0.002);
	engineRight.status = true;

	LCD_Init(LCD_SPI_PERIPHERAL);
	LCD_Clear(c_lcdInit);
	LCD_GramRefresh();

	MPU6050_wake_up(&mpu6050.config);
	MPU6050_initial(&mpu6050.config);

	if(MPU6050_connection(&mpu6050.config) != ADDRES){
		LCD_Clear(c_mpuInit);
		LCD_GramRefresh();
	}

	MPU6050_interupt(&mpu6050.config);
	MPU6050_filter(&mpu6050.config);

	simpleInitPID(&mpu6050.coef, 35, 1, 0.3);

	mpu6050.angleX = zero_angle;

	plotDataIndex = 0;

	//	setEngineLeft(1, 0, 100);
	//	setEngineRight(1, 0, 100);

	//	engineLeft.setSpeed = 100;
	//	engineRight.setSpeed = 100;

	for(volatile int i = 0; i < 1000000; i++){};

	__enable_irq();

	PRINTF("Loop \r\n");

	while(1)
	{
		LCD_Clear(c_background);
		plotData();
		LCD_GramRefresh();
	}

	return 0 ;
}
