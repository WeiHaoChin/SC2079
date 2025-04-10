/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "math.h"
#include "ICM20948.h"
#include "IMU.h"
#include "IMUFilter.h"
#include <string.h>
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "magcal.h"
#include "FSM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SERVO 230
#define MIN_SERVO 70
int tempflag=1;
#define MAX_SERVO_Angle 180
#define MIN_SERVO_Angle 0
#define WHEEL_DIAMETER 6.5    // 6.5 cm
volatile int Target_Distance=0;  // in cm
#define PI 3.14159265359
#define DEG_TO_RAD (PI/ 180.0)
float distanceTraveled = 0.0;  // Total distance traveled in cm
float delta_time_sec=0;
float gyro_offset = 0.0;
float current_gyro_reading = 0.0;
int calibrate,flagDone=0,flagReceived=0;
int count=0,reverse,adjustment=2;
// ir sensor
uint16_t iDistanceL = 0;
uint16_t iDistanceR = 0;
uint32_t start_time, end_time;

//#define Debug
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*Timer1 servo pwm
Timer2 left wheel encoder
Timer3 right wheel encoder
Timer4 ultrasonic sonic echo
Timer6 ultra sonic delay
Timer8 both wheel pwm */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
		.name = "oledTask",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
		.name = "sensorTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
		.name = "encoderTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for robotTask */
osThreadId_t robotTaskHandle;
const osThreadAttr_t robotTask_attributes = {
		.name = "robotTask",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */
//Ultrasonic
int tc1=0,tc2=0,echo=0;
float g_distanceUS=0,last_valid=0;//ultrasonic distance
double RPM_L=0,RPM_R=0;//Encoder

double RPS_L=0,RPS_R=0;

vec3 accel={};
vec3 gyro={};
vec3 mag={};
MagCalParams mag_params={-1.88,-18.15,-40.05,
		0.34,-1.02,0,
		1.02,0.34,0,
		0,0,0.23};

extern float magOffset[];
float q[4]={1,0,0,0};
float yaw=0,pitch,roll;
PIDController LMotorPID;
PIDController RMotorPID;

int servo_pwm=150;
int pwmValL,pwmValR;
float degree=0;
int angle;

#define RECEIVE_BUFFER_SIZE 4
#define Slight_Right 90
#define Slight_Left 100
#define Right 20
#define Center 95
#define Left 140 //PREVIOUSLY 140
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void StartOledTask(void *argument);
void Startsensor(void *argument);
void StartEncoderTask(void *argument);
void startrobotTask(void *argument);

/* USER CODE BEGIN PFP */
void delay_us(uint16_t us);
void servo_check(uint8_t* pwm);
void set_motor_pwm(int32_t L,int32_t R);
void set_servo_angle(uint8_t value);
void functionCheck();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[3]={0};
uint8_t buf[7]={0};

//SET both wheel direction 0BACK 1FORWARD
void Set_Motor_Direction(int foward_flag,int backward_flag) {
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, //AIn1 is forward AIn2 is backward
			((foward_flag) ? GPIO_PIN_SET : GPIO_PIN_RESET));
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin,
			((foward_flag) ? GPIO_PIN_RESET : GPIO_PIN_SET));
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, //BIn1 is forward BIn2 is backward
			((backward_flag) ? GPIO_PIN_RESET : GPIO_PIN_SET));
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin,
			((backward_flag) ? GPIO_PIN_SET : GPIO_PIN_RESET));
}
void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim6,0);
	while(__HAL_TIM_GET_COUNTER(&htim6)<us);

}
void resetYaw()
{	q[0]=1; q[1]=0; q[2]=0; q[3]=0;
yaw=0;
PID_Reset(&LMotorPID);
PID_Reset(&RMotorPID);
}
void updateYaw()//update yaw reading using filter
{
	static uint32_t prevtick=0;

	if(HAL_GetTick()-prevtick>1000L)//RST if never update for more than 1 second
	{prevtick=HAL_GetTick();return;}
	ICM20948_readAccelerometer_all(&hi2c1,0,ACCEL_SENS,&accel);
	ICM20948_readGyroscope_all(&hi2c1, 0, GYRO_SENS, &gyro);
	//ICM20948_readMagnetometer_all(&hi2c1,&mag);
	//magcal_adjust(&mag,&mag_params);

	//Madgwick Mahony
	MahonyQuaternionUpdate(accel.x*9.81f,accel.y*9.81f,accel.z*9.81f,
			gyro.x*DEG_TO_RAD,-gyro.y*DEG_TO_RAD,gyro.z*DEG_TO_RAD,
			(HAL_GetTick()-prevtick)*0.001f,q);

	prevtick=HAL_GetTick();
	yaw = GetYawFromQ(q);
}
void Backward(int target)
{

	//Ensure yaw is reseted before turning
	//int target=-85;//90
	//int target=25;//360
	do{
	Set_Motor_Direction(0,0);
	static uint8_t bTurn=1;
	if(bTurn&& target == 0)
	{
		set_servo_angle(Center);
		osDelay(350);
		resetYaw();
		bTurn=0;
	}
	updateYaw();
	//osDelay(10);
	LMotorPID.setpoint=4;
	RMotorPID.setpoint=4;
	if (yaw > target - 0.1f )
	{
		set_servo_angle(Slight_Left);
	}
	else if(yaw < target + 0.1f)
	{
		set_servo_angle(Slight_Right);
	}
//	if(Target_Distance- distanceTraveled <10)
//	{
//		LMotorPID.setpoint=3;
//		RMotorPID.setpoint=3;
//	}
	//	else if (yaw > target - 20.0f && yaw < target + 20.0f)
	//	{
	//	  LMotorPID.setpoint=0.5;
	//	  RMotorPID.setpoint=0.35;
	//	}
	int32_t L=(int32_t)PID_Update(&LMotorPID, RPS_L);
	int32_t R=(int32_t)PID_Update(&RMotorPID, RPS_R);
	set_motor_pwm(L, R);
	osDelay(10);
	}

	while(distanceTraveled< 10);

}

void Forward(int target)
{

	//Ensure yaw is reseted before turning
	//int target=-85;//90
	//int target=25;//360
	static uint8_t bTurn=1;
	if(bTurn&&target==0)
	{
		set_servo_angle(Center);
		osDelay(350);
		resetYaw();
		bTurn=0;
	}
	updateYaw();
	//osDelay(10);
	LMotorPID.setpoint=4;
	RMotorPID.setpoint=4;

	if (yaw > target - 0.1f )
	{
		set_servo_angle(Slight_Right);
	}
	else if(yaw < target + 0.1f)
	{
		set_servo_angle(Slight_Left);
	}
	if(Target_Distance- distanceTraveled <10)
	{
		LMotorPID.setpoint=2.5;
		RMotorPID.setpoint=2.5;
	}
	//	else if (yaw > target - 20.0f && yaw < target + 20.0f)
	//	{
	//	  LMotorPID.setpoint=0.5;
	//	  RMotorPID.setpoint=0.35;
	//	}
	int32_t L=(int32_t)PID_Update(&LMotorPID, RPS_L);
	int32_t R=(int32_t)PID_Update(&RMotorPID, RPS_R);
	set_motor_pwm(L, R);
}
void Calibrate(void)
{
	set_servo_angle(Left);
	osDelay(750);
	set_servo_angle(Right);
	osDelay(750);
	set_servo_angle(Center);
	osDelay(750);
}
//void ForwardLeft(int target)
//{
//	//Ensure yaw is reseted before turning
//// -25 for 360degree        85 for 90degree
//	static uint8_t bTurn=1;
//		if(bTurn&& target != 0)
//		{
//			set_servo_angle(Left);
//			osDelay(750);
//			resetYaw();
//			bTurn=0;
//		}
//	updateYaw();
//
//	LMotorPID.setpoint=0.5;
//	RMotorPID.setpoint=2.5;
//
//	if (yaw > target - 2.0f && yaw < target + 2.0f)
//			  {
//				set_motor_pwm(0, 0);
//				PID_Reset(&LMotorPID);
//				PID_Reset(&RMotorPID);
//				set_servo_angle(Center);
//				bTurn=1;
//				flagDone=1;
//
//				return;
//			  }
//	else if (yaw > target - 20.0f && yaw < target + 20.0f)
//	{
//	  LMotorPID.setpoint=0.1;
//	  RMotorPID.setpoint=0.5;
//	}
//	int32_t L=(int32_t)PID_Update(&LMotorPID, RPS_L);
//	int32_t R=(int32_t)PID_Update(&RMotorPID, RPS_R);
//	set_motor_pwm(L, R);
//}
void BackRight(int target)
{

	//Ensure yaw is reseted before turning
	//int target=-85;//90
	//int target=25;//360
	static uint8_t bTurn=1;
	if(bTurn&& target != 0)
	{
		distanceTraveled=0;
		Target_Distance=1;
		while(distanceTraveled < Target_Distance)
		{
			Set_Motor_Direction(0,0);
			Backward(0);
		}
		Motor_Stop();
		set_servo_angle(Right);
		osDelay(750);
		resetYaw();
		bTurn=0;
	}
	updateYaw();
	//osDelay(10);
	LMotorPID.setpoint=4;
	RMotorPID.setpoint=1;

	if (yaw > target - 1.0f && yaw < target + 1.0f)
	{
		set_motor_pwm(0, 0);
		PID_Reset(&LMotorPID);
		PID_Reset(&RMotorPID);
		set_servo_angle(Center);
		osDelay(1000);
		distanceTraveled=0;
		Target_Distance=12;
		while(distanceTraveled < Target_Distance)
		{
			Set_Motor_Direction(0,0);
			Backward(0);
		}
		Motor_Stop();
		bTurn=1;
		flagDone=1;
		tempflag=0;
		count++;
		return;
	}
	else if (yaw > target - 25.0f && yaw < target + 25.0f)
	{
		LMotorPID.setpoint=0.5;
		RMotorPID.setpoint=0.5;//0.35
	}
	int32_t L=(int32_t)PID_Update(&LMotorPID, RPS_L);
	int32_t R=(int32_t)PID_Update(&RMotorPID, RPS_R);
	set_motor_pwm(L, R);
}
void BackLeft(int target)
{

	//Ensure yaw is reseted before turning
	//int target=-85;//90
	//int target=25;//360
	static uint8_t bTurn=1;
	if(bTurn&& target != 0)
	{
		distanceTraveled=0;
		Target_Distance=2;
		while(distanceTraveled < Target_Distance)
		{
			Set_Motor_Direction(0,0);
			Backward(0);
		}
		Motor_Stop();
		set_servo_angle(Left);
		osDelay(750);
		resetYaw();
		bTurn=0;
	}
	updateYaw();
	//osDelay(10);
	LMotorPID.setpoint=1;
	RMotorPID.setpoint=4;

	if (yaw > target - 1.0f && yaw < target + 1.0f)
	{
		set_motor_pwm(0, 0);
		PID_Reset(&LMotorPID);
		PID_Reset(&RMotorPID);
		set_servo_angle(Center);
		osDelay(750);
		distanceTraveled=0;
		Target_Distance=12;
		while(distanceTraveled < Target_Distance)
		{
			Set_Motor_Direction(0,0);
			Backward(0);
		}
		Motor_Stop();
		bTurn=1;
		flagDone=1;
		tempflag=0;
		distanceTraveled=0;
		count++;
		return;
	}
	else if (yaw > target - 25.0f && yaw < target + 25.0f)
	{
		LMotorPID.setpoint=0.5;
		RMotorPID.setpoint=0.5;//0.35
	}
	int32_t L=(int32_t)PID_Update(&LMotorPID, RPS_L);
	int32_t R=(int32_t)PID_Update(&RMotorPID, RPS_R);
	set_motor_pwm(L, R);
}
void ForwardLeft(int target)
{

	//Ensure yaw is reseted before turning
	//int target=-85;//90
	//int target=25;//360
	static uint8_t bTurn=1;
	if(bTurn&& target != 0)
	{
		distanceTraveled=0;
		Target_Distance=10;
		while(distanceTraveled < Target_Distance)
		{
			Set_Motor_Direction(1,1);
			Forward(0);
		}
		Motor_Stop();
		set_servo_angle(Left);
		osDelay(750);
		resetYaw();
		bTurn=0;
	}
	updateYaw();
	//osDelay(10);
	LMotorPID.setpoint=1;
	RMotorPID.setpoint=4;

	if (yaw > target - 1.0f && yaw < target + 1.0f)
	{
		set_motor_pwm(0, 0);
		PID_Reset(&LMotorPID);
		PID_Reset(&RMotorPID);
		set_servo_angle(Center);
		osDelay(750);
		bTurn=1;
		flagDone=1;
		//tempflag=0;
		//Calibrate();
		count++;
		return;
	}
	else if (yaw > target - 25.0f && yaw < target + 25.0f)
	{
		LMotorPID.setpoint=0.5;
		RMotorPID.setpoint=0.5;//0.35
	}
	int32_t L=(int32_t)PID_Update(&LMotorPID, RPS_L);
	int32_t R=(int32_t)PID_Update(&RMotorPID, RPS_R);
	set_motor_pwm(L, R);
}
void ForwardRight(int target)
{

	//Ensure yaw is reseted before turning
	//int target=-85;//90
	//int target=25;//360
	static uint8_t bTurn=1;
	if(bTurn&& target != 0)
	{
		distanceTraveled=0;
		Target_Distance=10;
		while(distanceTraveled < Target_Distance)
		{
			Set_Motor_Direction(1,1);
			Forward(0);
		}
		Motor_Stop();
		set_servo_angle(Right);
		osDelay(750);
		resetYaw();
		bTurn=0;
	}
	updateYaw();
	//osDelay(10);
	LMotorPID.setpoint=4;
	RMotorPID.setpoint=1;

	if (yaw > target - 1.0f && yaw < target + 1.0f)
	{
		set_motor_pwm(0, 0);
		PID_Reset(&LMotorPID);
		PID_Reset(&RMotorPID);
		set_servo_angle(Center);
		osDelay(750);
		bTurn=1;
		flagDone=1;
		distanceTraveled=0;
		Target_Distance=30;
		// tempflag=0;
		count++;
		return;
	}
	else if (yaw > target - 25.0f && yaw < target + 25.0f)
	{
		LMotorPID.setpoint=0.5;
		RMotorPID.setpoint=0.5;//0.35
	}
	int32_t L=(int32_t)PID_Update(&LMotorPID, RPS_L);
	int32_t R=(int32_t)PID_Update(&RMotorPID, RPS_R);
	set_motor_pwm(L, R);
}
void FLeft(int target)
{

	//	static uint8_t bTurn=1;
	//	if(bTurn)
	//	{
	//		set_servo_angle(Left);
	//		osDelay(300);
	//		//resetYaw();
	//		bTurn=0;
	//	}
	uint8_t bTurn=1;
	int32_t L=0,R=0;
	//osDelay(10);

	do
	{
		if(bTurn)
		{
			set_servo_angle(Left);
			osDelay(350);
			bTurn=0;
			//osDelay(300);
			resetYaw();
		} else{
			updateYaw();

			LMotorPID.setpoint=1;
			RMotorPID.setpoint=3;
			L=(int32_t)PID_Update(&LMotorPID, RPS_L);
			R=(int32_t)PID_Update(&RMotorPID, RPS_R);
			osDelay(10);
			set_motor_pwm(L, R);
		}

		//osDelay(10);

	}while(!(yaw > target - 2.0f && yaw < target + 2.0f));
	Motor_Stop();

	//yaw > target - 1.0f && yaw < target + 1.0f


	//	else if (yaw > target - 25.0f && yaw < target + 25.0f)
	//	{
	//		LMotorPID.setpoint=0.5;
	//		RMotorPID.setpoint=0.5;//0.35
	//	}


}
void FL(int target)
{

	//	static uint8_t bTurn=1;
	//	if(bTurn)
	//	{
	//		set_servo_angle(Left);
	//		osDelay(300);
	//		//resetYaw();
	//		bTurn=0;
	//	}
	uint8_t bTurn=1;
	int32_t L=0,R=0;
	//osDelay(10);

	do
	{
		if(bTurn)
		{
			set_servo_angle(Left);
			osDelay(350);
			bTurn=0;
			//osDelay(300);
			resetYaw();
		} else{
			updateYaw();

			LMotorPID.setpoint=1;
			RMotorPID.setpoint=4;
			L=(int32_t)PID_Update(&LMotorPID, RPS_L);
			R=(int32_t)PID_Update(&RMotorPID, RPS_R);
			osDelay(10);
			set_motor_pwm(L, R);
		}

		//osDelay(10);

	}while(!(yaw > target - 2.0f && yaw < target + 2.0f));
	Motor_Stop();
}
void FR(int target)
{

	//	static uint8_t bTurn=1;
	//	if(bTurn)
	//	{
	//		set_servo_angle(Left);
	//		osDelay(300);
	//		//resetYaw();
	//		bTurn=0;
	//	}
	uint8_t bTurn=1;
	int32_t L=0,R=0;
	//osDelay(10);

	do
	{
		if(bTurn)
		{
			set_servo_angle(Right);
			osDelay(350);
			bTurn=0;
			//osDelay(300);
			resetYaw();
		} else{
			updateYaw();

			LMotorPID.setpoint=4;
			RMotorPID.setpoint=1;
			L=(int32_t)PID_Update(&LMotorPID, RPS_L);
			R=(int32_t)PID_Update(&RMotorPID, RPS_R);
			osDelay(10);
			set_motor_pwm(L, R);
		}

		//osDelay(10);

	}while(!(yaw > target - 2.0f && yaw < target + 2.0f));
	Motor_Stop();
}
void FRight(int target)
{

	//	static uint8_t bTurn=1;
	//	if(bTurn)
	//	{
	//		set_servo_angle(Left);
	//		osDelay(300);
	//		//resetYaw();
	//		bTurn=0;
	//	}
	uint8_t bTurn=1;
	int32_t L=0,R=0;
	//osDelay(10);

	do
	{
		if(bTurn)
		{
			set_servo_angle(Right);
			osDelay(350);
			bTurn=0;
			//osDelay(300);
			resetYaw();
		} else{
			updateYaw();

			LMotorPID.setpoint=3;
			RMotorPID.setpoint=1;
			L=(int32_t)PID_Update(&LMotorPID, RPS_L);
			R=(int32_t)PID_Update(&RMotorPID, RPS_R);
			osDelay(10);
			set_motor_pwm(L, R);
		}

		//osDelay(10);

	}while(!(yaw > target - 2.0f && yaw < target + 2.0f));
	Motor_Stop();
}
void BIR(int target,int IR)
{
	Set_Motor_Direction(0,0);
	distanceTraveled=0;
	int pass=0,IRUsed;
	uint8_t bTurn=1;
	int32_t L=0,R=0;
	do
	{
		if(IR==0)
		{
			IRUsed=iDistanceL;
		}
		else if(IR==1){
			IRUsed=iDistanceR;
		}
		if(bTurn)
		{
			set_servo_angle(Center);
			bTurn=0;
			osDelay(250);
			resetYaw();
		}
		updateYaw();
		//osDelay(10);
		LMotorPID.setpoint=4;
		RMotorPID.setpoint=4;
		if(yaw > -0.1f)
		{
			set_servo_angle(Slight_Right);
		}
		else if(yaw < 0.1f)
		{
			set_servo_angle(Slight_Left);
		}

		L=(int32_t)PID_Update(&LMotorPID, RPS_L);
		R=(int32_t)PID_Update(&RMotorPID, RPS_R);

		set_motor_pwm(L, R);
		osDelay(10);
	}while(IRUsed>target);
	Motor_Stop();
	Set_Motor_Direction(1,1);
}
void FIR2(int target,int IR)
{
	distanceTraveled=0;
	int pass=0,IRUsed;
	uint8_t bTurn=1;
	int32_t L=0,R=0;
	do
	{
		if(IR==0)
		{
			IRUsed=iDistanceL;
		}
		else if(IR==1){
			IRUsed=iDistanceR;
		}
		if(bTurn)
		{
			set_servo_angle(Center);
			bTurn=0;
			osDelay(250);
			resetYaw();
		}
		updateYaw();
		//osDelay(10);
		LMotorPID.setpoint=3;
		RMotorPID.setpoint=3;
		if(yaw > -0.1f)
		{
			set_servo_angle(Slight_Right);
		}
		else if(yaw < 0.1f)
		{
			set_servo_angle(Slight_Left);
		}

		L=(int32_t)PID_Update(&LMotorPID, RPS_L);
		R=(int32_t)PID_Update(&RMotorPID, RPS_R);

		set_motor_pwm(L, R);
		osDelay(10);
	}while(IRUsed<target);
	//reverse=distanceTraveled-40;//-50;
	if(distanceTraveled<50){
		reverse=(distanceTraveled+10)/2;
		adjustment=0;
	}
	else if(distanceTraveled>70){
		reverse=(distanceTraveled)/2;
		adjustment=1;
	}
	Motor_Stop();
}

void FIR(int target,int IR)
{
	distanceTraveled=0;
	int pass=0,IRUsed;
	uint8_t bTurn=1;
	int32_t L=0,R=0;
	do
	{
		if(IR==0)
		{
			IRUsed=iDistanceL;
		}
		else if(IR==1){
			IRUsed=iDistanceR;
		}
		if(bTurn)
		{
			set_servo_angle(Center);
			bTurn=0;
			osDelay(250);
			resetYaw();
		}
		updateYaw();
		//osDelay(10);
		LMotorPID.setpoint=3;
		RMotorPID.setpoint=3;
		if(yaw > -0.1f)
		{
			set_servo_angle(Slight_Right);
		}
		else if(yaw < 0.1f)
		{
			set_servo_angle(Slight_Left);
		}

		L=(int32_t)PID_Update(&LMotorPID, RPS_L);
		R=(int32_t)PID_Update(&RMotorPID, RPS_R);

		set_motor_pwm(L, R);
		osDelay(10);
	}while(IRUsed<target);
	Motor_Stop();
}
//Forward til Ultra sonic reach target
void F(int target)
{

	uint8_t bTurn=1;
	int32_t L=0,R=0;

	do
	{
		if(bTurn)
		{
			set_servo_angle(Center);
			bTurn=0;
			osDelay(250);
			resetYaw();
		}
		updateYaw();
		//osDelay(10);
		LMotorPID.setpoint=3.5;
		RMotorPID.setpoint=3.5;
		if(yaw > -0.1f)
		{
			set_servo_angle(Slight_Right);
		}
		else if(yaw < 0.1f)
		{
			set_servo_angle(Slight_Left);
		}

		L=(int32_t)PID_Update(&LMotorPID, RPS_L);
		R=(int32_t)PID_Update(&RMotorPID, RPS_R);

		set_motor_pwm(L, R);
		osDelay(10);
	}while(g_distanceUS>target);

	Motor_Stop();
}
//Back using the Ultra sonic
void B(int target)
{

	uint8_t bTurn=1;
	int32_t L=0,R=0;
Set_Motor_Direction(0, 0);
	do
	{
		if(bTurn)
		{
			set_servo_angle(Center);
			bTurn=0;
			osDelay(250);
			resetYaw();
		}
		updateYaw();
		//osDelay(10);
		LMotorPID.setpoint=3;
		RMotorPID.setpoint=3;
		if(yaw > -0.1f)
		{
			set_servo_angle(Slight_Right);
		}
		else if(yaw < 0.1f)
		{
			set_servo_angle(Slight_Left);
		}

		L=(int32_t)PID_Update(&LMotorPID, RPS_L);
		R=(int32_t)PID_Update(&RMotorPID, RPS_R);

		set_motor_pwm(L, R);
		osDelay(10);
	}while(g_distanceUS<target);

	Motor_Stop();
	Set_Motor_Direction(1, 1);//set back to forward direction

}
void ForwardDistance(int target)
{
	distanceTraveled=0;
	Set_Motor_Direction(1, 1);
	uint8_t bTurn=1;
	int32_t L=0,R=0;

	do
	{
		if(bTurn)
		{
			set_servo_angle(Center);
			bTurn=0;
			osDelay(350);
			resetYaw();
		}
		updateYaw();
		//osDelay(10);
		LMotorPID.setpoint=4;
		RMotorPID.setpoint=4;
		if(yaw > -0.1f)
		{
			set_servo_angle(Slight_Right);
		}
		else if(yaw < 0.1f)
		{
			set_servo_angle(Slight_Left);
		}

		L=(int32_t)PID_Update(&LMotorPID, RPS_L);
		R=(int32_t)PID_Update(&RMotorPID, RPS_R);

		set_motor_pwm(L, R);
		osDelay(10);
	}while(distanceTraveled<target);

	Motor_Stop();
	Set_Motor_Direction(1, 1);//set back for forward direction

}
//move back by distance
void BackDistance(int target)
{
	distanceTraveled=0;
	Set_Motor_Direction(0, 0);
	uint8_t bTurn=1;
	int32_t L=0,R=0;

	do
	{
		if(bTurn)
		{
			set_servo_angle(Center);
			bTurn=0;
			osDelay(350);
			resetYaw();
		}
		updateYaw();
		//osDelay(10);
		LMotorPID.setpoint=4;
		RMotorPID.setpoint=4;
		if(yaw > -0.1f)
		{
			set_servo_angle(Slight_Right);
		}
		else if(yaw < 0.1f)
		{
			set_servo_angle(Slight_Left);
		}

		L=(int32_t)PID_Update(&LMotorPID, RPS_L);
		R=(int32_t)PID_Update(&RMotorPID, RPS_R);

		set_motor_pwm(L, R);
		osDelay(10);
	}while(distanceTraveled<target);

	Motor_Stop();
	Set_Motor_Direction(1, 1);//set back for forward direction

}

void set_motor_pwm(int32_t L,int32_t R)
{	//4k max pwm

	pwmValL=L;
	pwmValR=R;
	if(pwmValL<0)
		pwmValL=500;
	else if(pwmValL>5000)
		pwmValL=5000;

	if(pwmValR<0)
		pwmValR=500;
	else if(pwmValR>5000)
		pwmValR=5000;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValR);

	osDelay(1);
}
void set_servo_pwm(uint8_t value) //not in use
{
	servo_pwm=value;
	if(servo_pwm < MIN_SERVO)
	{
		servo_pwm = MIN_SERVO;
	}
	if(servo_pwm > MAX_SERVO)
	{
		servo_pwm = MAX_SERVO;
	}
	htim1.Instance->CCR4 = servo_pwm;
}
void set_servo_angle(uint8_t value)
{
	if (value > 180) value = 180;
	if (value < 0) value = 0;

	servo_pwm = 240 - ((value * (240 - 60)) / 180);
	/*if(servo_pwm < MIN_SERVO)
    {
    	servo_pwm = MIN_SERVO;
    }
    if(servo_pwm > MAX_SERVO)
    {
    	servo_pwm = MAX_SERVO;

    }*/
	angle=value;
	htim1.Instance->CCR4 = servo_pwm;

}


//PID
float Kp = 0.5;  // Proportional gain
float Ki = 0.3;  // Integral gain
float Kd = 0.1;  // Derivative gain

float previous_error_L = 0.0f;
float previous_error_R = 0.0f;
float integral_L = 0.0f;
float integral_R = 0.0f;

/*TASK 2 RELATED*/
void MoveTo10By10(void);
void FirstDir(void);
void SecondDir(void);
void GoLeft(void);
void GoRight(void);
void GoLeft2(void);
void GoRight2(void);
void MoveTo2nd(void);
void GoBackHome(void);
float Y1Distance=0;

//Environment mode
volatile uint8_t mode =0;// 0 for lab , 1 for outside
//static const void (*fn[])(int) = {F, FLeft, FRight,};

#define LeftState 0
#define RightState 1
#define Duncare 0

#define MoveToBlock         &fsm[0] //Phase 1
#define Wait1stDir			&fsm[1]
#define MoveLeft           	&fsm[2]
#define MoveRight          	&fsm[3]
#define MoveTo2				&fsm[4] //Phase 2 in front of 2nd obstacle
#define Wait2ndDir			&fsm[5]
#define MoveLeft2       	&fsm[6]
#define MoveRight2       	&fsm[7]
#define MoveTo3      		&fsm[8] //Phase 3 parking phase
//#define DONEWITHLYFE		&fsm[9]


State_t fsm[9] = {
    {MoveTo10By10,    {Wait1stDir, Wait1stDir}},    // MoveToBlock
	{FirstDir,    {MoveLeft, MoveRight}},    // Wait1stDir
    {GoLeft,  {MoveTo2, MoveTo2}},  // MoveLeft
    {GoRight,  {MoveTo2, MoveTo2}}, // MoveRight
	{MoveTo2nd,{Wait2ndDir,Wait2ndDir}}, // MoveTo2
	{SecondDir,  {MoveLeft2, MoveRight2}}, // Wait2ndDir
    {GoLeft2, {MoveTo3, MoveTo3}},  // MoveLeft2
    {GoRight2, {MoveTo3, MoveTo3}},  // MoveRight2
	{GoBackHome, {NULL, NULL}},  // MoveTo3

//    {moveTo2Function,	30,    {MoveLeft2, MoveRight2}},  // MoveTo2
//    {moveLeft2Function,	30,  {MoveTo3, MoveLeft2}},     // MoveLeft2
//    {moveRight2Function,30, {MoveTo3, MoveRight2}},    // MoveRight2
//    {moveTo3Function,	30,    {MoveTo3, MoveTo3}}        // MoveTo3
};
State_t *CurrentState=NULL;  // pointer to the current state
void MoveTo10By10()
{

		F(30);

		flagDone=1;
		flagReceived=0;
		HAL_UART_Transmit(&huart3,(uint8_t *)"ACK\r\n",5,0xFFFF);
		CurrentState=CurrentState->next[Duncare];//


//	if(aRxBuffer[0]=='L')
//	{
//		CurrentState=CurrentState->next[LeftState];
//	}
//	else if(aRxBuffer[0]=='R')
//		{
//		CurrentState=CurrentState->next[RightState];
//		}

}
void FirstDir()
{
	//DEBUG
//	if(mode==0)
//		CurrentState=CurrentState->next[LeftState];
//	else if(mode==1)
//		CurrentState=CurrentState->next[RightState];
	if(aRxBuffer[0]=='A')
		CurrentState=CurrentState->next[LeftState];
	else if(aRxBuffer[0]=='D')
			CurrentState=CurrentState->next[RightState];
	else if(aRxBuffer[0]=='X')//ggwp we ball
	{
		//RANDOM DIR AND WHACK TO BE DONE!!!!
	}
	osDelay(10);
	}
void SecondDir()
{
	//DEBUG
	//CurrentState=CurrentState->next[LeftState];
	//=CurrentState->next[RightState];
//	if(mode==0)
//		CurrentState=CurrentState->next[RightState];
//	else if(mode==1)
//		CurrentState=CurrentState->next[RightState];
	if(aRxBuffer[0]=='A')
		CurrentState=CurrentState->next[LeftState];
	else if(aRxBuffer[0]=='D')
			CurrentState=CurrentState->next[RightState];
	else if(aRxBuffer[0]=='X')//ggwp we ball
	{
		//RANDOM DIR AND WHACK TO BE DONE!!!!
	}
	osDelay(10);
	}
void GoLeft()
{
	FLeft(41.5);
	osDelay(100);
	HAL_UART_Transmit(&huart3,(uint8_t *)"LOL\r\n",5,0xFFFF);//Pascal stream
	FRight(-90);
	osDelay(100);
	BackDistance(5);
	osDelay(100);
	FLeft(46.5);

	CurrentState=CurrentState->next[Duncare];
	}
void GoRight()
{
	FRight(-41.5);
	osDelay(100);
	HAL_UART_Transmit(&huart3,(uint8_t *)"LOL\r\n",5,0xFFFF);//Pascal stream
	FLeft(90);
	osDelay(100);
	BackDistance(10);
	osDelay(100);
	FRight(-47.5);

	CurrentState=CurrentState->next[Duncare];
	}

//Do your adjustment to 2nd Obstacle if necessary
void MoveTo2nd()
{

	osDelay(100);
	memset(aRxBuffer,"0",sizeof(aRxBuffer));//reset for next direction
	count=g_distanceUS+40;
	if(g_distanceUS<30)
	{
		B(30);
		count=count+35;
	}
	else if(g_distanceUS>=30){
		F(40);
		count=count+35;
	}
	HAL_UART_Transmit(&huart3,(uint8_t *)"ACK\r\n",5,0xFFFF);
	osDelay(10);
	CurrentState=CurrentState->next[Duncare];//change to waiting dir2
	}

void GoLeft2()
{
	FLeft(84);
	BIR(400,1);
	FIR(400,1);
	BIR(400,1);
	FIR(400,1);
	FR(-161);
	osDelay(100);
	//ForwardDistance(10);
	FIR2(400,1); //0 for Left 1 for Right
	BIR(400,1);
	FIR(400,1);
	osDelay(100);
	FRight(-80);
	ForwardDistance(count-10);// coming back after a turn
	//FIR(600,1);
	FRight(-80);
	osDelay(100);
//	if(reverse>0){
//		ForwardDistance(reverse);
//	}
//	else{
//		BackDistance(-(reverse));
//	}
	if(adjustment==1){
		reverse=reverse-35;
		ForwardDistance(reverse);
	}
	else if(adjustment==0){
		BackDistance(reverse);
	}
	else if(adjustment==2){
		ForwardDistance(10);
	}
	osDelay(150);
	FLeft(81);
	CurrentState=CurrentState->next[Duncare];//change to waiting dir2
	}

void GoRight2()
{
	FRight(-84);
	BIR(400,0);
	FIR(400,0);
	BIR(400,0);
	FIR(400,0);
	osDelay(100);
	FL(161);
	osDelay(100);
	//ForwardDistance(10);
	FIR2(400,0); //0 for Left 1 for Right
	BIR(400,0);
	FIR(400,0);
	FLeft(80);
	ForwardDistance(count);
	//FIR(600,1);
	FLeft(80);
	osDelay(100);
//	if(reverse>0){
//		ForwardDistance(reverse);
//	}
//	else{
//		BackDistance(-(reverse));
//	}
	if(adjustment==1){
		reverse=reverse-30;
		ForwardDistance(reverse);
	}
	else if(adjustment==0){
		BackDistance(reverse);
	}
	else if(adjustment==2){
		ForwardDistance(10);
	}
	osDelay(150);
	FRight(-80);
	CurrentState=CurrentState->next[Duncare];//change to waiting dir2
	}
void GoBackHome()
{
	F(20);
	CurrentState=CurrentState->next[Duncare];//change to waiting dir2
	}
void Task2()
{
	//	LMotorPID.output_min=2000;
	//	RMotorPID.output_min=2000;
	//	LMotorPID.output_max=5000;
	//	RMotorPID.output_max=5000;
	float targetAngle =45;
	switch(count)
	{
	case 0:
		F(30);
		count++;
		break;
	case 1:
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==0)
		{updateYaw();
		count++;
		}
		break;
	case 2:		//turn
		FLeft(targetAngle);count++;
		//MoveAndTurn(targetAngle);
		break;
	case 3:	//turnback
		FRight(-(targetAngle*2));count++;
		//FLeft(targetAngle);count++;
		//MoveAndTurn(-(targetAngle-10.0f));
		break;
	case 4:	//turn center
		//FRight(-(targetAngle));
		FLeft(40);count++;
		//MoveAndTurn(0);
		break;
	case 5://FLeft(targetAngle);
		F(30);
		count++;
		break;
	case 6:	Motor_Stop();
	break;
	default:
		Motor_Stop();break;

	}

	//	if(count==1)
	//	{
	//	FLeft(41);
	//	FRight(-90);
	//	FLeft(43.5);
	//	}
	//		fn[count](30);
	//		if(count==1)
	//			fn[count](45);

	//	if(count==0&&g_distanceUS>30)
	//	{
	//		Set_Motor_Direction(1,1);
	//		Forward(0);
	//	}
	//	else if(g_distanceUS<30&&count==0)
	//	{
	//		Motor_Stop();
	//		count++;
	//	}
	//	else if(count==1)
	//	{
	//		FLeft(41.5);
	//	}
	//	else if(count==2)
	//	{
	//		FRight(-90);
	//	}
	//	else if(count==3)
	//	{
	//		FLeft(43.5);
	//	}
	//	else if(count==4&&g_distanceUS>30)
	//	{
	//		Set_Motor_Direction(1,1);
	//		Forward(0);
	//	}
	//	else if(g_distanceUS<30&&count==4)
	//	{
	//		Motor_Stop();
	//		count++;
	//	}
	//	else if(count==5)
	//	{
	//		FLeft(83);
	//	}
	//	else if(iDistanceR<15&&count==5)
	//	{
	//		Forward(0);
	//	}
	//	else if(iDistanceR>15&&count==5)
	//	{
	//		Motor_Stop();
	//	}
}

void functionCheck()
{//Please call this function after everything is initialise
	OLED_ShowString(10, 10, "Press\0");
	OLED_ShowString(10, 20, "User button\0");
	OLED_Refresh_Gram();
	while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==1);
	OLED_Clear();
	//oled
	//imu
	while(1){
		char text[15];
		IR_Left_Read();
		IR_Right_Read();
		snprintf(text, sizeof(text), "Right:%d", iDistanceR);
		OLED_ShowString(10, 10, text);
		snprintf(text, sizeof(text), "Left:%d", iDistanceL);
		OLED_ShowString(10, 20, text);
		OLED_Refresh_Gram();
	}
	char text[15];
	ICM20948_readGyroscope_all(&hi2c1, 0, GYRO_SENS,&accel);
	snprintf(text, sizeof(text), "X:%5.2f", accel.x);
	OLED_ShowString(10, 10, text);
	snprintf(text, sizeof(text), "Y:%5.2f", accel.y);
	OLED_ShowString(10, 20, text);
	snprintf(text, sizeof(text), "Z:%5.2f", accel.z);
	OLED_ShowString(10, 30, text);
	OLED_Refresh_Gram();
	//servo
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);//Start servo pwm timer
	HAL_Delay(1000);
	set_servo_pwm(50);
	HAL_Delay(2000);
	set_servo_pwm(230);
	HAL_Delay(2000);
	set_servo_pwm(140);

	HAL_Delay(2000);
	//motor Note: not moving? check cables
	uint16_t pwmValL=1200;
	uint16_t pwmValR=1200;
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	Set_Motor_Direction(1,1);
	HAL_Delay(1000);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValL);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValR);
	HAL_Delay(1000);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	HAL_Delay(2000);
	Set_Motor_Direction(0,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValL);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValR);
	HAL_Delay(1000);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	//ultrasonic
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_TIM3_Init();
	MX_TIM6_Init();
	MX_TIM1_Init();
	MX_TIM8_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */
	OLED_Init();
	ICM20948_init(&hi2c1,0,GYRO_SENS,ACCEL_SENS);

	OLED_ShowString(10, 10, "Press to Start\0");
	OLED_Refresh_Gram();
	while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)!=0);
	HAL_Delay(200);
	ICM20948_CalibrateAccel(&hi2c1, ACCEL_SENS, 250);
	ICM20948_CalibrateGyro(&hi2c1,GYRO_SENS, 250);
	OLED_Clear();

	HAL_TIM_Base_Start(&htim6);// for microseond delay
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);//Timer for ultra sonic
	HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,RECEIVE_BUFFER_SIZE);//Receive data from uart


	PID_Init(&LMotorPID, 3.0*100.0f, 25 *100.0f,0 , 0, 1000, 5000);
	PID_Init(&RMotorPID, 3.0*100.0f, 25 *100.0f,0 , 0, 1000, 5000);
	Set_Motor_Direction(1,1);//Keep forget put lmao



	//MagCalParams params={};
	//magcal_calc_params(&mag_params);
#ifdef Debug
	functionCheck();
#endif
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of oledTask */
	oledTaskHandle = osThreadNew(StartOledTask, NULL, &oledTask_attributes);

	/* creation of sensorTask */
	sensorTaskHandle = osThreadNew(Startsensor, NULL, &sensorTask_attributes);

	/* creation of encoderTask */
	encoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &encoderTask_attributes);

	/* creation of robotTask */
	robotTaskHandle = osThreadNew(startrobotTask, NULL, &robotTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 160;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 16-1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 16-1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 65535;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 7199;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
			|TRIG_Pin|LED3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           TRIG_Pin LED3_Pin */
	GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
			|TRIG_Pin|LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
	GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PD8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : SWITCH_Pin */
	GPIO_InitStruct.Pin = SWITCH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SWITCH_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Interrupt Serial callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	//prevent unused argument
	UNUSED(huart);
	snprintf(buf,sizeof(buf),"%5.2f",g_distanceUS);
	sprintf(buf, "%s\r\n", buf);
	if (strncmp(aRxBuffer, "RUSD",4) == 0)
	{
		HAL_UART_Transmit(&huart3, (uint8_t*)buf,7,0XFFFF);
	}
	//HAL_UART_Transmit(&huart3,(uint8_t *)aRxBuffer,4,0xFFFF);
	if(flagReceived !=1){
		flagReceived=1;
		distanceTraveled=0;
		HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,RECEIVE_BUFFER_SIZE);
	}
	else{

	}
	// servo_pwm  = atoi(aRxBuffer);
	//memset(aRxBuffer,0,sizeof(aRxBuffer));

}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//timer 3 for the ultrasonic echo
	if(htim==&htim4)
	{
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12)==1)//rising edge
		{
			tc1=HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);

		}
		else
		{
			tc2=HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
			if(tc2>tc1)
				echo = (tc2-tc1);
			else //tc2 overflow
				echo=((65536-tc1)+tc2);
		}g_distanceUS = (tc2 > tc1 ? (tc2 - tc1) : (65535 - tc1 + tc2)) * 0.034 / 2;
		if(g_distanceUS>400)
		{
			g_distanceUS=last_valid;
		}
		else{
			last_valid=g_distanceUS;
		}
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);
	if(HAL_GPIO_ReadPin(GPIOE, SWITCH_Pin)==0) {
		//set value for LAB
		mode=0;
	} else if(HAL_GPIO_ReadPin(GPIOE, SWITCH_Pin)==1) {
		//set value for OUTSIDE
		mode=1;
	}else{
		__NOP();
	}

}

void IR_Left_Read() {
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 10);
	iDistanceL = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);

	//	filtered_irreading = (FILTER_ALPHA2 * iDistanceL) + ((1 - FILTER_ALPHA2) * filtered_irreading);
	//	filtered_irreading_int = (int) filtered_irreading;
	//iDistanceL=(int)pow(10, -1.754*(log10((float) iDistanceL))+7.064);
	//iDistanceL=(int)(iDistanceL*-4.286e-03)+2.189e+01;
	iDistanceL=(int)163690.8 * pow(iDistanceL, -0.9039361);
}

void IR_Right_Read() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	iDistanceR = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	iDistanceR=(int)163690.8 * pow(iDistanceR, -0.9039361);
//	iDistanceR=(int)(iDistanceR*-4.286e-03)+2.189e+01;
}

void Motor_Stop()
{
	resetYaw();
	PID_Reset(&LMotorPID);
	PID_Reset(&RMotorPID);
	distanceTraveled=0;
	pwmValL = 0;
	pwmValR = 0;
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValL); // Stop left motor
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValR); // Stop right motor
	set_servo_angle(Center);
	osDelay(350);
	//reset_gyro_at_rest();
	//  count++;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOE,LED3_Pin);
		osDelay(1000);

	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
 * @brief Function implementing the oledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument)
{
	/* USER CODE BEGIN StartOledTask */
	char text[16]={};
	char temp[50]={};

	//	LMotorPID.Kp=5*100.0f;
	//	  	LMotorPID.Ki=15 *100.0f;
	//	  	RMotorPID.Kp=5*100.0f;
	//	  	RMotorPID.Ki=15 *100.0f;
	//	LMotorPID.setpoint = 1;
	//	RMotorPID.setpoint = 1;

	/* Infinite loop */
	for(;;)
	{
		snprintf(text, sizeof(text), "DistanceU:%.2f",  g_distanceUS);
		OLED_ShowString(10, 20, text);
		snprintf(text, sizeof(text), "degree :%5.2f", yaw);//BEFORE DEGREE
		OLED_ShowString(10, 30, text);
		OLED_Refresh_Gram();
		snprintf(text, sizeof(text), "Distance: %f m", distanceTraveled);
		OLED_ShowString(10, 10, text);
		snprintf(text, sizeof(text), "DistanceIRL:%d m", iDistanceL);
		OLED_ShowString(10, 40, text);
		snprintf(text, sizeof(text), "DistanceIRR %d m", iDistanceR);
		OLED_ShowString(10, 50, text);
		//snprintf(temp,sizeof(temp),"%.5f\r\n",yaw);

		//for debug pid
		//		int32_t L=(int32_t)PID_Update(&LMotorPID, RPS_L);
		//		int32_t R=(int32_t)PID_Update(&RMotorPID, RPS_R);
		//			set_motor_pwm(L, R);
		//		snprintf(temp,sizeof(temp),"%.2f,%.2f,%d\r\n",RPS_L,RPS_R,(int)LMotorPID.setpoint);
		//		HAL_UART_Transmit(&huart3, temp, sizeof(temp), 0xFFFF);
		//		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==0)
		//		{
		//			int t= LMotorPID.setpoint;
		//			t++;
		//			t=t%5;
		//			if(t==0)
		//				t=1;
		//			LMotorPID.setpoint=t;
		//			RMotorPID.setpoint=t;
		//		}
		osDelay(200);
	}
	/* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_Startsensor */
/**
 * @brief Function implementing the sensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Startsensor */
void Startsensor(void *argument)
{
	/* USER CODE BEGIN Startsensor */
	/* Infinite loop */
	HAL_GPIO_WritePin(GPIOE, TRIG_Pin,GPIO_PIN_RESET);
	for(;;)
	{
		HAL_GPIO_WritePin(GPIOE, TRIG_Pin,GPIO_PIN_SET);
		delay_us(10);
		HAL_GPIO_WritePin(GPIOE, TRIG_Pin,GPIO_PIN_RESET);
		//g_distanceUS = (tc2 > tc1 ? (tc2 - tc1) : (65535 - tc1 + tc2)) * 0.034 / 2; //echo/1000000.0f *343.0f/2.0f *100.0f equivalent
		osDelay(1);
		IR_Left_Read();
		IR_Right_Read();
		osDelay(10);
	}
	/* USER CODE END Startsensor */
}

/* USER CODE BEGIN Header_StartEncoderTask */
/**
 * @brief Function implementing the encoderTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void *argument)
{
	/* USER CODE BEGIN StartEncoderTask */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	int16_t cnt1L,cnt2L,diffL;
	int16_t cnt1R,cnt2R,diffR;
	uint32_t tick;
	cnt1L = __HAL_TIM_GET_COUNTER(&htim2);
	cnt1R = __HAL_TIM_GET_COUNTER(&htim3);
	tick = HAL_GetTick(); // in milliseconds
	/* Infinite loop */
	for(;;)
	{
		if (HAL_GetTick() - tick > 10L) { // Checking per second
			cnt2L = __HAL_TIM_GET_COUNTER(&htim2);
			cnt2R = __HAL_TIM_GET_COUNTER(&htim3);

			//Left
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				diffL = (cnt2L < cnt1L) ? (cnt1L - cnt2L) : ((65535 - cnt2L) + cnt1L);
			} else {
				diffL = (cnt2L > cnt1L) ? (cnt2L - cnt1L) : ((65535 - cnt1L) + cnt2L);
			}

			//Right
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
				diffR = (cnt2R < cnt1R) ? (cnt1R - cnt2R) : ((65535 - cnt2R) + cnt1R);
			} else {
				diffR = (cnt2R > cnt1R) ? (cnt2R - cnt1R) : ((65535 - cnt1R) + cnt2R);
			}
			// RPM Calculation
			RPM_L = ((float) diffL / (ENCODER_PULSES_PER_REVOLUTION * 4));// * 60.0;
			RPM_R = ((float) diffR / (ENCODER_PULSES_PER_REVOLUTION * 4));

			float dt = (HAL_GetTick() - tick) * 0.001f;
			RPS_L=((float) diffL / (ENCODER_PULSES_PER_REVOLUTION * 4* dt));
			RPS_R=((float) diffR / (ENCODER_PULSES_PER_REVOLUTION * 4 *dt));

			// Calculate the distance traveled by each wheel (in meters)
			float distanceLeftThisSecond = RPM_L * PI * WHEEL_DIAMETER;
			float distanceRightThisSecond = RPM_R * PI * WHEEL_DIAMETER;

			// Average the distances to account for potential minor differences in wheel speed
			float distanceThisSecond = (distanceLeftThisSecond + distanceRightThisSecond) / 2.0;
			distanceTraveled += distanceThisSecond;
			// Accumulate the total distance traveled
			cnt1L = __HAL_TIM_GET_COUNTER(&htim2);
			cnt1R = __HAL_TIM_GET_COUNTER(&htim3);
			tick = HAL_GetTick();
		}
		osDelay(50);
	}
	/* USER CODE END StartEncoderTask */
}

/* USER CODE BEGIN Header_startrobotTask */
/**
 * @brief Function implementing the robotTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startrobotTask */
void startrobotTask(void *argument)
{
	/* USER CODE BEGIN startrobotTask */
	/* Infinite loop */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);//Start servo pwm timer
	degree=0;
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	//while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==1);
	int labTarget=87;
	//	if(mode ==0)
	//		labTarget=87;
	//	else
	//		labTarget=166;

	//{osDelay(200);osThreadYield();}
	//
	tempflag=0;
	set_servo_angle(Center);
	osDelay(200);
	start_time = HAL_GetTick();
	end_time = HAL_GetTick();  // Record end time
	delta_time_sec= (end_time - start_time) * 0.001f; // Time difference in ms
	for(;;)
	{

		//TEST TURN SEGMENT
		//	  {
		//
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==0)
		{
			//resetYaw();
			//yaw=0;
			tempflag=1;
			CurrentState=MoveToBlock;
			// distanceTraveled=0;

			//target=85;
			//set_servo_angle(Right);
			//osDelay(550);
		}
		if(flagReceived==1&&flagDone!=1)
		{
		if(aRxBuffer[0]=='G')//Start by rpi from android
			{
			CurrentState=MoveToBlock;
			}
		}

		if(CurrentState!=NULL)
			CurrentState->stateFunction();
		//////
		if(tempflag==1)
		{
			//B(10);
			//			if(mode==0)
			//			set_servo_angle(Left);
			//			else
			//				set_servo_angle(Right);
			//			osDelay(300);
			//set_servo_angle(Right);//THE SMALLER THE MORE RIGHT
//			F(30);
//			FLeft(41.5);
//			FRight(-90);
//			distanceTraveled=0;
//			Backward(0);
//			Set_Motor_Direction(1,1);
//			FLeft(43.5);
//			distanceTraveled=0;
//			Backward(0);
//			Motor_Stop();
//			Set_Motor_Direction(1,1);
			tempflag=0;
			//Task2();
			//						  if(count==0)
			//						  {
			//							  flagReceived=1;
			//							  aRxBuffer[0]='A';
			//							  aRxBuffer[1]='1';
			//							  aRxBuffer[2]='0';
			//							  aRxBuffer[3]='0';
			//							  tempflag=0;
			//						  }
			//						  else if (count==1)
			//						  {
			//			//				  Set_Motor_Direction(1,1);
			//			//				  ForwardRight(-83);
			//							  flagReceived=1;
			//							  aRxBuffer[0]='A';
			//							  aRxBuffer[1]='0';
			//							  aRxBuffer[2]='0';
			//							  aRxBuffer[3]='0';
			//						  }
			//						  else{
			//							  count=0;
			//						  }
		}




		//				Set_Motor_DirectionTest(0,0);
		//				BackLeftTest(-83);
		//				  if(distanceTraveled < 180)
		//				  {
		//					  Forward(0);
		//				  }
		//				  else
		//				  {
		//					  if(count==0)
		//					  {
		//						  Motor_Stop();
		//						  distanceTraveled =190;
		//						  count++;
		//					  }
		//					  else{
		//						  Set_Motor_DirectionTest(1,0);
		//						  ForwardRightTest(-83);
		//					  }
		//				  }
		//			  while(1)
		//			  {
		//			  while(1)
		//			  {
		//			  	if(count==0)
		//			  	{
		//					Set_Motor_Direction(1,1);
		//	//				BackRightTest(83);
		//					ForwardLeft(83);
		//			  	}
		//			  	else if(count== 1){
		//			  		Set_Motor_Direction(0,0);
		//			  		BackLeft(-83);
		//			  	}
		//			  	else if(count==2){
		//			  		Set_Motor_Direction(1,1);
		//			  		ForwardRight(-83);
		//			  	}
		//			  	else if(count==3){
		//			  		Set_Motor_Direction(0,0);
		//			  		BackRight(83);
		//			  	}
		//			  	else{
		//			  		count=0;
		//			  	}
		//			  }
		////			  	else if(count ==2 && distanceTraveled<Target_Distance){
		//			  		if(distanceTraveled>=Target_Distance)
		//			  		{
		//			  			count++;
		//			  		}
		//			  		Set_Motor_DirectionTest(1,1);
		//			  		Forward(0);
		//			  	}
		//			  	else{
		//			  		Motor_Stop();
		//			  	}
		//			  }


		//Set_Motor_DirectionTest(0,1);
		//BackRightTest(83);
		//BackLeftTest(-83);
		// ForwardLeftTest(83);
		//			  			  Set_Motor_DirectionTest(1,0);
		//			  			  ForwardRightTest(-83);
		//ForwardRight(-84);

		//	  }
		//END OF TEST SEGMENT
		//ICM20948_readGyroscope_all(&hi2c1, 0, GYRO_SENS, &accel);
//		end_time = HAL_GetTick();
//		delta_time_sec= (end_time - start_time) * 0.001f;
//		start_time = HAL_GetTick();
//		if(flagReceived==1&&flagDone!=1){
//			char temp[4] = {0};  // Temporary buffer to hold up to 4 characters + null terminator
//			strncpy(temp, aRxBuffer+1, 3);
//			Target_Distance=atoi(temp);
//			if(aRxBuffer[0]=='W' && distanceTraveled < Target_Distance){
//				Set_Motor_Direction(1,1);
//				Forward(0);
//			}
//			else if(aRxBuffer[0]=='D'&&aRxBuffer[1]=='1'){
//				Set_Motor_Direction(1,1);
//				ForwardRight(-labTarget);
//			}
//			else if(aRxBuffer[0]=='A' &&aRxBuffer[1]=='1'){
//				Set_Motor_Direction(1,1);
//				ForwardLeft(labTarget);
//			}
//			else if(aRxBuffer[0]=='D'&&aRxBuffer[1]=='0'){
//				Set_Motor_Direction(0,0);
//				BackRight(labTarget);
//			}
//			else if(aRxBuffer[0]=='A'&&aRxBuffer[1]=='0'){
//				Set_Motor_Direction(0,0);
//				BackLeft(-labTarget);
//			}
//			else if(aRxBuffer[0]=='S' && distanceTraveled < Target_Distance)
//			{
//				Set_Motor_Direction(0,0);
//				Backward(0);
//			}
//			else if (aRxBuffer[0]=='R'){
//				if( g_distanceUS< Target_Distance &&g_distanceUS<30)
//					Backward(0);
//				//			    else if(g_distanceUS>Target_Distance &&g_distanceUS<30 )
//				//			       forward(0);
//			}
//			else{
//				flagDone=1;
//				flagReceived=0;
//			}
//		}
//		else if(flagDone==1)
//		{
//			Motor_Stop();
//			HAL_UART_Transmit(&huart3,(uint8_t *)"ACK\r\n",5,0xFFFF);
//			flagReceived=0;
//			flagDone = 0;
//		}

		osDelay(10);
	}
	/* USER CODE END startrobotTask */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
