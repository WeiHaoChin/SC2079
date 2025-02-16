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
#include "magcal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SERVO 230
#define MIN_SERVO 70

#define MAX_SERVO_Angle 180
#define MIN_SERVO_Angle 0
static float filtered_gyro = 0,last_raw=0;
float alpha = 0.95;
#define WHEEL_DIAMETER 6.5    // 6.5 cm (in meters)
int Target_Distance;  // 80 cm in meters (0.8 meters)
#define PI 3.14159265359
float distanceTraveled = 0.0;  // Total distance traveled in meters
float delta_time_sec=0;
float gyro_offset = 0.0;
float current_gyro_reading = 0.0;
int error_angle=0,calibrate,flagDone=0,flagReceived=0,count=0;
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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ultraSonicTask */
osThreadId_t ultraSonicTaskHandle;
const osThreadAttr_t ultraSonicTask_attributes = {
  .name = "ultraSonicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servoTask */
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
  .name = "servoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for IRTask */
osThreadId_t IRTaskHandle;
const osThreadAttr_t IRTask_attributes = {
  .name = "IRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
//Ultrasonic
int tc1=0,tc2=0,echo=0;
float g_distanceUS=0;//ultrasonic distance
double RPM_L=0,RPM_R=0;//Encoder

vec3 accel={};
vec3 gyro={};
vec3 mag={};
MagCalParams mag_params={-1.88,-18.15,-40.05,
						0.34,-1.02,0,
						1.02,0.34,0,
						0,0,0.23};

extern float magOffset[];
double q0,q1,q2,q3;
float yaw,pitch,roll;

int servo_pwm=150;
int pwmValL,pwmValR;
float degree=0;
int angle;

#define Slight_Right 90
#define Slight_Left 100
#define Right 0
#define Center 95
#define Left 140
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
void StartUltraSonic(void *argument);
void StartServoTask(void *argument);
void StartMotorTask(void *argument);
void StartEncoderTask(void *argument);
void Start_IRTask(void *argument);

/* USER CODE BEGIN PFP */
void delay_us(uint16_t us);
void servo_check(uint8_t* pwm);
void functionCheck();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[3]={0};

//SET both wheel direction
void Set_Motor_Direction(int foward_flag) {
    HAL_GPIO_WritePin(GPIOA, AIN1_Pin,
            ((foward_flag) ? GPIO_PIN_SET : GPIO_PIN_RESET));
    HAL_GPIO_WritePin(GPIOA, AIN2_Pin,
            ((foward_flag) ? GPIO_PIN_RESET : GPIO_PIN_SET));
    HAL_GPIO_WritePin(GPIOA, BIN2_Pin,
            ((foward_flag) ? GPIO_PIN_RESET : GPIO_PIN_SET));
    HAL_GPIO_WritePin(GPIOA, BIN1_Pin,
            ((foward_flag) ? GPIO_PIN_SET : GPIO_PIN_RESET));
}
void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim6,0);
	while(__HAL_TIM_GET_COUNTER(&htim6)<us);

}
void reset_gyro_at_rest() {
    gyro_offset =accel.z;  // Store the current reading as offset
}

float get_corrected_gyro() {
    return accel.z - gyro_offset;  // Subtract the offset from new readings
}
float high_pass_filter(float raw_gyro, float alpha)
{
    // Apply the filter: filtered value = alpha * current value + (1 - alpha) * previous filtered value
//	if(raw_gyro<0.15&&raw_gyro>0)
//	{
//		//last_raw=raw_gyro;
//	}
//	else{
		filtered_gyro = alpha *(filtered_gyro +raw_gyro-last_raw);
	    last_raw=raw_gyro;
//	}


    return filtered_gyro;
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


// ir sensor
uint16_t iDistanceL = 0;
uint16_t iDistanceR = 0;

//PID
float Kp = 0.5;  // Proportional gain
float Ki = 0.3;  // Integral gain
float Kd = 0.1;  // Derivative gain

float previous_error_L = 0.0f;
float previous_error_R = 0.0f;
float integral_L = 0.0f;
float integral_R = 0.0f;


void PID_Control(float target_RPM_R ,float target_RPM_L) {
    float error_L = target_RPM_L - RPM_L;  // Left motor speed error
    float error_R = target_RPM_R - RPM_R;  // Right motor speed error

    // Proportional term (current error)
    float P_L = Kp * error_L;
    float P_R = Kp * error_R;

    // Integral term (sum of past errors)
    integral_L += error_L;
    integral_R += error_R;
    float I_L = Ki * integral_L;
    float I_R = Ki * integral_R;

    // Derivative term (change in error)
    float D_L = Kd * (error_L - previous_error_L);
    float D_R = Kd * (error_R - previous_error_R);

    // Compute the PID output
    float PID_output_L = P_L + I_L + D_L;
    float PID_output_R = P_R + I_R + D_R;

    // Adjust PWM based on PID output (clamping to a valid range)
    pwmValL = (int16_t)(pwmValL + PID_output_L);
    pwmValR = (int16_t)(pwmValR + PID_output_R);
    if (pwmValL > 4000) pwmValL = 4000;
    if (pwmValL < 2000) pwmValL = 2000;

    if (pwmValR > 4000) pwmValR = 4000;
    if (pwmValR < 2000) pwmValR = 2000;

    // Set the new PWM values
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValL);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValR);

    // Update previous errors for the next cycle
    previous_error_L = error_L;
    previous_error_R = error_R;
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
	Set_Motor_Direction(1);
	HAL_Delay(1000);
	 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValL);
	 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValR);
	 HAL_Delay(1000);
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
	 HAL_Delay(2000);
	 Set_Motor_Direction(0);
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

  HAL_Delay(200);
  ICM20948_CalibrateGyro(&hi2c1,GYRO_SENS, 250);

  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);//Timer for ultra sonic
 HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,4);//Receive data from uart

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

  /* creation of ultraSonicTask */
  ultraSonicTaskHandle = osThreadNew(StartUltraSonic, NULL, &ultraSonicTask_attributes);

  /* creation of servoTask */
  servoTaskHandle = osThreadNew(StartServoTask, NULL, &servoTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);

  /* creation of encoderTask */
  encoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &encoderTask_attributes);

  /* creation of IRTask */
  IRTaskHandle = osThreadNew(Start_IRTask, NULL, &IRTask_attributes);

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Interrupt Serial callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	//prevent unused argument
	UNUSED(huart);
	HAL_UART_Transmit(&huart3,(uint8_t *)aRxBuffer,4,0xFFFF);
	flagReceived=1;
	// servo_pwm  = atoi(aRxBuffer);
	//memset(aRxBuffer,0,sizeof(aRxBuffer));
	 HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,4);
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
}

}
void IR_Left_Read() {
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 10);
	iDistanceL = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	iDistanceL=(int)(iDistanceL*-4.286e-03)+2.189e+01;
}

void IR_Right_Read() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	iDistanceR = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	iDistanceR=(int)(iDistanceR*-4.286e-03)+2.189e+01;
}

void Move_Left(){
	Set_Motor_Direction(1);
	set_servo_angle(Left);
	osDelay(1000);
	PID_Control(1,10);

}
void Move_Right(){
	Set_Motor_Direction(1);
	set_servo_angle(Right);
	osDelay(1000);
	PID_Control(10,1);
}

void Motor_Stop()
{
	degree=0;
	distanceTraveled=0;
    pwmValL = 0;
    pwmValR = 0;
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValL); // Stop left motor
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValR); // Stop right motor
	set_servo_angle(Center);
	osDelay(4000);
	reset_gyro_at_rest();
    count++;
}

void Move_Straight(){
    // Set PWM duty cycle for both motors
	Set_Motor_Direction(1);
	PID_Control(5,5);
}
void Move_Backwards(){
	Set_Motor_Direction(0);
    // Set PWM duty cycle for both motors
	PID_Control(5,5);

}
void Calibrate()
 {

 	set_servo_angle(Right);
 	osDelay(1000);
 	set_servo_angle(Left);
 	osDelay(1000);
 	set_servo_angle(Center);
 	osDelay(1000);
 	calibrate=1;
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
	  	/*char text[15];
		ICM20948_readGyroscope_all(&hi2c1, 0, GYRO_SENS,&accel);
		snprintf(text, sizeof(text), "X:%5.2f", accel.x);
		OLED_ShowString(10, 10, text);
		snprintf(text, sizeof(text), "Y:%5.2f", accel.y);
		OLED_ShowString(10, 20, text);
		snprintf(text, sizeof(text), "Z:%5.2f", accel.z);
		OLED_ShowString(10, 30, text);
		OLED_Refresh_Gram();
		HAL_GPIO_TogglePin(GPIOE,LED3_Pin);
		osDelay(100);*/
/*if (angle > 250 || angle < 70 )
	angle =150;

	  htim1.Instance->CCR4 = angle;
*/
	  /*
  HAL_GPIO_WritePin(GPIOE, TRIG_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, TRIG_Pin,GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(GPIOE, TRIG_Pin,GPIO_PIN_RESET);

  float dist = echo/10000.0f *343.0f/2.0f; //echo/1000000.0f *343.0f/2.0f *100.0f equivalent*/
	 // uint8_t ch = 'A';
	//	  HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);
		  //uint8_t test[15] = "Test String\0";
		  //sprintf(test,"%s\0",aRxBuffer);
//		  snprintf(test,15,"dist: %3.2f cm\0",dist);
//		  OLED_ShowString(10,10,test);
//
//		  OLED_Refresh_Gram();
    osDelay(150);

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

  /* Infinite loop */
  for(;;)
  {

//
		snprintf(text, sizeof(text), "EA:%d", error_angle);
		OLED_ShowString(10, 40, text);
//		  snprintf(text, sizeof(text), "X:%d", flagReceived);
		snprintf(text, sizeof(text), "Distance:%d", g_distanceUS);
		  OLED_ShowString(10, 20, text);
		//		  snprintf(text, sizeof(text), "X:%d", flagReceived);
		//		  OLED_ShowString(10, 50, text);
//		  snprintf(text, sizeof(text), "PWM:%c", aRxBuffer[0]);
//		  OLED_ShowString(10, 20, text);
		  snprintf(text, sizeof(text), "degree :%5.2f", degree);
		  OLED_ShowString(10, 30, text);
		  OLED_Refresh_Gram();
		  //snprintf(text, sizeof(text), "A:%d", angle);
		  snprintf(text, sizeof(text), "Distance: %.2f m", distanceTraveled);
		  OLED_ShowString(10, 10, text);
//	  ICM20948_readAccelerometer_all(&hi2c1,0,ACCEL_SENS,&accel);
	  /*snprintf(text, sizeof(text), "X:%5.2f", 2.5);
	  OLED_ShowString(10, 10, text);*/

	  //ICM20948_readMagnetometer_all(&hi2c1,&accel);
//	  ShowMagDataOled(&accel);
//	  magcal_adjust(&accel,&mag_params);

//	 			 snprintf(text, sizeof(text), "H:%5.2f",fmod((atan2(-accel.y, accel.x) * (180.0f / M_PI) + 360), 360));
//	 			 OLED_ShowString(10, 50, text);

//	  snprintf(text, sizeof(text), "Dist:%5.1fcm", g_distanceUS);
//	  OLED_ShowString(10, 10, text);
//
//	  snprintf(text, sizeof(text), "RPM:%5.2fRPM", RPM_R);
//	  	  OLED_ShowString(10, 20, text);

/*
	  snprintf(text, sizeof(text), "Yaw:%5.2f", yaw);
	  OLED_ShowString(10, 20, text);
	  snprintf(text, sizeof(text), "Pitch:%5.2f", pitch);
	  	  OLED_ShowString(10, 30, text);
	  	snprintf(text, sizeof(text), "Roll:%5.2f", roll);
	  		  OLED_ShowString(10, 40, text);*/
	 //OLED_Refresh_Gram();
	  //HAL_UART_Transmit(&huart2,text,sizeof(text),0xFFFF);
	  //HAL_UART_Transmit(&huart2,"\n\r",2,0xFFFF);
    osDelay(200);
  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartUltraSonic */
/**
* @brief Function implementing the ultraSonicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUltraSonic */
void StartUltraSonic(void *argument)
{
  /* USER CODE BEGIN StartUltraSonic */
	HAL_GPIO_WritePin(GPIOE, TRIG_Pin,GPIO_PIN_RESET);
  /* Infinite loop */
  for(;;)
  {
	    HAL_GPIO_WritePin(GPIOE, TRIG_Pin,GPIO_PIN_SET);
	    delay_us(10);
	    HAL_GPIO_WritePin(GPIOE, TRIG_Pin,GPIO_PIN_RESET);
	    //g_distanceUS = (tc2 > tc1 ? (tc2 - tc1) : (65535 - tc1 + tc2)) * 0.034 / 2; //echo/1000000.0f *343.0f/2.0f *100.0f equivalent
    osDelay(10);
  }
  /* USER CODE END StartUltraSonic */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
  /* USER CODE BEGIN StartServoTask */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);//Start servo pwm timer
	degree=0;
	osDelay(200);
	//Calibrate();
	start_time = HAL_GetTick();
	end_time = HAL_GetTick();  // Record end time
	delta_time_sec= (end_time - start_time) / 1000.0f; // Time difference in ms
  /* Infinite loop */
  for(;;)
  {
//	  if(aRxBuffer[0]=='W'||aRxBuffer[0]=='S')
//	  {
		ICM20948_readGyroscope_all(&hi2c1, 0, GYRO_SENS, &accel);
		end_time = HAL_GetTick();
		delta_time_sec= (end_time - start_time) / 1000.0f;
		 float filtered_gyro_value = high_pass_filter(accel.z, alpha);
		  degree+=filtered_gyro_value * delta_time_sec;
		  error_angle=Center-(int)degree;
		  if(count==1)
		  {
			  if(error_angle>(Center+.2)) //(error_angle>(Center+.5))
			  {
				  //set_servo_pwm(146);//turn slght left 146
				  set_servo_angle(Slight_Left);
			  }
			  else if(error_angle<(Center-.2))
			  {
				  //set_servo_pwm(160); //turn slight right 160
				  set_servo_angle(Slight_Right);
			  }
			  else{
				  //set_servo_pwm(152);
				  set_servo_angle(Center);
			  }
		  }
		  else if(count==2)
		  {
			  if(error_angle<(Center-.2)) //(error_angle>(Center+.5))
			  {
				  //set_servo_pwm(146);//turn slght left 146
				  set_servo_angle(Slight_Left);
			  }
			  else if(error_angle>(Center+.2))
			  {
				  //set_servo_pwm(160); //turn slight right 160
				  set_servo_angle(Slight_Right);
			  }
			  else{
				  //set_servo_pwm(152);
				  set_servo_angle(Center);
			  }
		  }
		  start_time = HAL_GetTick();
		    osDelay(100);
	  }
//	  else{
//
//	  }


  }
//}

  /* USER CODE END StartServoTask */

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */


  /* USER CODE BEGIN StartMotorTask */
void StartMotorTask(void *argument)
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==1);
	//while(calibrate==0);
	set_servo_angle(Center);
	Target_Distance=200;
  /* Infinite loop */
  for(;;)
  {

//	  if(count==0)
//	  {
//		  Move_Left(27);
//	  }
//	  if(count==0)
//	  {
//		  Move_Right(40);
//	  } //70 is for left //140 for right
	  if(count==4)
	  {//for reverse left / right swap the function call and its fine and -5
		  if(error_angle<135) // Move left 135 for 90 degree //180 for 180 degree 175 works too
			  // do increments of 45 for each 90 degree
		  {
			  Move_Right();
		  }
		  		else{
		  			Motor_Stop();
		  		}
	  }
	  if(count==0)
	  {//for reverse left / right swap the function call and its fine
		  if(error_angle>55) // Move left 135 for 90 degree //180 for 180 degree 175 works too
			  // do increments of 45 for each 90 degree
		  {
			  Move_Left();
		  }
		  		else{
		  			Motor_Stop();
		  		}
	  }

//	  if(count==2)
//	  {
//		  if(error_angle>55) // Move left 55 for 90 degree //correct one forward
//		  {
//			  Move_Left();
//		  }
//		  		else{
//		  			Motor_Stop();
//		  		}
//	  }

	  if(distanceTraveled<Target_Distance)// Do not touch
	  {
		  if(count==1)
		  {
			  Move_Straight();
		  }
		  if(count==5)
		  {
			  Move_Backwards();
		  }

		}
//	  if(flagReceived==1)
//		{
//		  Target_Distance=atoi(aRxBuffer+1);
//			if(aRxBuffer[0]=='W')
//			{
//				if(distanceTraveled < Target_Distance)
//				{
//				  Move_Straight();
//				}
//				else{
//				Motor_Stop();
//				flagDone=1;
//				flagReceived=0;
//				}
//			}
//			else if(aRxBuffer[0]=='D')
//			{
//				if(distanceTraveled < Target_Distance)
//				{
//					Move_Right();
//				}
//				else{
//					Motor_Stop();
//					flagDone=1;
//					flagReceived=0;
//				}
//			}
//			else if(aRxBuffer[0]=='A')
//			{
//				if(distanceTraveled < Target_Distance)
//				{
//					Move_Left();
//				}
//				else{
//					Motor_Stop();
//					flagDone=1;
//					flagReceived=0;
//				}
//			}
//			else if(aRxBuffer[0]=='S')
//			{
//				if(distanceTraveled < Target_Distance)
//				{
//					Move_Backwards();
//				}
//				else{
//					Motor_Stop();
//					flagDone=1;
//					flagReceived=0;
//				}
//			}
//			else{
//				flagReceived=0;
//			}
////	}
//	  	if(g_distanceUS<20)
//	  	{
//	  		Motor_Stop();
//	  	}
		else{
			Motor_Stop();
		}
//	else if(flagDone==1)
//	{
//		flagReceived=0;
//		flagDone = 0;
//	}


	    osDelay(50); // Delay to control update frequency
  	  }
	    /*
	  Set_Motor_Direction(0);
	  while(pwmValL>0){
	 		  pwmValL--;
	 		  pwmValR--;

	 	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValL);
	 	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValR);
	 	  osDelay(10);
	 	  }*/
	  //__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
  /* USER CODE END StartMotorTask */
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
	  osDelay(150);
  }
  /* USER CODE END StartEncoderTask */
}

/* USER CODE BEGIN Header_Start_IRTask */
/**
* @brief Function implementing the IRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_IRTask */
void Start_IRTask(void *argument)
{
  /* USER CODE BEGIN Start_IRTask */
  /* Infinite loop */
  for(;;)
  {
		IR_Left_Read();
		IR_Right_Read();
		osDelay(150);
  }
  /* USER CODE END Start_IRTask */
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
