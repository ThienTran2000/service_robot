/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "stdio.h"

#define ki_s 0
#define kp_s 5
#define kd_s 0
#define t 0.02
#define pi 3.14159265358979323846
#define DELAY 200
#define REVERSE 130


typedef struct MOTOR
{		
	volatile long long pos;
	volatile short encoder;
	int pwm;
	float vel;
	float err;
	float err_p;	
	float P;
	float I;
	float D;
	float I_p;
	float kp;
	float ki;
} MOTOR;
	
typedef struct HC_SR04
{
	uint8_t Is_Fist_Captured;
	uint32_t ICValue_1;
	uint32_t ICValue_2;
	uint32_t Dis;
} HC_SR04;

unsigned char STATE; // 0 if result of Check_HC_SR04 < 50 and 1 contrary
int DELAY_COUNT = 0; // count up if robot stop at state
int REVERSE_COUNT = 0;
int a[] = {0 , 0, 0, 0, 0};
int L = 0; // length of arr;
float PID = 0; // result of PIDSensor 
int LINE = 0; // count up if robot reach line
int RUN = 1;
int COUNT_UART_IT = 0;
///////////////////////////////////////////////////////////HC_SR04
HC_SR04 HC_SR04_1;
HC_SR04 HC_SR04_2;
HC_SR04 HC_SR04_3;
///////////////////////////////////////////////////////////uart
uint8_t receive_data[5];
uint8_t prepare_data[2];
uint8_t is_fist_transmit = 0;
uint8_t send_data;
///////////////////////////////////////////////////////////sensor
int SENSOR_p = 31;
int SENSOR_p1 = 0;
int SENSOR;
int SENSOR1;
int SENSOR2;
int SENSOR3;
int SENSOR4;
int SENSOR5;
float err = 0;
//////////////////////////////////////////////////////////////////////////////motor
MOTOR motorA = {0,0,0,0,0,0,0,0,0,0,62.8,648.5};
MOTOR motorB = {0,0,0,0,0,0,0,0,0,0,73.49,683.75};
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void PIDVel(MOTOR *motor, float SP) // SP is setpoint (Rad/s)
{
	motor->err = SP - motor->vel;	
	motor->P = motor->kp*motor->err;
	motor->I = motor->I_p + motor->ki*motor->err*t;
	if (motor->I >1000)
		motor->I = 1000;
	else if (motor->I < -1000)
		motor->I = -1000;
	//motor->D = kd*(motor->err - motor->err_p)/t;
	
	motor->err_p = motor->err;
	motor->I_p = motor->I;
	
	motor->pwm = (int)(motor->P + motor->I );
	
	if (motor->pwm >1000)
		motor->pwm = 1000;
	else if (motor->pwm < -1000)
		motor->pwm = -1000;
}

float PIDSensor()
{
	static float P = 0, I=0, D=0, err_p = 0;
	static float PID_Value = 0;
	P = err;
	I = I + err;
	if (PID_Value >1)
			PID_Value = 1;
	else if (PID_Value < -1)
			PID_Value = -1;
	D = err - err_p;
	PID_Value = kp_s*P + ki_s*I + kd_s*D;
	err_p = err;
	if (PID_Value >1)
			PID_Value = 1;
	else if (PID_Value < -1)
			PID_Value = -1;
	return PID_Value;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void ReadSensor()
{
	SENSOR1 = HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin);
	SENSOR2 = HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin);
	SENSOR3 = HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin);
	SENSOR4 = HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin);
	SENSOR5 = HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin);
	SENSOR = SENSOR1*pow(2,4)+SENSOR2*pow(2,3)+SENSOR3*pow(2,2)+SENSOR4*pow(2,1)+SENSOR5*pow(2,0);
	switch(SENSOR)
	{
		case 15: 
			err = -0.4;
			break;
		case 7: 
			err = -0.3;
			break;
		case 23: 
			err = -0.2;
			break;
		case 19: 
			err = -0.1;
			break;
		case 27: 
			err = 0;
			break;
		case 25: 
			err = 0.1;
			break;
		case 29: 
			err = 0.2;
			break;
		case 28: 
			err = 0.3;
			break;
		case 30: 
			err = 0.4;
			break;
		case 0:
			if (SENSOR_p1 != 0)
			{
				LINE ++;
				RUN = 0;
			} 
		default:
			if((SENSOR_p == 15)||(SENSOR_p == 7)||(SENSOR_p == 23)||(SENSOR_p == 19))
				err = -0.5;
			else if ((SENSOR_p == 27)||(SENSOR_p == 25)||(SENSOR_p == 29)||(SENSOR_p == 28))
				err = +0.5;
			break;
	}
	if ((SENSOR != 31)&&(SENSOR != 0))
		SENSOR_p = SENSOR;
	SENSOR_p1 = SENSOR;
} 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
///////////////////////////////////////////////////////////
void Rotate_A(int pwm)
{
	if (pwm<0)
	{
		HAL_GPIO_WritePin(MOTORA_1_GPIO_Port, MOTORA_1_Pin, 0);
		HAL_GPIO_WritePin(MOTORA_2_GPIO_Port, MOTORA_2_Pin, 1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,-pwm);
	}
	else
	{
		HAL_GPIO_WritePin(MOTORA_1_GPIO_Port, MOTORA_1_Pin, 1);
		HAL_GPIO_WritePin(MOTORA_2_GPIO_Port, MOTORA_2_Pin, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,pwm);
	}
}
///////////////////////////////////////////////////////////
void Rotate_B(int pwm)
{
	if (pwm<0)
	{
		HAL_GPIO_WritePin(MOTORB_1_GPIO_Port, MOTORB_1_Pin, 0);
		HAL_GPIO_WritePin(MOTORB_2_GPIO_Port, MOTORB_2_Pin, 1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,-pwm);
	}
	else
	{
		HAL_GPIO_WritePin(MOTORB_1_GPIO_Port, MOTORB_1_Pin, 1);
 		HAL_GPIO_WritePin(MOTORB_2_GPIO_Port, MOTORB_2_Pin, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,pwm);
	}
}
///////////////////////////////////////////////////////////
void ResetVariable()
{
	DELAY_COUNT = 0;
	REVERSE_COUNT = 0;
	L = 0;
	PID = 0;
	LINE = 0;
	RUN = 1;
	COUNT_UART_IT = 0;
}
///////////////////////////////////////////////////////////
void GetLength() // check length of array
{
	L = 5;
	int i = 4;
	while (a[i] <= 0 && i>=0)
	{
		L--;
		i--;
	}
}
///////////////////////////////////////////////////////////
void ReceiveData()
{
	for (int k = 0; k < 5; k++)
	{
		a[k] = receive_data[k] - 48;
	}
	ResetVariable();
	GetLength();
}
///////////////////////////////////////////////////////////
void TransmitData()
{
	if (is_fist_transmit == 0)
	{
		prepare_data[0] = LINE;
		prepare_data[1] = 1;
		send_data = prepare_data[0]*10 + prepare_data[1];
		HAL_UART_Transmit_IT(&huart2, &send_data, 1);
		is_fist_transmit = 1;
	}
}
///////////////////////////////////////////////////////////
void StopRobot() // Stop robot
{
	Rotate_A(0);
	Rotate_B(0);
}
///////////////////////////////////////////////////////////
void StopMotor()  // stop motor when robot reach line
{
	DELAY_COUNT++;
	if (DELAY_COUNT < DELAY)
	{
		StopRobot();
		TransmitData();
	}
	else
	{
		DELAY_COUNT = 0;
		RUN = 1;
		is_fist_transmit = 0;
	}
}
///////////////////////////////////////////////////////////
void RunMotor() // run motor when robot on way
{
	PIDVel(&motorA, 3 + PID);
	PIDVel(&motorB, 3 - PID);
	Rotate_A(motorA.pwm );
	Rotate_B(motorB.pwm );	
}
///////////////////////////////////////////////////////////
void ReverseRobot() // reverse motor when robot reach end line or begin line
{
	REVERSE_COUNT ++;
	if (REVERSE_COUNT < REVERSE)
	{
		PIDVel(&motorA, 4);
		PIDVel(&motorB, -4);
		Rotate_A(motorA.pwm );
		Rotate_B(motorB.pwm );
	}
	else
	{
		REVERSE_COUNT = 0;
		if (LINE == L || LINE == 2*L + 1)
			LINE++;
	}
}
///////////////////////////////////////////////////////////
void OperateRobot()
{
	if (LINE == 0)
		RunMotor();
	else if (LINE > 0 && LINE <= L)
	{
		if (a[LINE - 1] != 0)
		{
			if (RUN == 0)
				StopMotor();
			else 
			{
				if (LINE == L)
					ReverseRobot();
				else
					RunMotor();
			}
		}
		else
			RunMotor();
	}
	else
	{
		if (LINE == 2*L + 1)
			ReverseRobot();
		else if (LINE == 2*L + 2)
			StopRobot();
		else
			RunMotor();
	}
}
///////////////////////////////////////////////////////////
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///////////////////////////////////////////////////////////
void CheckHC_SR04()
{
	if ((HC_SR04_1.Dis > 50)||(HC_SR04_2.Dis > 50)||(HC_SR04_3.Dis > 50))
		STATE = 1;
	else 
		STATE = 1;
}
///////////////////////////////////////////////////////////
void delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<time);
}
///////////////////////////////////////////////////////////
void read_HC_SR04()
{
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
	
	HC_SR04_1.Is_Fist_Captured =0;
	HC_SR04_2.Is_Fist_Captured =0;
	HC_SR04_3.Is_Fist_Captured =0;

	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
	delay(10);
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	//__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}
///////////////////////////////////////////////////////////
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Channel==HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(HC_SR04_1.Is_Fist_Captured == 0)
		{
			HC_SR04_1.ICValue_1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
			HC_SR04_1.Is_Fist_Captured = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if(HC_SR04_1.Is_Fist_Captured == 1)
		{
			HC_SR04_1.ICValue_2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
			HC_SR04_1.Dis = (int)((HC_SR04_1.ICValue_2-HC_SR04_1.ICValue_1)/58);			
		}			
	}
	////////////////////////////////////////////////
	if(htim -> Channel==HAL_TIM_ACTIVE_CHANNEL_4)
	{
		if(HC_SR04_2.Is_Fist_Captured == 0)
		{
			HC_SR04_2.ICValue_1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
			HC_SR04_2.Is_Fist_Captured = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if(HC_SR04_2.Is_Fist_Captured == 1)
		{
			HC_SR04_2.ICValue_2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
			HC_SR04_2.Dis = (int)((HC_SR04_2.ICValue_2-HC_SR04_2.ICValue_1)/58);			
		}			
	}
	///////////////////////////////////////////////
	if(htim -> Channel==HAL_TIM_ACTIVE_CHANNEL_3)
	{
		if(HC_SR04_3.Is_Fist_Captured == 0)
		{
			HC_SR04_3.ICValue_1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
			HC_SR04_3.Is_Fist_Captured = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if(HC_SR04_3.Is_Fist_Captured == 1)
		{
			HC_SR04_3.ICValue_2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
			HC_SR04_3.Dis = (int)((HC_SR04_3.ICValue_2-HC_SR04_3.ICValue_1)/58);			
		}			
	}
}
///////////////////////////////////////////////////////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance==htim10.Instance)
	{			
		motorA.encoder = __HAL_TIM_GET_COUNTER(&htim3);
		motorB.encoder = __HAL_TIM_GET_COUNTER(&htim4);

		__HAL_TIM_SET_COUNTER(&htim3,0);
		__HAL_TIM_SET_COUNTER(&htim4,0);

		motorA.vel = ((motorA.encoder*50)/1996.8)*2*pi;
		motorB.vel = ((motorB.encoder*50)/1996.8)*2*pi;
			
		ReadSensor();	
		PID = PIDSensor();
		CheckHC_SR04();
		if (STATE == 1 && L != 0)
		{
			OperateRobot();
		}
		else
		{
			StopRobot();
		}
 	}
}
///////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		ReceiveData();
	}
}
///////////////////////////////////////////////////////////
int dem = 0;
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim10);
	//HAL_TIM_Base_Start(&htim11);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);

	GetLength();
	HAL_UART_Receive_DMA(&huart2, receive_data, 5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		read_HC_SR04();
		HAL_Delay(200);
		if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)==0)
			dem++;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 41;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 39999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTORA_1_Pin|MOTORA_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MOTORB_1_Pin|MOTORB_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTORA_1_Pin MOTORA_2_Pin */
  GPIO_InitStruct.Pin = MOTORA_1_Pin|MOTORA_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR1_Pin SENSOR2_Pin SENSOR3_Pin SENSOR4_Pin
                           SENSOR5_Pin */
  GPIO_InitStruct.Pin = SENSOR1_Pin|SENSOR2_Pin|SENSOR3_Pin|SENSOR4_Pin
                          |SENSOR5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTORB_1_Pin MOTORB_2_Pin */
  GPIO_InitStruct.Pin = MOTORB_1_Pin|MOTORB_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : trig_Pin */
  GPIO_InitStruct.Pin = trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(trig_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
