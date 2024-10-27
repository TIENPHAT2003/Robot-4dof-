/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include"math.h"
#include"Encoder.h"
#include"MotorDrive.h"
#include"stdlib.h"
#include "PID.h"
#include "stdio.h"
#include <stdbool.h>
#include "cJSON.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId TaskLogicHandle;
osThreadId TaskSetHomeHandle;
osThreadId TaskCalPIDHandle;
osThreadId TaskTrajectoryHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);
void StartTaskLogic(void const * argument);
void StartTaskSetHome(void const * argument);
void StartTaskPID(void const * argument);
void StartTaskTrajectory(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//----------------GLOBAL VARIABLE------------------//
typedef struct{
	uint8_t startSetHome;
	uint8_t startProgram;
	uint8_t starKinematics;
	uint8_t SetPoint_Nha;
	uint8_t SetPoint_Hut;
	uint8_t Fail;
}FlagStart_;
FlagStart_ FlagStart;

typedef struct{
	int16_t SpeedSetHomeJ1;
	int16_t SpeedSetHomeJ2;
	int16_t SpeedSetHomeJ3;
	int16_t SpeedSetHomeJ4;
}SpeedSetHomeJ_;
SpeedSetHomeJ_ SpeedSetHomeJ;

typedef struct{
	uint8_t sethomeJ1;
	uint8_t sethomeJ2;
	uint8_t sethomeJ3;
	uint8_t sethomeJ4;
}sethomeJ_;
sethomeJ_ sethomeJ;

typedef struct{
	int8_t sensor1;
	int8_t sensor2;
	int8_t sensor3;
	int8_t sensor4;
}sensor_;
sensor_ sensor;

typedef struct{
	float AngleLink1;
	float AngleLink2;
	float AngleLink3;
	float AngleLink4;
}Angle_;
Angle_ Angle;
//----------------GLOBAL VARIABLE------------------//

//--------------TRAJECTORY PLANNING----------------//
#define MAX_POINTS 50
typedef struct {
    float theta1;
    float theta2;
    float theta3;
    float theta4;
} Point;

typedef struct{
	float setpoint1;
	float setpoint2;
	float setpoint3;
	float setpoint4;

	float preSetpoint1;
	float preSetpoint2;
	float preSetpoint3;
	float preSetpoint4;

	float p0_1;
	float p0_2;
	float p0_3;
	float p0_4;

	float theta1_Nha;
	float theta2_Nha;
	float theta3_Nha;
	float theta4_Nha;

	float theta1_Hut;
	float theta2_Hut;
	float theta3_Hut;
	float theta4_Hut;
	Point points[MAX_POINTS];
}Setpoint_;
Setpoint_ Setpoint;

float T1, T2, T3, T4;
float Tf=3000;

float p(float p0, float pf, float tf, float v0, float vf, float T)
{
    return p0+v0*T+(3*(pf-p0)/(tf*tf)-2*v0/tf-vf/tf)*(T*T)+(-2*(pf-p0)/(tf*tf*tf)+(vf+v0)/(tf*tf))*(T*T*T);
}
//--------------TRAJECTORY PLANNING----------------//
#define MAX_MESG 2048
char uartLogBuffer[MAX_MESG];
uint8_t flag_uart_rx = 0;
uint16_t uartLogRxSize;

void UartIdle_Init()
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)uartLogBuffer, MAX_MESG);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void UART_Handle(char* data, Setpoint_* Setpoint)
{
    static char uartDataBuffer[1024] = "";
    static size_t dataBufferIndex = 0;

    if (flag_uart_rx == 1 && strstr(data, "\n"))
    {
        // Check for specific commands
        if (strstr(data, "theta1"))
        {
            if (sscanf(data, "theta1:%f,theta2:%f,theta3:%f,theta4:%f\n",
                       &Setpoint->setpoint1, &Setpoint->setpoint2,
                       &Setpoint->setpoint3, &Setpoint->setpoint4) == 4)
            {
                FlagStart.starKinematics = 1;
            }
        }
        else if (strstr(data, "NhaT1"))
        {
            if (sscanf(data, "NhaT1:%f,NhaT2:%f,NhaT3:%f,NhaT4:%f\n",
                       &Setpoint->theta1_Nha, &Setpoint->theta2_Nha,
                       &Setpoint->theta3_Nha, &Setpoint->theta4_Nha) == 4)
            {
                FlagStart.SetPoint_Nha = 1;
            }
        }
        else if (strstr(data, "Point"))
        {
        	if (dataBufferIndex + strlen(data) < sizeof(uartDataBuffer) - 1) {
				strncat(uartDataBuffer, data, sizeof(uartDataBuffer) - dataBufferIndex - 1);
				dataBufferIndex += strlen(data);

				if (strchr(uartDataBuffer, '\n') != NULL) {
					char* savePtr;
					char* token = strtok_r(uartDataBuffer, ";", &savePtr);

					while (token != NULL) {
						int pointId;
						float theta1, theta2, theta3, theta4;


						if (sscanf(token, "Point:%d, HutT1:%f, HutT2:%f, HutT3:%f, HutT4:%f",
								   &pointId, &theta1, &theta2, &theta3, &theta4) == 5) {
							Setpoint->points[pointId].theta1 = theta1;
							Setpoint->points[pointId].theta2 = theta2;
							Setpoint->points[pointId].theta3 = theta3;
							Setpoint->points[pointId].theta4 = theta4;
							FlagStart.SetPoint_Hut = 1;
						}
						else{
							FlagStart.Fail = 1;
						}
						token = strtok_r(NULL, ";", &savePtr);
					}

					memset(uartDataBuffer, 0, sizeof(uartDataBuffer));
					dataBufferIndex = 0;
				}
			}
        }
        else if (strstr(data, "home"))
        {
            // Handle "home" command here
        }
        else if (strstr(data, "Reset"))
        {
            HAL_NVIC_SystemReset();
        }
        else if (strstr(data, "hut"))
        {
            // Handle "hut" command here
        }
        else if (strstr(data, "nha"))
        {
            // Handle "nha" command here
        }
        else if (strstr(data, "start"))
        {
            FlagStart.startProgram = 1;
        }
        else if (strstr(data, "disconnected"))
        {
            FlagStart.startProgram = 0;
            FlagStart.SetPoint_Hut = 0;
            FlagStart.SetPoint_Nha = 0;
            FlagStart.starKinematics = 0;
        }
        flag_uart_rx = 0;
        memset(data, 0, uartLogRxSize);
    }
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    uartLogRxSize = Size;
    flag_uart_rx = 1;
	UART_Handle(uartLogBuffer, &Setpoint);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)uartLogBuffer, MAX_MESG);
  }
}
//----------------LINK1------------------//
EncoderRead ENC_LINK1;
MotorDrive 	Motor_LINK1;
PID_Param	PID_DC_SPEED_LINK1;
PID_Param	PID_DC_POS_LINK1;
void PID_LINK1_Init()
{
	PID_DC_SPEED_LINK1.kP = 50;
	PID_DC_SPEED_LINK1.kI = 250;
	PID_DC_SPEED_LINK1.kD = 0;
	PID_DC_SPEED_LINK1.alpha = 0;
	PID_DC_SPEED_LINK1.deltaT = 0.01;
	PID_DC_SPEED_LINK1.uI_AboveLimit = 1000;
	PID_DC_SPEED_LINK1.uI_BelowLimit = -1000;
	PID_DC_SPEED_LINK1.u_AboveLimit  = 1000;
	PID_DC_SPEED_LINK1.u_BelowLimit  = -1000;

	PID_DC_POS_LINK1.kP = 10;
	PID_DC_POS_LINK1.kI = 0;
	PID_DC_POS_LINK1.kD = 0;
	PID_DC_POS_LINK1.alpha = 0;
	PID_DC_POS_LINK1.deltaT = 0.01;
	PID_DC_POS_LINK1.uI_AboveLimit = 1000;
	PID_DC_POS_LINK1.uI_BelowLimit = -1000;
	PID_DC_POS_LINK1.u_AboveLimit  = 1000;
	PID_DC_POS_LINK1.u_BelowLimit  = -1000;
}
void PID_LINK1_Speed(){
	SpeedReadNonReset(&ENC_LINK1);
	Pid_Cal(&PID_DC_SPEED_LINK1, PID_DC_POS_LINK1.u, ENC_LINK1.vel_Real);
	Drive(&Motor_LINK1, &htim8, PID_DC_SPEED_LINK1.u, TIM_CHANNEL_3, TIM_CHANNEL_4);
}
void PID_LINK1_Pos(){
	Pid_Cal(&PID_DC_POS_LINK1, Angle.AngleLink1, CountRead(&ENC_LINK1, count_ModeDegree));
	PID_LINK1_Speed();
}
//----------------LINK1------------------//

//----------------LINK 2------------------//
EncoderRead ENC_LINK2;
MotorDrive 	Motor_LINK2;
PID_Param	PID_DC_SPEED_LINK2;
PID_Param	PID_DC_POS_LINK2;
void PID_LINK2_Init()
{
	PID_DC_SPEED_LINK2.kP = 50;
	PID_DC_SPEED_LINK2.kI = 250;
	PID_DC_SPEED_LINK2.kD = 0;
	PID_DC_SPEED_LINK2.alpha = 0;
	PID_DC_SPEED_LINK2.deltaT = 0.01;
	PID_DC_SPEED_LINK2.uI_AboveLimit = 1000;
	PID_DC_SPEED_LINK2.uI_BelowLimit = -1000;
	PID_DC_SPEED_LINK2.u_AboveLimit  = 1000;
	PID_DC_SPEED_LINK2.u_BelowLimit  = -1000;

	PID_DC_POS_LINK2.kP = 10;
	PID_DC_POS_LINK2.kI = 0;
	PID_DC_POS_LINK2.kD = 0;
	PID_DC_POS_LINK2.alpha = 0;
	PID_DC_POS_LINK2.deltaT = 0.01;
	PID_DC_POS_LINK2.uI_AboveLimit = 1000;
	PID_DC_POS_LINK2.uI_BelowLimit = -1000;
	PID_DC_POS_LINK2.u_AboveLimit  = 1000;
	PID_DC_POS_LINK2.u_BelowLimit  = -1000;
}
void PID_LINK2_Speed(){
	SpeedReadNonReset(&ENC_LINK2);
	Pid_Cal(&PID_DC_SPEED_LINK2, PID_DC_POS_LINK2.u, ENC_LINK2.vel_Real);
	Drive(&Motor_LINK2, &htim4, PID_DC_SPEED_LINK2.u, TIM_CHANNEL_3, TIM_CHANNEL_4);
}
void PID_LINK2_Pos(){
	Pid_Cal(&PID_DC_POS_LINK2, Angle.AngleLink2 -187, CountRead(&ENC_LINK2, count_ModeDegree));
	PID_LINK2_Speed();
}
//----------------LINK 2------------------//

//----------------LINK 3------------------//
EncoderRead ENC_LINK3;
MotorDrive 	Motor_LINK3;
PID_Param	PID_DC_SPEED_LINK3;
PID_Param	PID_DC_POS_LINK3;
void PID_LINK3_Init()
{
	PID_DC_SPEED_LINK3.kP = 50;
	PID_DC_SPEED_LINK3.kI = 300;
	PID_DC_SPEED_LINK3.kD = 0;
	PID_DC_SPEED_LINK3.alpha = 0;
	PID_DC_SPEED_LINK3.deltaT = 0.01;
	PID_DC_SPEED_LINK3.uI_AboveLimit = 1000;
	PID_DC_SPEED_LINK3.uI_BelowLimit = -1000;
	PID_DC_SPEED_LINK3.u_AboveLimit  = 1000;
	PID_DC_SPEED_LINK3.u_BelowLimit  = -1000;

	PID_DC_POS_LINK3.kP = 10;
	PID_DC_POS_LINK3.kI = 0;
	PID_DC_POS_LINK3.kD = 0;
	PID_DC_POS_LINK3.alpha = 0;
	PID_DC_POS_LINK3.deltaT = 0.01;
	PID_DC_POS_LINK3.uI_AboveLimit = 1000;
	PID_DC_POS_LINK3.uI_BelowLimit = -1000;
	PID_DC_POS_LINK3.u_AboveLimit  = 1000;
	PID_DC_POS_LINK3.u_BelowLimit  = -1000;
}
void PID_LINK3_Speed(){
	SpeedReadNonReset(&ENC_LINK3);
	Pid_Cal(&PID_DC_SPEED_LINK3, PID_DC_POS_LINK3.u, ENC_LINK3.vel_Real);
	Drive(&Motor_LINK3, &htim4, PID_DC_SPEED_LINK3.u, TIM_CHANNEL_1, TIM_CHANNEL_2);
}
void PID_LINK3_Pos(){
	Pid_Cal(&PID_DC_POS_LINK3, Angle.AngleLink3 + 135, CountRead(&ENC_LINK3, count_ModeDegree));
	PID_LINK3_Speed();
}
//----------------LINK 3------------------//

//----------------LINK 4------------------//
EncoderRead ENC_LINK4;
MotorDrive 	Motor_LINK4;
PID_Param	PID_DC_SPEED_LINK4;
PID_Param	PID_DC_POS_LINK4;
void PID_LINK4_Init()
{
	PID_DC_SPEED_LINK4.kP = 50;
	PID_DC_SPEED_LINK4.kI = 250;
	PID_DC_SPEED_LINK4.kD = 0;
	PID_DC_SPEED_LINK4.alpha = 0;
	PID_DC_SPEED_LINK4.deltaT = 0.01;
	PID_DC_SPEED_LINK4.uI_AboveLimit = 1000;
	PID_DC_SPEED_LINK4.uI_BelowLimit = -1000;
	PID_DC_SPEED_LINK4.u_AboveLimit  = 1000;
	PID_DC_SPEED_LINK4.u_BelowLimit  = -1000;

	PID_DC_POS_LINK4.kP = 10;
	PID_DC_POS_LINK4.kI = 0;
	PID_DC_POS_LINK4.kD = 0;
	PID_DC_POS_LINK4.alpha = 0;
	PID_DC_POS_LINK4.deltaT = 0.01;
	PID_DC_POS_LINK4.uI_AboveLimit = 1000;
	PID_DC_POS_LINK4.uI_BelowLimit = -1000;
	PID_DC_POS_LINK4.u_AboveLimit  = 1000;
	PID_DC_POS_LINK4.u_BelowLimit  = -1000;
}
void PID_LINK4_Speed(){
	SpeedReadNonReset(&ENC_LINK4);
	Pid_Cal(&PID_DC_SPEED_LINK4, PID_DC_POS_LINK4.u, ENC_LINK4.vel_Real);
	Drive(&Motor_LINK4, &htim9, PID_DC_SPEED_LINK4.u, TIM_CHANNEL_1, TIM_CHANNEL_2);
}
void PID_LINK4_Pos(){
	Pid_Cal(&PID_DC_POS_LINK4, Angle.AngleLink4 - 90, CountRead(&ENC_LINK4, count_ModeDegree));
	PID_LINK4_Speed();
}
//----------------LINK 4------------------//



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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  EncoderSetting(&ENC_LINK1, &htim1, 6950, 0.01);
  EncoderSetting(&ENC_LINK2, &htim2, 3250, 0.01);
  EncoderSetting(&ENC_LINK3, &htim3, 7050, 0.01);
  EncoderSetting(&ENC_LINK4, &htim5, 3220, 0.01);

  PID_LINK1_Init();
  PID_LINK2_Init();
  PID_LINK3_Init();
  PID_LINK4_Init();

  UartIdle_Init();

  /* USER CODE END 2 */

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
  /* definition and creation of TaskLogic */
  osThreadDef(TaskLogic, StartTaskLogic, osPriorityNormal, 0, 128);
  TaskLogicHandle = osThreadCreate(osThread(TaskLogic), NULL);

  /* definition and creation of TaskSetHome */
  osThreadDef(TaskSetHome, StartTaskSetHome, osPriorityNormal, 0, 128);
  TaskSetHomeHandle = osThreadCreate(osThread(TaskSetHome), NULL);

  /* definition and creation of TaskCalPID */
  osThreadDef(TaskCalPID, StartTaskPID, osPriorityNormal, 0, 128);
  TaskCalPIDHandle = osThreadCreate(osThread(TaskCalPID), NULL);

  /* definition and creation of TaskTrajectory */
  osThreadDef(TaskTrajectory, StartTaskTrajectory, osPriorityBelowNormal, 0, 128);
  TaskTrajectoryHandle = osThreadCreate(osThread(TaskTrajectory), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 6;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 6;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  huart1.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : Sensor_J3_Pin Sensor_J4_Pin */
  GPIO_InitStruct.Pin = Sensor_J3_Pin|Sensor_J4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Sensor_J1_Pin Sensor_J2_Pin */
  GPIO_InitStruct.Pin = Sensor_J1_Pin|Sensor_J2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskLogic */
/**
  * @brief  Function implementing the TaskLogic thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskLogic */
void StartTaskLogic(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskSetHome */
/**
* @brief Function implementing the TaskSetHome thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSetHome */
void StartTaskSetHome(void const * argument)
{
  /* USER CODE BEGIN StartTaskSetHome */
  /* Infinite loop */
  for(;;)
  {
	sensor.sensor1 = HAL_GPIO_ReadPin(Sensor_J1_GPIO_Port, Sensor_J1_Pin);
	sensor.sensor2 = HAL_GPIO_ReadPin(Sensor_J2_GPIO_Port, Sensor_J2_Pin);
	sensor.sensor3 = HAL_GPIO_ReadPin(Sensor_J3_GPIO_Port, Sensor_J3_Pin);
	sensor.sensor4 = HAL_GPIO_ReadPin(Sensor_J4_GPIO_Port, Sensor_J4_Pin);
	if(FlagStart.startSetHome == 1){
		sethomeJ.sethomeJ1 = 0;
		sethomeJ.sethomeJ2 = 0;
		sethomeJ.sethomeJ3 = 0;
		sethomeJ.sethomeJ4 = 0;
		FlagStart.startProgram = 0;
		FlagStart.startSetHome = 0;
	}
	if(FlagStart.startProgram == 0){
		if(sethomeJ.sethomeJ1 == 0){
			if(HAL_GPIO_ReadPin(Sensor_J1_GPIO_Port, Sensor_J1_Pin) == 1){
				osDelay(1);
				if(HAL_GPIO_ReadPin(Sensor_J1_GPIO_Port, Sensor_J1_Pin) == 1){
					ResetCount(&ENC_LINK1, 1);
					SpeedSetHomeJ.SpeedSetHomeJ1 = 0;
					sethomeJ.sethomeJ1 = 1;
					Angle.AngleLink1 = 0;
					Setpoint.p0_1 = 0;
				}
			}
			else {
				SpeedSetHomeJ.SpeedSetHomeJ1 = -400;
				if(CountRead(&ENC_LINK1, count_ModeDegree) > 90 && SpeedSetHomeJ.SpeedSetHomeJ1 > 0){
					SpeedSetHomeJ.SpeedSetHomeJ1 *= -1;
				}
				else if(CountRead(&ENC_LINK1, count_ModeDegree) < -90 && SpeedSetHomeJ.SpeedSetHomeJ1 < 0) {
					SpeedSetHomeJ.SpeedSetHomeJ1 *= -1;
				}
				Drive(&Motor_LINK1, &htim8, SpeedSetHomeJ.SpeedSetHomeJ1, TIM_CHANNEL_3, TIM_CHANNEL_4);
			}
		}
		if(sethomeJ.sethomeJ2 == 0){
			if(HAL_GPIO_ReadPin(Sensor_J2_GPIO_Port, Sensor_J2_Pin) == 1){
				osDelay(1);
				if(HAL_GPIO_ReadPin(Sensor_J2_GPIO_Port, Sensor_J2_Pin) == 1){
					ResetCount(&ENC_LINK2, 1);
					SpeedSetHomeJ.SpeedSetHomeJ2 = 0;
					sethomeJ.sethomeJ2 = 1;
					Angle.AngleLink2 = 187;
					Setpoint.p0_2 = 187;

				}
			}
			else {
				SpeedSetHomeJ.SpeedSetHomeJ2 = 400;
				Drive(&Motor_LINK2, &htim4, SpeedSetHomeJ.SpeedSetHomeJ2, TIM_CHANNEL_3, TIM_CHANNEL_4);
			}
		}
		if(sethomeJ.sethomeJ3 == 0){
			if(HAL_GPIO_ReadPin(Sensor_J3_GPIO_Port, Sensor_J3_Pin) == 0){
				osDelay(1);
				if(HAL_GPIO_ReadPin(Sensor_J3_GPIO_Port, Sensor_J3_Pin) == 0){
					ResetCount(&ENC_LINK3, 1);
					sethomeJ.sethomeJ3 = 1;
					SpeedSetHomeJ.SpeedSetHomeJ3 = 0;
					Angle.AngleLink3 = -135;
					Setpoint.p0_3 = -135;
				}
			}
			else {
				SpeedSetHomeJ.SpeedSetHomeJ3 = -300;
				Drive(&Motor_LINK3, &htim4, SpeedSetHomeJ.SpeedSetHomeJ3, TIM_CHANNEL_1, TIM_CHANNEL_2);
			}
		}
		if(sethomeJ.sethomeJ4 == 0){
			if(HAL_GPIO_ReadPin(Sensor_J4_GPIO_Port, Sensor_J4_Pin) == 0){
				osDelay(1);
				if(HAL_GPIO_ReadPin(Sensor_J4_GPIO_Port, Sensor_J4_Pin) == 0){
					ResetCount(&ENC_LINK4, 1);
					SpeedSetHomeJ.SpeedSetHomeJ4 = 0;
					sethomeJ.sethomeJ4 = 1;
					Angle.AngleLink4 = 90;
					Setpoint.p0_4 = 90;
				}
			}
			else {
				SpeedSetHomeJ.SpeedSetHomeJ4 = 300;
				Drive(&Motor_LINK4, &htim9, SpeedSetHomeJ.SpeedSetHomeJ4, TIM_CHANNEL_1, TIM_CHANNEL_2);
			}
		}
//		if(sethomeJ.sethomeJ1 == 1 && sethomeJ.sethomeJ2 == 1 && sethomeJ.sethomeJ3 == 1 && sethomeJ.sethomeJ4 == 1){
//			FlagStart.startProgram = 1;
//		}
	}
    osDelay(10);
  }
  /* USER CODE END StartTaskSetHome */
}

/* USER CODE BEGIN Header_StartTaskPID */
/**
* @brief Function implementing the TaskCalPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPID */
void StartTaskPID(void const * argument)
{
  /* USER CODE BEGIN StartTaskPID */
  /* Infinite loop */
  for(;;)
  {
	  if(sethomeJ.sethomeJ1 == 1)	PID_LINK1_Pos();
	  if(sethomeJ.sethomeJ2 == 1)	PID_LINK2_Pos();
	  if(sethomeJ.sethomeJ3 == 1)	PID_LINK3_Pos();
	  if(sethomeJ.sethomeJ4 == 1)	PID_LINK4_Pos();

	  osDelay(10);
  }
  /* USER CODE END StartTaskPID */
}

/* USER CODE BEGIN Header_StartTaskTrajectory */
/**
* @brief Function implementing the TaskTrajectory thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTrajectory */
void StartTaskTrajectory(void const * argument)
{
  /* USER CODE BEGIN StartTaskTrajectory */
  /* Infinite loop */
  static uint8_t mode = 0;
  for(;;)
  {
	if(FlagStart.startProgram == 1){
		switch(mode){
			case 0:
				if(T1 < Tf){
					T1 += 5;
					Angle.AngleLink1 = p(Setpoint.p0_1, Setpoint.setpoint1, Tf, 0, 0, T1);
				}
				mode = 1;
				break;
			case 1:
				if(T2 < Tf){
					T2 += 5;
					Angle.AngleLink2 = p(Setpoint.p0_2, Setpoint.setpoint2, Tf, 0, 0, T2);
				}
				mode = 2;
				break;

			case 2:
				if(T3 < Tf){
					T3 += 5;
					Angle.AngleLink3 = p(Setpoint.p0_3, Setpoint.setpoint3, Tf, 0, 0, T3);
				}
				mode = 3;
				break;
			case 3:
				if(T4 < Tf){
					T4 += 5;
					Angle.AngleLink4 = p(Setpoint.p0_4, Setpoint.setpoint4, Tf, 0, 0, T4);
				}
				mode = 4;
				break;
			case 4:
				  if (Setpoint.setpoint1 != Setpoint.preSetpoint1)
				  {
					T1 = 0;
					Setpoint.p0_1 = Angle.AngleLink1;
					Setpoint.preSetpoint1 = Setpoint.setpoint1;
				  }
				  if (Setpoint.setpoint2 != Setpoint.preSetpoint2)
				  {
					T2 = 0;
					Setpoint.p0_2 = Angle.AngleLink2;
					Setpoint.preSetpoint2 = Setpoint.setpoint2;
				  }
				  if (Setpoint.setpoint3 != Setpoint.preSetpoint3)
				  {
					T3 = 0;
					Setpoint.p0_3 = Angle.AngleLink3;
					Setpoint.preSetpoint3 = Setpoint.setpoint3;
				  }
				  if (Setpoint.setpoint4 != Setpoint.preSetpoint4)
				  {
					T4 = 0;
					Setpoint.p0_4 = Angle.AngleLink4;
					Setpoint.preSetpoint4 = Setpoint.setpoint4;
				  }
				  mode = 0;

				  break;
			default:
			  break;
		}
	}
    osDelay(1);
  }
  /* USER CODE END StartTaskTrajectory */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
