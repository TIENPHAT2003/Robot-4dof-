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

osThreadId defaultTaskHandle;
osThreadId TaskSetHomeHandle;
osThreadId TaskCalPIDHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskSetHome(void const * argument);
void StartTaskPID(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//----------------GLOBAL VARIABLE------------------//
uint8_t startProgram, startFK, startIK_BN1, startIK_BN2;
int SpeedSetHomeJ1, SpeedSetHomeJ2, SpeedSetHomeJ3, SpeedSetHomeJ4;
uint8_t sethomeJ1, sethomeJ2, sethomeJ3, sethomeJ4;
float inputTheta1, inputTheta2, inputTheta3, inputTheta4;
float inputPx, inputPy, inputPz, inputTheta;
int8_t sensor1, sensor2, sensor3, sensor4;

//----------------GLOBAL VARIABLE------------------//

//----------------LINK1------------------//
EncoderRead ENC_LINK1;
MotorDrive 	Motor_LINK1;
PID_Param	PID_DC_SPEED_LINK1;
PID_Param	PID_DC_POS_LINK1;
float AngleLink1;
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
	Pid_Cal(&PID_DC_POS_LINK1, AngleLink1, CountRead(&ENC_LINK1, count_ModeDegree));
	PID_LINK1_Speed();
}
//----------------LINK1------------------//

//----------------LINK 2------------------//
EncoderRead ENC_LINK2;
MotorDrive 	Motor_LINK2;
PID_Param	PID_DC_SPEED_LINK2;
PID_Param	PID_DC_POS_LINK2;
float AngleLink2;
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
	Pid_Cal(&PID_DC_POS_LINK2, AngleLink2 -187, CountRead(&ENC_LINK2, count_ModeDegree));
	PID_LINK2_Speed();
}
//----------------LINK 2------------------//

//----------------LINK 3------------------//
EncoderRead ENC_LINK3;
MotorDrive 	Motor_LINK3;
PID_Param	PID_DC_SPEED_LINK3;
PID_Param	PID_DC_POS_LINK3;
float AngleLink3;
void PID_LINK3_Init()
{
	PID_DC_SPEED_LINK3.kP = 50;
	PID_DC_SPEED_LINK3.kI = 250;
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
	Pid_Cal(&PID_DC_POS_LINK3, AngleLink3 + 135, CountRead(&ENC_LINK3, count_ModeDegree));
	PID_LINK3_Speed();
}
//----------------LINK 3------------------//

//----------------LINK 4------------------//
EncoderRead ENC_LINK4;
MotorDrive 	Motor_LINK4;
PID_Param	PID_DC_SPEED_LINK4;
PID_Param	PID_DC_POS_LINK4;
float AngleLink4;
void PID_LINK4_Init()
{
	PID_DC_SPEED_LINK4.kP = 50;
	PID_DC_SPEED_LINK4.kI = 300;
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
	Pid_Cal(&PID_DC_POS_LINK4, AngleLink4 - 90, CountRead(&ENC_LINK4, count_ModeDegree));
	PID_LINK4_Speed();
}
//----------------LINK 4------------------//

//--------------Cal Forward Kinematics-----------//
float theta1_FK = 0, theta2_FK = 0, theta3_FK = 0, theta4_FK = 0, t = 0, theta_FK = 0, psi_FK = 0;

float pre_theta1_FK = 0, pre_theta2_FK = 0, pre_theta3_FK = 0, pre_theta4_FK = 0, pre_theta_FK = 0;

float px_qd = 0, py_qd = 0, pz_qd = 0, theta_qd = 0, time = 0;

float theta1_FK_rad = 0, theta2_FK_rad = 0, theta3_FK_rad = 0, theta4_FK_rad = 0, theta_FK_rad, psi_FK_rad = 0;

float Px_FK = 0, Py_FK = 0, Pz_FK = 0;

float L1 = 91, L2 = 122, L3 = 77, L4 = 78, d1 = 62 + 176; // đơn vị mm

int16_t t1, t2, t3, t4;

float Px, Py, Pz;
void TINH_FK(float theta1Value, float theta2Value, float theta3Value, float theta4Value) {

	theta1_FK = theta1Value;
	theta1_FK_rad = (theta1_FK * M_PI) / 180.0;

	theta2_FK = theta2Value;
	theta2_FK_rad = (theta2_FK * M_PI) / 180.0;

	theta3_FK = theta3Value;
	theta3_FK_rad = (theta3_FK * M_PI) / 180.0;

	theta4_FK = theta4Value;
	theta4_FK_rad = (theta4_FK * M_PI) / 180.0;

	psi_FK = theta2_FK + theta3_FK + theta4_FK;
	psi_FK_rad = theta2_FK_rad + theta3_FK_rad + theta4_FK_rad;

	// Tính toán giá trị Px, Py, Pz
	Px_FK = cos(theta1_FK_rad) * (L1 + L2 * cos(theta2_FK_rad) + L3 * cos(theta2_FK_rad + theta3_FK_rad) + L4 * cos(psi_FK_rad));

	Py_FK = sin(theta1_FK_rad) * (L1 + L2 * cos(theta2_FK_rad) + L3 * cos(theta2_FK_rad + theta3_FK_rad) + L4 * cos(psi_FK_rad));

	Pz_FK = d1 + L3 * sin(theta2_FK_rad + theta3_FK_rad) + L2 * sin(theta2_FK_rad) + L4 * sin(psi_FK_rad);

}
//--------------Cal Forward Kinematics-----------//

//--------------Cal Inverse Kinematics-----------//
float Px_IK = 0, Py_IK = 0, Pz_IK = 0, Theta_IK = 0;

float theta1_IK_rad = 0, theta2_IK_rad = 0, theta3_IK_rad = 0, theta4_IK_rad = 0;

float Theta1_IK = 0, Theta2_IK = 0, Theta3_IK = 0, Theta4_IK = 0;

float alpha = 0, k = 0, E = 0, F = 0, a = 0, b = 0, d = 0, f = 0, var_temp = 0, c23 = 0, s23 = 0, t_rad = 0;

void calculate_IK_BN1(float Px_value, float Py_value, float Pz_value, float Theta_value){

    Px_IK = Px_value;
    Py_IK = Py_value;
    Pz_IK = Pz_value;
    Theta_IK = Theta_value;

    t_rad = Theta_IK * (M_PI / 180);
    k = sqrt(pow(Px_IK, 2) + pow(Py_IK, 2));
    theta1_IK_rad = atan2((Py_IK / k), (Px_IK / k));
    Theta1_IK = theta1_IK_rad * (180 / M_PI);

    if (Theta1_IK < -180) {
        Theta1_IK += 360;
    } else if (Theta1_IK > 180) {
        Theta1_IK -= 360;
    }

    E = Px_IK * cos(theta1_IK_rad) + Py_IK * sin(theta1_IK_rad) - L1 - L4 * cos(t_rad);
    F = Pz_IK - d1 - L4 * sin(t_rad);
    a = -2 * L2 * F;
    b = -2 * L2 * E;
    d = pow(L3, 2) - pow(E, 2) - pow(F, 2) - pow(L2, 2);
    f = sqrt(pow(a, 2) + pow(b, 2));
    alpha = atan2(-2 * L2 * F / f, -2 * L2 * E / f);

    var_temp = pow(d, 2) / pow(f, 2);
    if (var_temp > 1) var_temp = 1;

    theta2_IK_rad = atan2(sqrt(1 - var_temp), d / f) + alpha;
    Theta2_IK = theta2_IK_rad * (180 / M_PI);

    if (Theta2_IK < -180) {
        Theta2_IK += 360;
    } else if (Theta2_IK > 180) {
        Theta2_IK -= 360;
    }

    c23 = (Px_IK * cos(theta1_IK_rad) + Py_IK * sin(theta1_IK_rad) - L1 - L2 * cos(theta2_IK_rad) - L4 * cos(t_rad)) / L3;
    s23 = (Pz_IK - d1 - L2 * sin(theta2_IK_rad) - L4 * sin(t_rad)) / L3;
    theta3_IK_rad = atan2(s23, c23) - theta2_IK_rad;
    Theta3_IK = theta3_IK_rad * (180 / M_PI);

    if (Theta3_IK < -180) {
        Theta3_IK += 360;
    } else if (Theta3_IK > 180) {
        Theta3_IK -= 360;
    }

    theta4_IK_rad = t_rad - theta2_IK_rad - theta3_IK_rad;
    Theta4_IK = theta4_IK_rad * (180 / M_PI);

}

void calculate_IK_BN2(float px_value, float py_value, float pz_value, float Theta_value) {

    Px_IK = px_value;
    Py_IK = py_value;
    Pz_IK = pz_value;
    Theta_IK = Theta_value;

    t_rad = Theta_IK * (M_PI / 180);
    k = sqrt(pow(Px_IK, 2) + pow(Py_IK, 2));
    theta1_IK_rad = atan2((Py_IK / k), (Px_IK / k));
    Theta1_IK = theta1_IK_rad * (180 / M_PI);

    if (Theta1_IK < -180) {
        Theta1_IK += 360;
    } else if (Theta1_IK > 180) {
        Theta1_IK -= 360;
    }

    E = Px_IK * cos(theta1_IK_rad) + Py_IK * sin(theta1_IK_rad) - L1 - L4 * cos(t_rad);
    F = Pz_IK - d1 - L4 * sin(t_rad);

    a = -2 * L2 * F;
    b = -2 * L2 * E;
    d = pow(L3, 2) - pow(E, 2) - pow(F, 2) - pow(L2, 2);
    f = sqrt(pow(a, 2) + pow(b, 2));
    alpha = atan2(-2 * L2 * F / f, -2 * L2 * E / f);

    var_temp = pow(d, 2) / pow(f, 2);
    if (var_temp > 1) var_temp = 1;

    theta2_IK_rad = atan2(-sqrt(1 - var_temp), d / f) + alpha;
    Theta2_IK = theta2_IK_rad * (180 / M_PI);

    if (Theta2_IK < -180) {
        Theta2_IK += 360;
    } else if (Theta2_IK > 180) {
        Theta2_IK -= 360;
    }

    c23 = (Px_IK * cos(theta1_IK_rad) + Py_IK * sin(theta1_IK_rad) - L1 - L2 * cos(theta2_IK_rad) - L4 * cos(t_rad)) / L3;
    s23 = (Pz_IK - d1 - L2 * sin(theta2_IK_rad) - L4 * sin(t_rad)) / L3;
    theta3_IK_rad = atan2(s23, c23) - theta2_IK_rad;
    Theta3_IK = theta3_IK_rad * (180 / M_PI);

    if (Theta3_IK < -180) {
        Theta3_IK += 360;
    } else if (Theta3_IK > 180) {
        Theta3_IK -= 360;
    }

    theta4_IK_rad = t_rad - theta2_IK_rad - theta3_IK_rad;
    Theta4_IK = theta4_IK_rad * (180 / M_PI);
}
//--------------Cal Inverse Kinematics-----------//

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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
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
  EncoderSetting(&ENC_LINK3, &htim3, 6880, 0.01);
  EncoderSetting(&ENC_LINK4, &htim5, 3220, 0.01);

  PID_LINK1_Init();
  PID_LINK2_Init();
  PID_LINK3_Init();
  PID_LINK4_Init();
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TaskSetHome */
  osThreadDef(TaskSetHome, StartTaskSetHome, osPriorityNormal, 0, 128);
  TaskSetHomeHandle = osThreadCreate(osThread(TaskSetHome), NULL);

  /* definition and creation of TaskCalPID */
  osThreadDef(TaskCalPID, StartTaskPID, osPriorityNormal, 0, 128);
  TaskCalPIDHandle = osThreadCreate(osThread(TaskCalPID), NULL);

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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	if(startProgram == 1){
		if(startFK == 1){
			TINH_FK(inputTheta1, inputTheta2, inputTheta3, inputTheta4);
		}
		if(startIK_BN1 == 1){
			calculate_IK_BN1(inputPx, inputPy, inputPz, inputTheta);
		}
		if(startIK_BN2 == 1){
			calculate_IK_BN2(inputPx, inputPy, inputPz, inputTheta);
		}
	}
//	TINH_FK(inputTheta1, inputTheta2, inputTheta3, inputTheta4);
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
  startProgram = 0;
  for(;;)
  {
	sensor1 = HAL_GPIO_ReadPin(Sensor_J1_GPIO_Port, Sensor_J1_Pin);
	sensor2 = HAL_GPIO_ReadPin(Sensor_J2_GPIO_Port, Sensor_J2_Pin);
	sensor3 = HAL_GPIO_ReadPin(Sensor_J3_GPIO_Port, Sensor_J3_Pin);
	sensor4 = HAL_GPIO_ReadPin(Sensor_J4_GPIO_Port, Sensor_J4_Pin);

	if(startProgram == 0){
		if(sethomeJ1 == 0){
			if(HAL_GPIO_ReadPin(Sensor_J1_GPIO_Port, Sensor_J1_Pin) == 1){
				osDelay(1);
				if(HAL_GPIO_ReadPin(Sensor_J1_GPIO_Port, Sensor_J1_Pin) == 1){
					ResetCount(&ENC_LINK1, 1);
					SpeedSetHomeJ1 = 0;
					sethomeJ1 = 1;
					AngleLink1 = 0;
				}
			}
			else {
				SpeedSetHomeJ1 = -400;
				if(CountRead(&ENC_LINK1, count_ModeDegree) > 90 && SpeedSetHomeJ1 > 0){
					SpeedSetHomeJ1 *= -1;
				}
				else if(CountRead(&ENC_LINK1, count_ModeDegree) < -90 && SpeedSetHomeJ1 < 0) {
					SpeedSetHomeJ1 *= -1;
				}
				Drive(&Motor_LINK1, &htim8, SpeedSetHomeJ1, TIM_CHANNEL_3, TIM_CHANNEL_4);
			}
		}
		if(sethomeJ2 == 0){
			if(HAL_GPIO_ReadPin(Sensor_J2_GPIO_Port, Sensor_J2_Pin) == 1){
				osDelay(1);
				if(HAL_GPIO_ReadPin(Sensor_J2_GPIO_Port, Sensor_J2_Pin) == 1){
					ResetCount(&ENC_LINK2, 1);
					SpeedSetHomeJ2 = 0;
					sethomeJ2 = 1;
					AngleLink2 = 187;
				}
			}
			else {
				SpeedSetHomeJ2 = 400;
				Drive(&Motor_LINK2, &htim4, SpeedSetHomeJ2, TIM_CHANNEL_3, TIM_CHANNEL_4);
			}
		}
		if(sethomeJ3 == 0){
			if(HAL_GPIO_ReadPin(Sensor_J3_GPIO_Port, Sensor_J3_Pin) == 0){
				osDelay(1);
				if(HAL_GPIO_ReadPin(Sensor_J3_GPIO_Port, Sensor_J3_Pin) == 0){
					ResetCount(&ENC_LINK3, 1);
					sethomeJ3 = 1;
					AngleLink3 = -135;
				}
			}
			else {
				SpeedSetHomeJ3 = -300;
				Drive(&Motor_LINK3, &htim4, SpeedSetHomeJ3, TIM_CHANNEL_1, TIM_CHANNEL_2);
			}
		}
		if(sethomeJ4 == 0){
			if(HAL_GPIO_ReadPin(Sensor_J4_GPIO_Port, Sensor_J4_Pin) == 0){
				osDelay(1);
				if(HAL_GPIO_ReadPin(Sensor_J4_GPIO_Port, Sensor_J4_Pin) == 0){
					ResetCount(&ENC_LINK4, 1);
					SpeedSetHomeJ4 = 0;
					sethomeJ4 = 1;
					AngleLink4 = 90;
				}
			}
			else {
				SpeedSetHomeJ4 = 300;
				Drive(&Motor_LINK4, &htim9, SpeedSetHomeJ4, TIM_CHANNEL_1, TIM_CHANNEL_2);
			}
		}
		if(sethomeJ1 == 1 && sethomeJ2 == 1 && sethomeJ3 == 1 && sethomeJ4 == 1){
			startProgram = 1;
		}
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
	  if(sethomeJ1 == 1)	PID_LINK1_Pos();
	  if(sethomeJ2 == 1)	PID_LINK2_Pos();
	  if(sethomeJ3 == 1)	PID_LINK3_Pos();
	  if(sethomeJ4 == 1)	PID_LINK4_Pos();

	  if(startFK == 1){
		  AngleLink1 = inputTheta1;
		  AngleLink2 = inputTheta2;
		  AngleLink3 = inputTheta3;
		  AngleLink4 = inputTheta4;
	  }
	  if(startIK_BN1 == 1 || startIK_BN2 == 1){
		  AngleLink1 = Theta1_IK;
		  AngleLink2 = Theta2_IK;
		  AngleLink3 = Theta3_IK;
		  AngleLink4 = Theta4_IK;
	  }
	  osDelay(10);
  }
  /* USER CODE END StartTaskPID */
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
