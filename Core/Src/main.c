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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "ModBusRTU.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

//-----------------Encoder Feedback-------------------//
uint32_t QEIReadRaw;

typedef struct
{
	// for record New / Old value to calculate dx / dt
	uint32_t Position[2];
	uint64_t TimeStamp[2];
	float QEIPostion_1turn;
	float QEIAngularVelocity;
	float rad_s;
	float rpm;
	float linearPos;
	float linearVel;
	float Velocity[2];
	float linearAcc;
}QEI_StructureTypeDef;

QEI_StructureTypeDef QEIdata = {0};
//---------------------------------------------------//

//Timer
uint64_t _micros = 0;

enum
{
	NEW,OLD
};

//Read joystick manual mode
uint16_t ADCBuffer[2];
float x_position;
float set_manual_point = 0;
uint8_t check_up = 0;
uint8_t check_down = 0;

//mode control
uint8_t mode = 3;
uint8_t mode_savestate = 3;
uint64_t start_IT = 0;

//PWM
float Vin = 0;
float Vin_old = 0;
float Vin_force = 0;
float duty_cycle = 50;

//--------------Trajectory-------------------//
//Define
float q_d_i = 0;
float q_d_max = 570; //Maximum velocity
float q_2d_max = 600; //Maximum acc
//init
float q_f = 0;
float q_i = 0;
float delta_q = 0;
//output trajectory
float ref_p = 0;
float ref_v = 0;
float ref_a = 0;
//other parameters
uint8_t calmode = 3; //stop mode
float q_f_old = 0;
float t_acc = 0;
float q_acc = 0;
float t_const = 0;
float q_const = 0;
float q_d_acc = 0;
uint64_t T_start = 0;
float t = 0;
int dir = 1;
float total_t = 0;
//---------------------------------------------//

//------------------Manual Button--------------//
//Button
uint8_t B_up = 0;
uint8_t B_down = 0;
uint8_t B_reset = 0;
uint8_t B_save = 0;
uint8_t emergency_reset = 0;

uint8_t count_save = 0;
float shelves_pos[6] = {0};
float last_shelf_save = 0;
//-------------------------------------------//

//PID
arm_pid_instance_f32 PID1 = {0}; //Control Position
arm_pid_instance_f32 PID2 = {0}; //Control Velocity
float setPosition = 0;
float setVelocity = 0;
float max_velo = 0;

//Sensor
int S_top = 0;
int S_down = 0;
int S_top_savestate = 0;
int S_down_savestate = 0;

int check_state_B= 0;
//------------------MODBUS-----------------------//
ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[200];

//maunualadd
int check  = 0;
int check2  = 0;
int vaccum_status = 0;
int gripper_status = 0;
int check_state  = 0;

// set shelf
int Enter_click = 0;
int set_shelf_state = 0;
//set point
float set_point_modbus = 0;
int point_state_triger = 0;
int set_point_state = 0;
// set home
int set_Home_state = 0;
int Home_state_triger = 0;
//run jog
int PickOrder = 0;
int PlaceOrder = 0;
int set_jog_state = 0;
int PickArray[5] = {0};
int PlaceArray[5] = {0};

int Moving_status = 0;
int Jog_state_triger = 0;
int Jog_order = 0;
int Jog_oneloop_trigger = 0;
//---------------------------------------------//
// Noise filter
uint64_t timestamp_savestate = 0;
int64_t prev_vac_savestate = 0;
int64_t prev_grp_savestate = 0;
uint8_t trigger_savestate = 0;
int check_noise = 0;
int64_t currentTime = 0;

//uint8_t test_Gripper = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros();
void QEIEncoderPosVel_Update();
void checkStartMoving();
void createTrajectory();
void DriveMotor();
void JoystickInput();
void button_up_down_input();
void button_reset_input();
void SoftwareLimit();

void check_vaccum_status();
void check_gripper_status();
void set_shelf();
void Pointmode();
void Home();
void Run_jog();
void Heartbeat();
void Routine();
void button_save_position();

void LED_Emergency();
void LED_Homing();
void LED_Ready();
void LED_Auto();
void LED_Manual();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //PWM Motor
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);

  //Read Encoder
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

  //time microsec
  HAL_TIM_Base_Start_IT(&htim5);

  //Read joystick
  HAL_ADC_Start_DMA(&hadc1, ADCBuffer, 2);
  HAL_TIM_Base_Start(&htim4);

  //PID Control Position
  PID1.Kp = 0; // 7.5
  PID1.Ki = 0; // 0.0025
  PID1.Kd = 0; // 3
  arm_pid_init_f32(&PID1, 0);

  //PID Control Velocity
  PID2.Kp = 0.003; //0.5
  PID2.Ki = 0.019; // 0.006
  PID2.Kd = 0.07; // 0.05
  arm_pid_init_f32(&PID2, 0);

  //Modbus Setting
  // manaul add
  HAL_TIM_Base_Start_IT(&htim6);

  //  modbus
  hmodbus.huart = &huart2;
  hmodbus.htim = &htim16;
  hmodbus.slaveAddress = 0x15;
  hmodbus.RegisterSize =200;
  Modbus_init(&hmodbus, registerFrame);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //------Modbus Function------//
	  Modbus_Protocal_Worker();
	  check_vaccum_status();
	  check_gripper_status();
	  set_shelf();
	  Pointmode();
	  Home();
//	  Run_jog();

	  static uint64_t timestamp = 0;
	  static uint64_t timestamp2 = 0;

	  currentTime = micros();
	  if (max_velo < QEIdata.linearVel)
	  {
		  max_velo = QEIdata.linearVel;
	  }

	  if(currentTime > timestamp2){
		  timestamp2 = currentTime + 167; //6,000 Hz

		  //Read encoder
		  QEIEncoderPosVel_Update();

		  //Create Trajectory
		  q_f = setPosition;
		  checkStartMoving();
		  createTrajectory();

		  if(mode == 1){
			  LED_Auto();
			  if(fabs(setPosition - QEIdata.linearPos) <= 5 || setPosition == 0){
				  Vin = 0;
//				  if (setPosition != 0)
//				  {
//					  mode = 3;
//				  }

			  }
//			  else if(setPosition - QEIdata.linearPos <= 5 && setPosition - QEIdata.linearPos > 0.1){
//				  Vin = 2.5;
//				  check = 1;
//			  }
//			  else if(setPosition - QEIdata.linearPos >= -5 && setPosition - QEIdata.linearPos < -0.1){
//			  	  Vin = -1.35;
//			  	  check = -1;
//			  }
			  else{
				  //Vin = arm_pid_f32(&PID2, (setVelocity + ref_v)/2 - QEIdata.linearVel);
				  Vin = arm_pid_f32(&PID2, ref_v - QEIdata.linearVel);
			  }
			  if(Vin > 24){
	  			  Vin = 24;
			  }
			  if(Vin < -24){
				  Vin = -24;
			  }
		  }
	      else if(mode == 2){ //manual (control with joy stick)
	    	  LED_Manual();
			  JoystickInput();
			  button_up_down_input();
			  button_reset_input(); //set 0;
			  button_save_position();
		  }
		  else if(mode == 3){ //stop mode
			  Vin = Vin_force;
		  }
		  else if(mode == 4){ //Emergency mode
			  Vin = 0;
			  LED_Emergency();

			  if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 1){
				  mode = 2;
				  emergency_reset = 1;
			  }
		  }

		  if(mode == 1){
			  if(QEIdata.linearPos < -0.5 || QEIdata.linearPos > 450){
				  Vin = 0;
			  }
		  }
		  else{
			  //software limit
			  SoftwareLimit();
		  }

		  //Drive Motor which PWM
		  DriveMotor();

	  }

	  if(currentTime > timestamp){
		  timestamp = currentTime + 4000; //250 Hz

		  //control mode
		  if(mode == 1){ //auto
			  if(fabs(setPosition - QEIdata.linearPos) <= 5 || setPosition == 0){
				  Vin = 0;
//				  if (setPosition != 0)
//				  {
//					  mode = 3;
//				  }
			  }
//			  else if(setPosition - QEIdata.linearPos <= 5 && setPosition - QEIdata.linearPos > 0.1){
//				  Vin = 2.575;
//				  check = 1;
//			  }
//			  else if(setPosition - QEIdata.linearPos >= -5 && setPosition - QEIdata.linearPos < -0.1){
//				  Vin = -1.5;
//				  check = -1;
//			  }
			  else{
				  //setVelocity = arm_pid_f32(&PID1, (setPosition + ref_p)/2 - QEIdata.linearPos);
				  //Vin = arm_pid_f32(&PID2, (setVelocity + ref_v)/2 + QEIdata.linearVel);
				  Vin = arm_pid_f32(&PID2, ref_v - QEIdata.linearVel);
			  }

			  if(Vin > 24){
				  Vin = 24;
			  }
			  if(Vin < -24){
				  Vin = -24;
			  }

			  if(QEIdata.linearPos < -0.5 || QEIdata.linearPos > 450){
			 	  Vin = 0;
			  }

			  //Drive Motor which PWM
			  DriveMotor();
		  }

	  }

	  if((currentTime > timestamp_savestate)&& (trigger_savestate == 1)){
		   S_top = S_top_savestate;
		   S_down = S_down_savestate;
		   mode = mode_savestate;
		   trigger_savestate = 0;
		   check_noise += 1;
	  }

	  //Check Emergency Status
//	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 1){
//		  LED_Emergency();
//	  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 169;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 169;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
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
  htim6.Init.Prescaler = 1699;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 20000;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim16, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC1 PC4 PC5
                           PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA7 PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//MicroSecondTimer Implement
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim5)
	{
		_micros += 1;
	}
//	 Check which version of the timer triggered this callback and toggle LED
	if (htim == &htim6)
	{
	    //check2 +=1;
		Heartbeat();
		Routine();
	}
}

uint64_t micros()
{
	return __HAL_TIM_GET_COUNTER(&htim5)+((_micros - 1) * 2,147,483,647);
}

void QEIEncoderPosVel_Update()
{
	//collect data
	QEIdata.TimeStamp[NEW] = micros();
	QEIdata.Position[NEW] = __HAL_TIM_GET_COUNTER(&htim2);
	//Postion 1 turn calculation
	QEIdata.QEIPostion_1turn = QEIdata.Position[NEW] % 8192;
	//calculate dx
	int32_t diffPosition = QEIdata.Position[NEW] - QEIdata.Position[OLD];
	//Handle Warp around
	if(diffPosition > UINT32_MAX/2)
		diffPosition -=UINT32_MAX;
	if(diffPosition < -UINT32_MAX/2)
		diffPosition +=UINT32_MAX;
	//calculate dt
	float diffTime = (QEIdata.TimeStamp[NEW]-QEIdata.TimeStamp[OLD]) * 0.000001; //sec
	//calculate anglar velocity
	QEIdata.QEIAngularVelocity = diffPosition / diffTime;  //pulse/sec

	QEIdata.rad_s = (QEIdata.QEIAngularVelocity*2*3.14)/8192;
	QEIdata.rpm = (QEIdata.QEIAngularVelocity*60)/8192;
	QEIdata.linearVel = (QEIdata.QEIAngularVelocity*10)/8192; //velocity (mm/s)
	QEIdata.Velocity[NEW] = QEIdata.linearVel;
	QEIdata.linearAcc = (QEIdata.Velocity[NEW]-QEIdata.Velocity[OLD])/diffTime;


	if(QEIdata.Position[NEW] < 2147483648){
		QEIdata.linearPos = ((float)QEIdata.Position[NEW]*10)/8192; //position (mm)
	}
	else{
		QEIdata.linearPos = -((float)(4294967295 - QEIdata.Position[NEW])*10)/8192;
	}

	//store value for next loop
	QEIdata.Position[OLD] = QEIdata.Position[NEW];
	QEIdata.TimeStamp[OLD]=QEIdata.TimeStamp[NEW];
	QEIdata.Velocity[OLD] = QEIdata.Velocity[NEW];
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	if(GPIO_Pin == GPIO_PIN_10)  //change mode IT
//	{
//		mode += 1;
//		if(mode==4){
//			mode = 1;
//		}

//		if(mode == 1){
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
//		}
//		else if(mode == 2){
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
//		}
//	}

	if(GPIO_Pin == GPIO_PIN_8){ //check top sensor
		if (start_IT > 11 )
		{
			S_top = 1;
		}

		//Vin = -2;
//		DriveMotor();
	}
	if(GPIO_Pin == GPIO_PIN_9){ //check down sensor

		if (start_IT > 11)
		{
			S_down = 1;
		}
		//Vin = 2;
//		DriveMotor();
	}
//	if(GPIO_Pin == GPIO_PIN_10){ //check emergency
//
//		if (start_IT > 11)
//		{
//			mode = 4; //Emergency mode
//			Vin = 0;
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
//		}
//	}
	start_IT += 1;
}

//-----------------------------------Trajectory------------------------------------------//
void checkStartMoving(){
	if(q_f != q_f_old){
		q_i = q_f_old;   //Define q_i
		delta_q = q_f - q_i;
		if(delta_q < 0){
			dir = -1;
		}
		else{
			dir = 1;
		}

		float q_check = fabs(q_d_i + q_2d_max*sqrt(fabs(delta_q)/q_2d_max)*dir);

		if(q_check >= q_d_max){ //Accelerate to maximum speed
			calmode = 1;
			t_acc = fabs((q_d_max-q_d_i)/q_2d_max);
			q_acc = q_d_i*t_acc*dir + 0.5*q_2d_max*t_acc*t_acc*dir;
			t_const = fabs((q_f-q_i-2*q_acc)/q_d_max);
			q_const = q_acc + q_d_max*t_const*dir;
			total_t = 2*t_acc + t_const;
		}
		else{ //Accelerating does not reach maximum speed
			calmode = 2;
			t_acc = sqrt(fabs(delta_q)/q_2d_max);
			q_acc = q_d_i*t_acc*dir + 0.5*q_2d_max*t_acc*t_acc*dir;
			q_d_acc = q_d_i + q_2d_max*t_acc*dir;
			total_t = 2*t_acc;
		}
		T_start = micros();
		q_f_old = q_f;
	}
}

void createTrajectory(){
	if(calmode == 1){
		t = (float)(micros()-T_start)/1000000; //sec
		if((0 <= t) && (t < t_acc)){ //Acceleration Segment
			ref_p = q_i + q_d_i*t*dir + 0.5*q_2d_max*t*t*dir;
			ref_v = q_d_i + q_2d_max*t*dir;
			ref_a = q_2d_max*dir;
		}
		else if((t_acc <= t) && (t < t_acc+t_const)){ //Constant Velocity Segment
			ref_p = q_i + q_acc + q_d_max*(t-t_acc)*dir;
			ref_v = q_d_max*dir;
			ref_a = 0;
		}
		else if((t_acc+t_const <= t) && (t < 2*t_acc+t_const)){ //Deceleration Segment
			ref_p = q_i + q_const + q_d_max*(t-t_acc-t_const)*dir-0.5*q_2d_max*(t-t_acc-t_const)*(t-t_acc-t_const)*dir;
			ref_v = -q_2d_max*(t-t_acc-t_const)*dir + q_d_max*dir;
			ref_a = -q_2d_max*dir;
		}
		else{
			calmode = 3;
		}
	}
	else if(calmode == 2){
		t = (float)(micros()-T_start)/1000000; //sec
		if((0 <= t) && (t < t_acc)){ //Acceleration Segment
			ref_p = q_i + q_d_i*t*dir + 0.5*q_2d_max*t*t*dir;
			ref_v = q_d_i + q_2d_max*t*dir;
			ref_a = q_2d_max*dir;
		}
		else if((t_acc <= t) && (t < t_acc*2)){
			ref_p = q_i + q_acc + q_d_acc*(t-t_acc) - 0.5*q_2d_max*(t-t_acc)*(t-t_acc)*dir;
			ref_v = q_d_acc - q_2d_max*(t-t_acc)*dir;
			ref_a = -q_2d_max*dir;
		}
		else{
			calmode = 3;
		}
	}
	else if(calmode == 3){ //stop
		ref_p = q_f;
		ref_v = 0;
		ref_a = 0;
	}
}

//--------------------------Drive Motor----------------//

void DriveMotor(){

	//if((Vin_old < 0.3 && Vin_old>-0.3) && (Vin >= 0.3 || Vin_old <= -0.3))
//	if(Vin_old == 0 && Vin !=0 )
//	{
//		trigger_savestate = 1;
//		S_top_savestate = S_top;
//		S_down_savestate = S_down;
//		mode_savestate = mode;
//		timestamp_savestate = micros() + 50000;
//	}
	//PWM Motor
	duty_cycle = (fabs(Vin)*100)/24;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_cycle*9.99);

	//Control Motor Direction
	if(Vin < 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
	}
//	Vin_old = Vin;
}

void SoftwareLimit(){
	if(S_top == 1 && Vin >= -2){
		Vin = 0;
	}
	else if(S_top == 1 && Vin < -2){
		S_top = 0;
	}

	if(S_down == 1 && Vin <= 2){
		Vin = 0;
	}
	else if(S_down == 1 && Vin > 2){
		S_down = 0;
	}

}


//------------------------------------Joy---------------------------------------//

void JoystickInput(){
	//Control y-axis by joy
	Vin = (float)(ADCBuffer[1]-1850)*24/2048; //0->24V
	if(Vin > -3 && Vin < 3){
		Vin = 0;
	}

	//Control x-axis by joy
	x_position += ((ADCBuffer[0]-2048)*0.005);
}

void button_up_down_input(){
	//check button up
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == 1){
		set_manual_point = QEIdata.linearPos + 10; //stem 10 mm
		check_up = 1;
		B_up = 1;
		check_state_B = 1;
	}
	else{
		B_up = 0;
		check_state_B = 2;
	}
	if(QEIdata.linearPos < set_manual_point && check_up == 1){
		Vin = 4;
		check_state_B = 3;
	}
	else{
		check_up = 0;
		check_state_B = 4;
	}

	//check button down
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 1){
		set_manual_point = QEIdata.linearPos - 10;
		check_down = 1;
		B_down = 1;
		check_state_B = 5;
	}
	else{
		B_down = 0;
		check_state_B = 6;
	}
	if(QEIdata.linearPos > set_manual_point && check_down == 1){
		Vin = -3;
		check_state_B = 7;
	}
	else{
		check_down = 0;
		check_state_B = 8;
	}

	if(check_up == 0 && check_down == 0){
		Vin = 0;
	}
}

void button_reset_input(){
	//check button reset
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 1 || (Home_state_triger == 0 && set_Home_state == 1) || emergency_reset == 1){
		B_reset = 1;
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 0){
			Modbus_Protocal_Worker();
			Vin = -3.5;
			//software limit
			SoftwareLimit();
			//Drive Motor which PWM
			DriveMotor();
		}

		Vin = 2.1;
		DriveMotor();
		HAL_Delay(1000);
		SoftwareLimit();

		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 0){
			Modbus_Protocal_Worker();
			Vin = -1.3;
			//software limit
			SoftwareLimit();
			//Drive Motor which PWM
			DriveMotor();
		}
		Vin = 0;
		__HAL_TIM_SET_COUNTER(&htim2, 0); //set position to 0

		if(Home_state_triger == 0 && set_Home_state == 1){
			Home_state_triger = 1;
			B_reset = 0;
		}

		if(emergency_reset == 1){
			emergency_reset = 0;

		}
	}
	else{
		B_reset = 0;
	}
}

void button_save_position(){
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1 && B_save == 0 && fabs(QEIdata.linearPos-last_shelf_save) > 20){
		B_save = 1;
		count_save += 1;
		shelves_pos[count_save] = QEIdata.linearPos;
		last_shelf_save =  QEIdata.linearPos;
	}
	else{
		B_save = 0;
	}
}

//-------------------------------------------------//
//-------------------------MODBUS FUNCTIONS--------------------------//
void Heartbeat(){
	registerFrame[0x00].U16 = 22881;
}

void Routine(){
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0){
			gripper_status = 0;
		}
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1){
			gripper_status = 1;
		}
		if (gripper_status == 0 ) // not suck  in
		{
			registerFrame[0x04].U16 = 0b0000;
			registerFrame[0x04].U16 = 0b0001;
		}
		else if (gripper_status == 1) // suck in
		{
			registerFrame[0x04].U16 = 0b0000;
			registerFrame[0x04].U16 = 0b0010;
		}
		registerFrame[0x10].U16 = Moving_status;
		registerFrame[0x11].U16 = QEIdata.linearPos*10;
		registerFrame[0x12].U16 = QEIdata.linearVel;
		registerFrame[0x13].U16 = QEIdata.linearAcc;
		registerFrame[0x40].U16 = x_position;
}

void check_vaccum_status()
{
	check_state = 1;
	if(registerFrame[0x02].U16 == 0b0000){ // not suck  in

		vaccum_status = 0;
		if (prev_vac_savestate != vaccum_status)
			{
				trigger_savestate = 1;
				S_top_savestate = S_top;
				S_down_savestate = S_down;
				mode_savestate = mode;
				timestamp_savestate = micros() + 500000;
			}
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	}
	if(registerFrame[0x02].U16 == 0b0001){ //suck  in
		vaccum_status = 1;
		if (prev_vac_savestate != vaccum_status)
			{
				trigger_savestate = 1;
				S_top_savestate = S_top;
				S_down_savestate = S_down;
				mode_savestate = mode;
				timestamp_savestate = micros() + 500000;
			}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	}

	prev_vac_savestate = vaccum_status;
}

void check_gripper_status()
{
	check_state = 2;

	if(registerFrame[0x03].U16 == 0b0000) //backward ,not on
	//if(test_Gripper == 0)
	{

		gripper_status = 0;

		if (prev_grp_savestate != gripper_status)
			{
					trigger_savestate = 1;
					S_top_savestate = S_top;
					S_down_savestate = S_down;
					mode_savestate = mode;
					timestamp_savestate = micros() + 500000;
			}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
	}
	if(registerFrame[0x03].U16 == 0b0001) //Forward
	//if(test_Gripper == 1)
	{
		gripper_status = 1;

		if (prev_grp_savestate != gripper_status)
			{
					trigger_savestate = 1;
					S_top_savestate = S_top;
					S_down_savestate = S_down;
					mode_savestate = mode;
					timestamp_savestate = micros() + 500000;
			}

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
	}

	prev_grp_savestate = gripper_status;
}

void set_shelf()
{
	check_state = 3;
	if(registerFrame[0x01].U16 ==  0b0001)
	{
		set_shelf_state = 1;
		LED_Manual();
	}
	if (set_shelf_state == 1 && Enter_click == 0)
	{
			registerFrame[0x01].U16 = 0b0000;
			Moving_status = 1;
			registerFrame[0x10].U16 = Moving_status; // set shelve
			mode = 2; //manual
			if(count_save == 5){
				Enter_click = 1;
				count_save = 0;
				mode = 3;
			}
	}
	if (set_shelf_state == 1 && Enter_click == 1)
	{
		registerFrame[0x01].U16 = 0b0000;
		registerFrame[0x23].U16 = shelves_pos[1]*10;
		registerFrame[0x24].U16 = shelves_pos[2]*10;
		registerFrame[0x25].U16 = shelves_pos[3]*10;
		registerFrame[0x26].U16 = shelves_pos[4]*10;
		registerFrame[0x27].U16 = shelves_pos[5]*10;
		Moving_status = 0;
		registerFrame[0x10].U16 = Moving_status;
		set_shelf_state = 0;
		Enter_click = 0;
		LED_Ready();
	}

}

void Pointmode()
{
	if(registerFrame[0x01].U16 ==  0b1000) // Run Point Mode
		{
			 set_point_state = 1;
			 set_point_modbus = (registerFrame[0x30].U16) / 10;
			 registerFrame[0x01].U16 = 0b0000;
			 LED_Auto();
//			 Modbus_Protocal_Worker();
		}

	if (point_state_triger == 0 && set_point_state == 1)
		{
				Moving_status = 16;
				registerFrame[0x10].U16 = Moving_status;
				mode = 1;
				setPosition = set_point_modbus;
				if(fabs(setPosition-QEIdata.linearPos) <= 5){
					mode = 3;
					point_state_triger = 1;
				}
				Modbus_Protocal_Worker();
		}
	if (point_state_triger == 1 && set_point_state == 1)
		{
			mode = 3;
			Moving_status = 0;
			registerFrame[0x10].U16 = Moving_status;
			set_point_state = 0;
			point_state_triger = 0;
			Modbus_Protocal_Worker();
			LED_Ready();
//
		}
}
void Home()
{
	if(registerFrame[0x01].U16 ==  0b0010) // Run Home Mode
	{
		set_Home_state = 1;
		registerFrame[0x01].U16 = 0b0000;
		LED_Homing();
//		Modbus_Protocal_Worker();
	}
	if(Home_state_triger == 0 && set_Home_state == 1)
	{
		Moving_status = 2;
		registerFrame[0x10].U16 = Moving_status;
		//set home
		mode = 2;
//		Modbus_Protocal_Worker();
	}
	if(Home_state_triger == 1 && set_Home_state == 1)
	{
		mode = 3;
		Moving_status = 0;
		registerFrame[0x10].U16 = Moving_status;
		set_Home_state = 0;
		Home_state_triger = 0;
//		Modbus_Protocal_Worker();
		LED_Ready();
	}
}
void Run_jog()
{
	if(registerFrame[0x01].U16 ==  0b0100) // Run Jog Mode
	{
		set_jog_state = 1;
		Jog_state_triger = 0;
		Jog_oneloop_trigger = 0;
		Jog_order = 0;

		PickOrder = registerFrame[0x21].U16;
		PlaceOrder = registerFrame[0x22].U16;

		PickArray[0] = (PickOrder - (PickOrder % 10000)) / 10000;
		PickArray[1] = ((PickOrder % 10000) - (PickOrder % 1000)) / 1000;
		PickArray[2] = ((PickOrder % 1000) - (PickOrder % 100)) / 100;
		PickArray[3] = ((PickOrder % 100) - (PickOrder % 10)) / 10;
		PickArray[4] = PickOrder % 10;

		PlaceArray[0] = (PlaceOrder - (PlaceOrder % 10000)) / 10000;
		PlaceArray[1] = ((PlaceOrder % 10000) - (PlaceOrder % 1000)) / 1000;
		PlaceArray[2] = ((PlaceOrder % 1000) - (PlaceOrder % 100)) / 100;
		PlaceArray[3] = ((PlaceOrder % 100) - (PlaceOrder % 10)) / 10;
		PlaceArray[4] = PlaceOrder % 10;

		registerFrame[0x01].U16 = 0b0000;
		LED_Auto();

	}
	if (Jog_state_triger == 0 && set_jog_state == 1){

		// run 5 loop of pick and place

		if(Jog_oneloop_trigger == 0){ //Go pick
			Moving_status = 4; //Go pick
			registerFrame[0x10].U16 =  Moving_status;

			mode = 1;
			setPosition = shelves_pos[PickArray[Jog_order]]; //Update set point

			if(fabs(setPosition-QEIdata.linearPos) < 0.05){
					mode = 3;
					__HAL_TIM_SET_COUNTER(&htim2, 0);

					//pick
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
					HAL_Delay(500);

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
					HAL_Delay(500);

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
					HAL_Delay(500);

					Jog_oneloop_trigger = 1;
			}
		}
		else if(Jog_oneloop_trigger == 1){ //Go place
			Moving_status = 8; //Go place
			registerFrame[0x10].U16 =  Moving_status;

			mode = 1;
			setPosition = shelves_pos[PlaceArray[Jog_order]]; //Update set point

			if(fabs(setPosition-QEIdata.linearPos) < 0.05){
					mode = 3;
					__HAL_TIM_SET_COUNTER(&htim2, 0);

					//place
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
					HAL_Delay(500);

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
					HAL_Delay(500);

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
					HAL_Delay(500);

					Jog_order += 1;
					Jog_oneloop_trigger = 0;
			}
		}

		if(Jog_order == 5){
			mode = 3;
			Jog_state_triger = 1;
			Jog_order = 0;
		}
	}

	if (Jog_state_triger == 1 && set_jog_state == 1)
	{
		Moving_status = 0;
		registerFrame[0x10].U16 =  Moving_status;
		Jog_state_triger = 0;
		set_jog_state = 0;
		LED_Ready();
	}
}
//-----------------------------------------------------------//

//-----------------------LED STATUS--------------------------//

void LED_Emergency(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
}

void LED_Homing(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
}

void LED_Ready(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
}

void LED_Auto(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
}

void LED_Manual(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
}

//-----------------------------------------------------------//

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
