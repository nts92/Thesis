/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*-----Variables for controller-----*/
double T = 0.01;
double ga=0.000005;
const double set1z = 50;
double set1 = 0;
int16_t N1=0, preN1;
double y11, y12, y13, y14, ym11, ym12, ym13;
double ec11, ec12, ec13, em11, em12, em13;
double delkp11, delkp12, delkp13;
double delki11, delki12, delki13;
double Kp1, Ki1;
double gammap1 = 0.000009, gammai1 = 0.000009;
double alpha1, beta1;
double u11, u12;

const double set2z = 50;
double set2 = 0;
int16_t N2, preN2;
double y21, y22, y23, y24, ym21, ym22, ym23;
double ec21, ec22, ec23, em21, em22, em23;
double delkp21, delkp22, delkp23;
double delki21, delki22, delki23;
double Kp2, Ki2;
double gammap2 = 0.000009, gammai2 = 0.000009;
double alpha2, beta2;
double u21, u22;
int time_parameter = 0;

/////*-----Variables for UART2-----*/
//#define UART2_RX_BUFF_SIZE 20
//#define UART2_TX_BUFF_SIZE 30
//char rx2_data[2*UART2_RX_BUFF_SIZE];
//int rx2_indx;
//char tx2_data[2*UART2_TX_BUFF_SIZE] = "+00.00/+00.00/+000.00/+000.00\n";

/*-----Variables for UART2-----*/
#define UART2_BUFF_SIZE 20
char rx2_data[2*UART2_BUFF_SIZE], rx2_buffer[UART2_BUFF_SIZE];
int rx2_indx;
int send_enable = 0;
char tx2_data[29] = "connected\n";
char a_data[29];
int flat0 = 0, flat1 = 0;

/*-----Variables for controller-----*/
double leftWheel;
double rightWheel;
uint8_t rxBuffer[64]; 

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == huart4.Instance)
//	{				
//		flat1 = 1;
//	}
//}

void data_processing(void)
	{
		HAL_UART_Receive_IT(&huart4,(uint8_t*)rx2_data,UART2_BUFF_SIZE);
		if(rx2_data[19] == 'k')
		{
			if((rx2_data[1] + rx2_data[2] + rx2_data[4] + rx2_data[5]  - 192 == (rx2_data[14] - 48)*10 + (rx2_data[15] - 48))
						&& (rx2_data[8] + rx2_data[9] + rx2_data[11] + rx2_data[12] - 192 == (rx2_data[17] - 48)*10 + (rx2_data[18] - 48)))
			{
				set1 = (double)(rx2_data[1] - 48)*10 + (double)(rx2_data[2] - 48) + (double)(rx2_data[4] - 48)/10 + (double)(rx2_data[5] - 48)/100;
				set2 = (double)(rx2_data[8] - 48)*10 + (double)(rx2_data[9] - 48) + (double)(rx2_data[11] - 48)/10 + (double)(rx2_data[12] - 48)/100;
				if(rx2_data[0] == '-') set1 = - set1;
				if(rx2_data[7] == '-')	set2 = - set2;	
			}
		}
		 rx2_data[19] = '0';
	}

	void  motor_run(void)
	{
			// Left wheel
	N1 = __HAL_TIM_GET_COUNTER(&htim4);
	if(abs((int)(N1 - preN1)) < 40000) y13 = (N1 - preN1)*60.0*100/4000;
	y11 = 0.9753*y12 + 0.02469*y14;
	y12 = y11; y14 = y13;
	preN1 = N1;
	
	ec11 = set1 - y11;
	ym11 = 0.00122*set1 + 0.00078*set1 + 1.921*ym12 - 0.923*ym13;
	em11 = y11 - ym11;
	delkp11 = 1.921*delkp12 - 0.923*delkp13 + 0.07685*ec12 - 0.07685*ec13;
	Kp1 += -gammap1*em11*delkp11;
	delki11 = 1.921*delki12 - 0.923*delki13 + 0.00039*ec12 + 0.00025*ec13;
	Ki1 += -gammai1*em11*delki11;
	
	alpha1 = Kp1*(ec11 - ec12);
	beta1 = T/2*Ki1*(ec11 + ec12);
	u11 = u12 + alpha1 + beta1;
	
	if(u11 < 0)
	{
		if(u11 < -60) u11 = -60;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -u11);
	}
	else
	{
		if(u11 > 60) u11 = 60;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, u11);
	}

	if(set2 == 0) u11 = 0;
	
	/*if((u11-u12) > 300){ 
		u11 = 100;
	}
	if((u11-u12) < -300){
		u11 = -100;
	}*/
	
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, u11);
	
	u12 = u11;
	ec13 = ec12; ec12 = ec11; em13 = em12; em12 = em11;	ym13 = ym12; ym12 = ym11;
	delkp13 = delkp12; delkp12 = delkp11; delki13 = delki12; delki12 = delki11;
	
	// Right wheel
	N2 = __HAL_TIM_GET_COUNTER(&htim3);
	if(abs((int)(N2 - preN2)) < 40000) y23 = (N2 - preN2)*60.0*100/4000;
	y21 = 0.9753*y22 + 0.02469*y24;
	y22 = y21; y24 = y23;
	preN2 = N2;
	
	ec21 = set2 - y21;
	ym21 = 0.00122*set2 + 0.00078*set2 + 1.921*ym22 - 0.923*ym23;
	em21 = y21 - ym21;
	delkp21 = 1.921*delkp22 - 0.923*delkp23 + 0.07685*ec22 - 0.07685*ec23;
	Kp2 += -gammap2*em21*delkp21;
	delki21 = 1.921*delki22 - 0.923*delki23 + 0.00039*ec22 + 0.00025*ec23;
	Ki2 += -gammai2*em21*delki21;
	
	alpha2 = Kp2*(ec21 - ec22);
	beta2 = T/2*Ki2*(ec21 + ec22);
	u21 = u22 + alpha2 + beta2;
	
	if(u21 < 0)
	{
		if(u21 < -60) u21 = -60;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -u21);
	}
	else
	{
		if(u21 > 60) u21 = 60;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, u21);
	}

	if(set2 == 0) u21 = 0;
	
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, u21);
	
	u22 = u21;
	ec23 = ec22; ec22 = ec21; em23 = em22; em22 = em21;	ym23 = ym22; ym22 = ym21;
	delkp23 = delkp22; delkp22 = delkp21; delki23 = delki22; delki22 = delki21;
	
	// Sending speed data through UART2
	int temp1 = y11*100;
	int temp2 = y21*100;
	
	tx2_data[0] = (temp1 >= 0)? '+' : '-';
	tx2_data[7] = (temp2 >= 0)? '+' : '-';
	
	temp1 = abs(temp1); temp2 = abs(temp2);
	
//	tx2_data[1] = (uint8_t)(temp1/1000) + 48;
//	tx2_data[2] = (uint8_t)((temp1 - 1000*(tx2_data[1] - 48))/100) + 48;
//	tx2_data[4] = (uint8_t)((temp1 - 1000*(tx2_data[1] - 48) - 100*(tx2_data[2] - 48))/10) + 48;
//	tx2_data[5] = (uint8_t)(temp1%10) + 48;
//	
//	tx2_data[8] = (uint8_t)(temp2/1000) + 48;
//	tx2_data[9] = (uint8_t)((temp2 - 1000*(tx2_data[8] - 48))/100) + 48;
//	tx2_data[11] = (uint8_t)((temp2 - 1000*(tx2_data[8] - 48) - 100*(tx2_data[9] - 48))/10) + 48;
//	tx2_data[12] = (uint8_t)(temp2%10) + 48;
	a_data[1] = (uint8_t)(temp1/1000) +48;
	a_data[2] = (uint8_t)((temp1 - 1000*(a_data[1] - 48))/100) +48;
	
	a_data[4] = (uint8_t)((temp1 - 1000*(a_data[1] - 48) - 100*(a_data[2] - 48))/10)+48;
	a_data[5] = (uint8_t)(temp1%10) +48;
	
	a_data[8] = (uint8_t)(temp2/1000)+48;
	a_data[9] = (uint8_t)((temp2 - 1000*(a_data[8] - 48))/100)+48;
	a_data[11] = (uint8_t)((temp2 - 1000*(a_data[8] - 48) - 100*(a_data[9] - 48))/10)+48;
	a_data[12] = (uint8_t)(temp2%10)+48;

	
	
	tx2_data[1] = (uint8_t)(temp1/1000) +48;
	tx2_data[2] = (uint8_t)((temp1 - 1000*(tx2_data[1] - 48))/100) +48;
	tx2_data[3]= '.';
	tx2_data[4] = (uint8_t)((temp1 - 1000*(tx2_data[1] - 48) - 100*(tx2_data[2] - 48))/10)+48;
	tx2_data[5] = (uint8_t)(temp1%10) +48;
	tx2_data[6]= '/';
	tx2_data[8] = (uint8_t)(temp2/1000)+48;
	tx2_data[9] = (uint8_t)((temp2 - 1000*(tx2_data[8] - 48))/100)+48;
	tx2_data[10]= '.';
	tx2_data[11] = (uint8_t)((temp2 - 1000*(tx2_data[8] - 48) - 100*(tx2_data[9] - 48))/10)+48;
	tx2_data[12] = (uint8_t)(temp2%10)+48;
	//////////////////////////////////
	tx2_data[13]= '/';
	tx2_data[14] = '0';
	tx2_data[15] = '0';
	tx2_data[16] = '0';

	tx2_data[17] = '0';
	tx2_data[19] = '0';
	tx2_data[20] = '0';
	
	tx2_data[21] = '/';
	
	tx2_data[22] = '0';
	tx2_data[23] = '0';
	tx2_data[24] = '0';
	tx2_data[25] = '.';
	tx2_data[26] = '0';
	tx2_data[27] = '0';
	tx2_data[28] = '\n';
	////////
	
	HAL_UART_Transmit(&huart4, (uint8_t*)tx2_data, sizeof(tx2_data), 1000);		
	time_parameter++;
	flat0 = 0;
	}
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
	flat0 = 1;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_UART_Receive_DMA(&huart4,(uint8_t*)rx2_data,20);

	//HAL_UART_Receive_DMA(&huart4,rxBuffer,40);
	//HAL_UART_Receive_IT(&huart2,(uint8_t*)rx2_data,1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (flat0 == 1)
			{
				motor_run();
			}
		if (rx2_data[19] == 'k' )
			{
				data_processing();
			}
		HAL_Delay(10);
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
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 42;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim3.Init.Period = 0xffff-1;
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
  htim4.Init.Period = 0xffff-1;
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
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pins : PC14 PC15 PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

