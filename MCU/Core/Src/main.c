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
#include "PS2/PS2.h"
#include "i2c-lcd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
PS2Buttons PS2;
int mode1=0, mode2=0;
int read=0X00000000;
uint16_t kc1, kc2;
int memory, value, new_value,P=0 ,I=0,D=0 , PID_value;
float  Ki=0.4, Kd=15, Kp=1;
float speedr, speedl;
	const uint16_t maxspeedr = 1000;
	const uint16_t maxspeedl = 1000;
	const uint16_t basespeedr = 900;
	const uint16_t basespeedl = 900;
	int last_P;
	int cua=0;
	int vat_can=0;
	int s1, s2, s3, tt;
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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) 
{ 
		if (hadc-> Instance==ADC1)
	 {kc1=HAL_ADC_GetValue(&hadc1);
	 }
		if (hadc-> Instance==ADC2)
	 {kc2=HAL_ADC_GetValue(&hadc2);
	 }
	 	 
	} 
uint16_t read_line(void)
	{     /* USER CODE END WHILE */
  /* USER CODE END WHILE */
if(HAL_GPIO_ReadPin(Line1_GPIO_Port, Line1_Pin)==1){
	read |=0x00000001;
}
else{read&=0x11111110;}

if(HAL_GPIO_ReadPin(Line2_GPIO_Port, Line2_Pin)==1){
	read |=0x00000010;
}
else{read&=0x11111101;}
if(HAL_GPIO_ReadPin(Line3_GPIO_Port, Line3_Pin)==1){
	read|=0x00000100;
}
else{read&=0x11111011;}
if(HAL_GPIO_ReadPin(Line4_GPIO_Port, Line4_Pin)==1){
	read |=0x00001000;
}
else{read&=0x11110111;}
if(HAL_GPIO_ReadPin(Line5_GPIO_Port, Line5_Pin)==1){
	read |=0x00010000;
}
else{read&=0x11101111;}
return read;
HAL_Delay(20);

		}
void speed(uint16_t T, uint16_t P)
{ 
	htim3.Instance->CCR1=(P-50);
	htim4.Instance->CCR4=(T-50);
	}
void Go(void)
	{ 		 HAL_GPIO_WritePin(DT_GPIO_Port, DT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(XT_GPIO_Port, XT_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(XP_GPIO_Port, XP_Pin, GPIO_PIN_SET);
		}
void Back(void)
	{			 HAL_GPIO_WritePin(DT_GPIO_Port, DT_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(XT_GPIO_Port, XT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(XP_GPIO_Port, XP_Pin, GPIO_PIN_RESET);
		}
void Left(void)
	{			 HAL_GPIO_WritePin(DT_GPIO_Port, DT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(XT_GPIO_Port, XT_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(XP_GPIO_Port, XP_Pin, GPIO_PIN_RESET);
		}
 void Right(void)
	 {		 HAL_GPIO_WritePin(DT_GPIO_Port, DT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(XT_GPIO_Port, XT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(XP_GPIO_Port, XP_Pin, GPIO_PIN_SET);
		 }
	void Left2(void)
	{			 HAL_GPIO_WritePin(DT_GPIO_Port, DT_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(XT_GPIO_Port, XT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(XP_GPIO_Port, XP_Pin, GPIO_PIN_SET);
	  }
	void Right2(void)
	{      HAL_GPIO_WritePin(DT_GPIO_Port, DT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(XT_GPIO_Port, XT_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(XP_GPIO_Port, XP_Pin, GPIO_PIN_RESET);
	   }
		
  void Stop(void)
		{    HAL_GPIO_WritePin(DT_GPIO_Port, DT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(XT_GPIO_Port, XT_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(XP_GPIO_Port, XP_Pin, GPIO_PIN_RESET);
			}
void read_value(void)
{ if (read==0)
	{value=-5;
	  }
	else if(read==1)
	{value=4;
		}
	else if(read==17)
	{value=3;
		}
	else if(read==16)
	{value=2;
		}
	else if(read==272)
	{value=1;
		}
	else if(read==256)
	{value=0;
		}
	else if(read==4352)
	{value=-1;
		}
	else if(read==4096)
	{value=-2;
		}
	else if(read==4352)
	{value=-1;
		}
	else if(read==4096)
	{value=-2;
		}
		else if(read==69632)
	{value=-3;
		}
	else if(read==65536)
	{value=-4;
		}
	}
void algorithm(void)
	{ read_value();
		switch (read)
		{
		 case 0:
			{ 
				if(memory==0)
				{	
				 speed(700,1000); 			
				 Right2();
				}
				else
				{          
				 speed(1000,700); 
				 Left2();                 
         }
			}
		 break;
		 case 0x00000111:
			{ if(cua==1){
				speed(750,700);
		  	}
			  else{
				speed(900,700);}
		  	Right2();
				memory=0;
				
				}
		 break; 
		 case 0x00000001:
			{ if(cua==1){
				speed(750,700);
		  	}
			  else{
				speed(900,700);}
		  	Right2();
				memory=0;
				
				}
		 break;
		 case 0x00000011:
			{ if(cua==1){
				speed(750,700);}
				else{
				speed(900,700);}
				 Right2();
				memory=0;
		
				}
		 break;
		 case 0x00000010:
			{ if(cua==1){
				speed(750,700);}
			  else{
				speed(800,700);} 
				 Right2();
				memory=0;
			
			}
		 break;
		 case 0x00000110:
			{ if(cua==1){
				speed(750,700);}
			  else{
				speed(800,700);} 
				 Right2();
			
				}
		 break;
		 case 0x00000100:
			{ if(cua==1)
				{speed(750,750);}
				else{
				speed(800,800);} 
				 Go();
				}
		 break;
		 case 0x00001100:
			{  if(cua==1)
				{speed (700,800);}
         else{
					speed(700,850);
         }					 
				 Left2();				
			}
			break;		
		 case 0x00001000:
			{  if(cua==1)
				{speed(700,750);}
        else{speed(700,850);}				
         Left2();
				 memory=1;
			}
		 break;
		 case 0x00011000:
			{ if(cua==1)
        {speed(700,750);}				
				else
				{speed(700,900);}
				 Left2();
				 memory=1;
				
			}
		 break;	
		 case 0x00010000:
		 case 0x00011100:
			{  if(cua==1)
				 {speed(700,950);}
         else{speed(700,1000);}				 
         Left2();
				 memory=1;
			
			}
		  break;						
     case 0x00011111:
			{ if(cua==0)
				{speed(1000,1000); 
         Go();}
				else
				{speed(650,1000);
					Right2();
					cua=0;
		HAL_Delay(400);
			  }
			}	
		  break;						
		 
     case 0x00000101:
		 case 0x00001101:
		 case 0x00010101:
		 case 0x00010110:
		 case 0x00010100:
		 {Go();
			 speed(1000,1000);
			 cua=1;
		 }			 
	}
}		
			
	void pid(void)
{ 
	
	P=0-value;
	I+=P;
	D=value-last_P;
	PID_value=Kp*P+Ki*I+Kd*D;
	last_P=P;
	speedr=PID_value+basespeedr;
	speedl=PID_value+basespeedl;
	if(speedr>maxspeedr)
		{ speedr=maxspeedr;
			}
	if(speedl>maxspeedl)
		{ speedl=maxspeedl;
			}
	speed(speedr,speedl);	
 }	
void LCD(void)
	{lcd_init();
lcd_goto_XY(1,4);
lcd_send_string("QUOC HUY;");
HAL_Delay(30);

		}
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
	lcd_init();
  /* USER CODE BEGIN 2 */
	PS2_Init(&htim1, &PS2);  
HAL_ADC_Start_IT(&hadc1);
HAL_ADC_Start_IT(&hadc2);
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  /* USER CODE END 2 */

LCD();
htim2.Instance->CCR1=1500;
htim2.Instance->CCR3=1500;
htim2.Instance->CCR4=1500;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
		HAL_ADC_Start_IT(&hadc1);
		HAL_ADC_Start_IT(&hadc2);
    /* USER CODE END WHILE */
    /* USER CODE END WHILE */
PS2_Update();
 if(mode1==0)
  { speed(1000,1000);
		if (PS2.UP) {
		Go();
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  } else if (PS2.DOWN) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    Back();
	}
	else if (PS2.LEFT) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    Left();
	}
	else if (PS2.RIGHT) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    Right();
	} else if (PS2.L1){
	if(s1>=2450)  {
	 continue;
   }		 
	htim2.Instance-> CCR1=s1;
	s1=s1+2;
	}
  else if (PS2.L2){
	if(s1<=550)  {
		continue;
		}		
	htim2.Instance-> CCR1=s1;
	s1=s1-2;
	}	
	else if (PS2.L3){
  Left2();
	HAL_Delay(8);
	Stop();
  }  
	else if (PS2.R3){ 
	Right2();
	HAL_Delay(8);
	Stop();
	}
	else if (PS2.CROSS){	
	htim2.Instance-> CCR3=1650;
	HAL_Delay(10);
	htim2.Instance-> CCR3=1500;
	HAL_Delay(10);
	htim2.Instance-> CCR1=850;
	HAL_Delay(10);
	htim2.Instance-> CCR4=1650;
	htim2.Instance-> CCR3=2450;
  s3=2450;	
  s2=1650;		
	s1=850;
	}
	else if (PS2.TRIANGLE){	
	htim2.Instance-> CCR4=600;
	HAL_Delay(100);
	htim2.Instance-> CCR3=1750;
	htim2.Instance-> CCR1=1350;
		s3=1750;
		s2=600;
		s1=1350;
	}
  else if (PS2.R1){
	if(s3>=2450)  {
	 continue;
   }		 
	htim2.Instance-> CCR3=s3;
	s3=s3+4;
	}
  else if (PS2.R2){
	if(s3<=550)  {
		continue;
		}		
	htim2.Instance-> CCR3=s3;
	s3=s3-4;
	}	
	  else if (PS2.CIRCLE){
	if(s2>=2450)  {
	 continue;
   }		 
	htim2.Instance-> CCR4=s2;
	s2=s2+2;
	}
  else if (PS2.SQUARE){
	if(s2<=550)  {
		continue;
		}		
	htim2.Instance-> CCR4=s2;
	s2=s2-2;
	}
	else if (PS2.SELECT)
		{ if(mode1!=0)
			{continue;}
			mode1=~mode1;
			}
	else{
		Stop();
		}
	 
	}
	else
	{  if(PS2.START)
		{if(mode1==0)
			{continue;}
			mode1=~mode1;
			}
if(kc1>2500)		
	{ 
		Left();
		speed(1000,1000);
		HAL_Delay(300);
		Go();
		speed(1000,1000);
		HAL_Delay(300);
		Right();
		speed(1000,1000);
		HAL_Delay(750);
    Left();
		HAL_Delay(400);}
	else if(kc2>2500)
	 { Right();
		speed(1000,1000);
		HAL_Delay(300);
		Go();
		speed(1000,1000);
		HAL_Delay(300);
		Left();
		speed(1000,1000);
		HAL_Delay(750);
    Right();
		HAL_Delay(400);}
	else{
	read_line();
	algorithm();}
		}


		
	HAL_Delay(2);
    /* USER CODE BEGIN 3 */
 }
  /* USER CODE END 3 */
}

/**  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
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
  sConfigOC.Pulse = 0;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SS_Pin|SCK_Pin|MOSI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DP_Pin|XP_Pin|XT_Pin|DT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SS_Pin SCK_Pin MOSI_Pin */
  GPIO_InitStruct.Pin = SS_Pin|SCK_Pin|MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MISO_Pin Line1_Pin */
  GPIO_InitStruct.Pin = MISO_Pin|Line1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Line5_Pin Line4_Pin Line3_Pin Line2_Pin */
  GPIO_InitStruct.Pin = Line5_Pin|Line4_Pin|Line3_Pin|Line2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DP_Pin XP_Pin XT_Pin DT_Pin */
  GPIO_InitStruct.Pin = DP_Pin|XP_Pin|XT_Pin|DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
