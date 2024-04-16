/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define p2r pi/2000
#define pi 3.1415
#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint16_t data[5];
uint16_t average_sensor[5]={1961,1958,1959,1957,1959};
uint16_t digital[5];
uint16_t sensor=0;
int32_t rPosCnt,rCnttmp,rspeed;
int16_t rCountValue=0,rRealVel,rDesiredSpeed,rHILIM,rLOLIM;
uint16_t rAngPos1, rAngPos0,rCntVel;
uint8_t rPreviousState,rpwm,rSpeedmode;	
float rCurVel;	
float rVset;
float rCurrentPos;
short rroundCnt;
int32_t lPosCnt,lCnttmp,lspeed;
int16_t lCountValue=0,lRealVel,lDesiredSpeed,lHILIM,lLOLIM;
uint16_t AngPos1, AngPos0,lCntVel;
uint8_t lPreviousState,lpwm,lSpeedmode;	
float lCurVel;	
float lVset;
float lCurrentPos;
short lroundCnt;
float rGearVel;
float lGearVel;
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  
uint8_t Distance  = 0; 
uint32_t startTime = 0;
uint8_t count=0;
uint8_t battery=90;
uint8_t charge=15;
uint8_t previousSensorValue;
uint8_t previousSensor = 0;
uint32_t rightspeed ,leftspeed;
float rcurDis;
float rCurPos;
float rPosset;
float rtarget;
float lcurDis;
float lCurPos;
float lPosset;
float ltarget;
float lGearPos;
uint32_t i=0;
int pidEnabled = 1;
float turnleft=55;
float back=55;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Setup for follow line 
void follow_line(){
	 uint8_t i=0;
	 for (i = 0; i < 5; i++ )
		{
			if (data[i]<average_sensor[i])
				digital[i]=1;
			else digital[i]=0;
		}
		sensor=digital[0]+digital[1]*2+digital[2]*4+digital[3]*8+digital[4]*16;
		
 }


/*******************************Encoder for right wheel***************************************/
void EXTI9_5_IRQHandler(void)		
{
unsigned char rState0;
	rState0 = (rState0<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	rState0 = (rState0<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
	rState0 = rState0&0x03;
	switch (rState0) {
		case 0:
			if(rPreviousState==1) rCountValue++;
			else rCountValue--;
		break;
		case 1:
			if(rPreviousState==3) rCountValue++;
			else rCountValue--;
		break;
		case 2:
			if(rPreviousState==0) rCountValue++;
			else rCountValue--;
		break;
		case 3:
			if(rPreviousState==2) rCountValue++;
			else rCountValue--;
		break;
		}
	rPreviousState = rState0;
	rCntVel++;
	if (rCountValue>=48) {
		rCountValue = 0;
		rPosCnt++;
	}
	else if	(rCountValue<=-48) {
		rCountValue = 0;
		rPosCnt--;
	}
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
 
}


void EXTI2_IRQHandler(void)		
{
unsigned char rState1;
	rState1 = (rState1<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	rState1 = (rState1<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
	rState1 = rState1&0x03;
	switch (rState1) {
		case 0:
			if(rPreviousState==1) rCountValue++;
			else rCountValue--;
		break;
		case 1:
			if(rPreviousState==3) rCountValue++;
			else rCountValue--;
		break;
		case 2:
			if(rPreviousState==0) rCountValue++;
			else rCountValue--;
		break;
		case 3:
			if(rPreviousState==2) rCountValue++;
			else rCountValue--;
		break;
		}
	rPreviousState = rState1;
	rCntVel++;
	if (rCountValue>=48) {
		rCountValue = 0;
		rPosCnt++;
	}
	else if	(rCountValue<=-48 ){
		rCountValue = 0;
		rPosCnt--;
	}
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}
/*********************************************************************************************/


/*******************************Encoder for left wheel***************************************/
void EXTI0_IRQHandler(void)		
{
unsigned char lState0;
	lState0 = (lState0<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	lState0 = (lState0<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	lState0 = lState0&0x03;
	switch (lState0) {
		case 0:
			if(lPreviousState==1) lCountValue++;
			else lCountValue--;
		break;
		case 1:
			if(lPreviousState==3) lCountValue++;
			else lCountValue--;
		break;
		case 2:
			if(lPreviousState==0) lCountValue++;
			else lCountValue--;
		break;
		case 3:
			if(lPreviousState==2) lCountValue++;
			else lCountValue--;
		break;
		}
	lPreviousState = lState0;
	lCntVel++;
	if (lCountValue>=48) {
		lCountValue = 0;
		lPosCnt++;
	}
	else if	(lCountValue<=-48) {
		lCountValue = 0;
		lPosCnt--;
	}
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}


void EXTI1_IRQHandler(void)	
{
unsigned char lState1;
	lState1 = (lState1<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	lState1 = (lState1<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	lState1 = lState1&0x03;
	switch (lState1) {
		case 0:
			if(lPreviousState==1) lCountValue++;
			else lCountValue--;
		break;
		case 1:
			if(lPreviousState==3) lCountValue++;
			else lCountValue--;
		break;
		case 2:
			if(lPreviousState==0) lCountValue++;
			else lCountValue--;
		break;
		case 3:
			if(lPreviousState==2) lCountValue++;
			else lCountValue--;
		break;
		}
	lPreviousState = lState1;
	lCntVel++;
	if (lCountValue>=48) {
		lCountValue = 0;
		lPosCnt++;
	}
	else if	(lCountValue<=-48 ){
		lCountValue = 0;
		lPosCnt--;
	}
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}
/******************************************************************************/



/**************************************function PID for right wheel*********************/ 
void rPIDvel(float rDesiredVel,float rVel){
	float rKp=0.0717;
	float rKd=0;
	float rKi=4.48;
	float rsampletime = 0.02;
	float rHILIM=100;
	float rLOLIM=0;
	int rpid_sat;
	static float rerr_p=0;
	static float rui_p=0;
	static float rud_f_p=0;
	float rerr,rup,rud,rui,rud_f;
	static float rerr_sat;
	float ruout;
	
	rerr=rDesiredVel-rVel;
	rup=rKp*rerr;
	rud=rKd*(rerr-rerr_p)/rsampletime;
	rud_f=0.1*rud+(1-0.1)*rud_f;
	rui=rui_p+rKi*rerr*rsampletime;
	
	rerr_p=rerr;
	rui_p=rui;
	rud_f_p=rud_f;
	
	ruout =(int)(rup+rud+rui);
	if(ruout>rHILIM)
		rpid_sat=rHILIM;
	else if (ruout<rLOLIM)
		rpid_sat=rLOLIM;
	else rpid_sat=ruout;
	
	if(rpid_sat>0)
	{
	
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);
	
		TIM2->CCR1=rpid_sat;
	}
	else{TIM2->CCR1=0;}
}


/**************************************function PID for left wheel*********************/ 
void lPIDvel(float lDesiredVel,float lVel){
	float lKp=0.0717;
	float lKd=0;
	float lKi=4.48;
	float lsampletime = 0.02;
	float lHILIM=100;
	float lLOLIM=0;
	int lpid_sat;
	static float lerr_p=0;
	static float lui_p=0;
	static float lud_f_p=0;
	float lerr,lup,lud,lui,lud_f;
	static float lerr_sat;
	float luout;
	
	lerr=lDesiredVel-lVel;
	lup=lKp*lerr;
	lud=lKd*(lerr-lerr_p)/lsampletime;
	lud_f=0.1*lud+(1-0.1)*lud_f;
	lui=lui_p+lKi*lerr*lsampletime;
	
	lerr_p=lerr;
	lui_p=lui;
	lud_f_p=lud_f;
	
	luout =(int)(lup+lud+lui);
	if(luout>lHILIM)
		lpid_sat=lHILIM;
	else if (luout<lLOLIM)
		lpid_sat=lLOLIM;
	else lpid_sat=luout;
	
	if(lpid_sat>0)
	{
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
		
		TIM2->CCR2=lpid_sat;
	}
	else{TIM2->CCR2=0;}

}



/*************************Run PID*****************************************/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
			
        rCnttmp = rCntVel;
        rCntVel = 0;
        rRealVel = rCnttmp * 250; // RPM
        rCurVel = rCnttmp * 4.444444444 * pi;
        rGearVel = rRealVel / 19.2;

        lCnttmp = lCntVel;
        lCntVel = 0;
        lRealVel = lCnttmp * 250; // RPM
        lCurVel = lCnttmp * 4.444444444 * pi;
        lGearVel = lRealVel / 19.2;
				follow_line();
			
        switch(sensor){
					//00100//
					case 4:
						leftspeed=20;
						rightspeed=20;
					break;
					//00001//
					case 1:
						leftspeed=8;
						rightspeed=20;
					break;
					//00011//
					case 3:
						leftspeed=12;
						rightspeed=20;
					break;
					//00010//
					case 2:
						leftspeed=16;
						rightspeed=20;
					break; 
					//00110//
					case 6:
						leftspeed=18;
						rightspeed=20;
					break;
					//01100//	
					case 12:
						leftspeed=20;
						rightspeed=18;
					break;
					//01000//
					case 8:
						leftspeed=20;
						rightspeed=16;
					break;
					//10000//
					case 16:
						leftspeed=20;
						rightspeed=8;
					break;
					//11000//
					case 24:
						leftspeed=20;
						rightspeed=12;
					break;
					
					
			}

	   if (pidEnabled) {
 		   rPIDvel(rightspeed, rGearVel); 
			 lPIDvel(leftspeed, lGearVel);
        
		 }
   }
}


  
int main(void)
{
  
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)data, 5);
  
	
  while (1)
  {
		
	      if (sensor == 31 && previousSensorValue != 31)
        {
            count++;
        }
        previousSensorValue = sensor; // Update the previous sensor value
				if(count==2){
					pidEnabled = 0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
					TIM2->CCR1=0;
					TIM2->CCR2=0;
					HAL_Delay(2000);
					TIM2->CCR1=0;
					TIM2->CCR2=turnleft;
					HAL_Delay(2000);
					pidEnabled = 1;
					count=3;
				}
			 if (count==4){
				 if(battery>=charge){
			 
					pidEnabled = 0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
					TIM2->CCR1=0;
					TIM2->CCR2=0;
					HAL_Delay(2000);
					TIM2->CCR1=0;
					TIM2->CCR2=turnleft;
					HAL_Delay(2000);
					pidEnabled = 1;
					count=5;}
				 else count=9;
				 }
						
		 		 if (count==6){
						 
					pidEnabled = 0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
					TIM2->CCR1=0;
					TIM2->CCR2=0;
					HAL_Delay(2000);	  
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
					TIM2->CCR1=back;
					TIM2->CCR2=back;
					HAL_Delay(1000);
					TIM2->CCR1=0;
					TIM2->CCR2=0;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
					HAL_Delay(2000);
					TIM2->CCR1=back;
					TIM2->CCR2=back;
					HAL_Delay(3500);
					pidEnabled = 1;
					count=7;
				 }
						 
					if (count==8){
				 	pidEnabled = 0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
					TIM2->CCR1=0;
					TIM2->CCR2=0;
					HAL_Delay(2000);
					TIM2->CCR1=0;
					TIM2->CCR2=turnleft;
					HAL_Delay(2000);
					pidEnabled = 1;
					count=9;					
					}
						
					if (count==10){
					if(battery>=charge){
					pidEnabled = 0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
					TIM2->CCR1=0;
					TIM2->CCR2=0;
					HAL_Delay(2000);
					TIM2->CCR1=0;
					TIM2->CCR2=turnleft;
					HAL_Delay(2000);
					pidEnabled = 1;
					count=11;}
					else count=15;
					}
						
		 			 if (count==12){
						
					pidEnabled = 0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
					TIM2->CCR1=0;
					TIM2->CCR2=0;
					HAL_Delay(2000);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
					TIM2->CCR1=back;
					TIM2->CCR2=back;
					HAL_Delay(1000);
					TIM2->CCR1=0;
					TIM2->CCR2=0;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
					HAL_Delay(2000);
					TIM2->CCR1=back;
					TIM2->CCR2=back;
					HAL_Delay(3500);
					pidEnabled = 1;
					count=13;}
						
					}
					if (count==14){
							
					pidEnabled = 0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
					TIM2->CCR1=0;
					TIM2->CCR2=0;
					HAL_Delay(2000);
					TIM2->CCR1=0;
					TIM2->CCR2=turnleft;
					HAL_Delay(2000);
					pidEnabled = 1;
					count=15;
							
					}
					if (count==16){
					if(battery<charge){
					pidEnabled = 0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
					TIM2->CCR1=0;
					TIM2->CCR2=0;
					HAL_Delay(2000);
					TIM2->CCR1=back;
					TIM2->CCR2=back;
					HAL_Delay(1000);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
					TIM2->CCR1=back;
					TIM2->CCR2=back;
					HAL_Delay(1600);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
		      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
					TIM2->CCR1=back;
					TIM2->CCR2=back;
					HAL_Delay(6000);
					count=17;
					pidEnabled=1;}
					else count =17;
					}
					if(count>=17&&battery>=charge){
						count=3;
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 23;
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
  htim2.Init.Prescaler = 11;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
