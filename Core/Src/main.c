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
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Motor_PID{ 
  const double P, I, D;
	double PWM, Last_Error, Previous_Error;
} Motor_PID;

void Incremental_PID (Motor_PID *motor_pid, const double *Encoder, const double *Target){
		double iError = *Target - *Encoder;
		motor_pid->PWM +=  (motor_pid->P * (iError - motor_pid->Last_Error)
										  + motor_pid->I * iError
										  + motor_pid->D * (iError - 2 * motor_pid->Last_Error + motor_pid->Previous_Error));
		motor_pid->Previous_Error = motor_pid->Last_Error;
		motor_pid->Last_Error = iError;	           
}

void limited_Pwm(Motor_PID *motor_pid){	
    if(motor_pid->PWM<-7200) motor_pid->PWM=-7200;	
		if(motor_pid->PWM>7200)  motor_pid->PWM=7200;
}

void Set_Pwm(const Motor_PID *motor_A, const Motor_PID *motor_B){
    	if(motor_A->PWM<0){
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 7200);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 7200+motor_A->PWM);
			}
			else{
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 7200);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 7200-motor_A->PWM);
			}
		
		  if(motor_B->PWM<0){
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 7200);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 7200+motor_B->PWM);
			}
			else{
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 7200);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 7200-motor_B->PWM);
			}
}

void oled_show(double current_rps[2], double target_rps[2], uint32_t *Voltage, bool flag){
													OLED_ShowString(00,20,"LT");
		if(target_rps[0]<0)	  OLED_ShowString(20,20,"-"),
		                      OLED_ShowNumber(35,20,-target_rps[0],3,12);
		else               		OLED_ShowString(20,20,"+"),
		                      OLED_ShowNumber(35,20,target_rps[0],3,12);	

		                      OLED_ShowString(60,20,"LV");
		if(current_rps[0]<0)	OLED_ShowString(80,20,"-"),
		                      OLED_ShowNumber(95,20,-current_rps[0],3,12);
		else               		OLED_ShowString(80,20,"+"),
		                      OLED_ShowNumber(95,20,current_rps[0],3,12);	

		
				                  OLED_ShowString(00,30,"RT");
		if(target_rps[1]<0)	  OLED_ShowString(20,30,"-"),
		                      OLED_ShowNumber(35,30,-target_rps[1],3,12);
		else               		OLED_ShowString(20,30,"+"),
		                      OLED_ShowNumber(35,30,target_rps[1],3,12);	

		                      OLED_ShowString(60,30,"RV");
		if(current_rps[1]<0)	OLED_ShowString(80,30,"-"),
		                      OLED_ShowNumber(95,30,-current_rps[1],3,12);
		else               		OLED_ShowString(80,30,"+"),
		                      OLED_ShowNumber(95,30,current_rps[1],3,12);	


		                      OLED_ShowString(00,40,"VOLTAGE");
													OLED_ShowString(78,40,".");
													OLED_ShowString(100,40,"V");
													OLED_ShowNumber(65,40,*Voltage/100,2,12);
													OLED_ShowNumber(88,40,*Voltage%100,2,12);
		if(*Voltage%100<10) 	  OLED_ShowNumber(82,40,0,2,12);
		                    
													OLED_ShowString(00,50,"Motor");
		if(flag)							OLED_ShowString(50,50,"O-N");
		else									OLED_ShowString(50,50,"OFF");
	
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t tim1_count=0;
static uint16_t adcData=0;
static uint32_t Voltage=0;
static bool motor_flag=true, system_flag=true;
static double target_rps[2] = {0,0}, current_rps[2]={0,0};
static Motor_PID Motor_A = {.P=46,.I=46*10,.D=46/10,.PWM=0,.Last_Error=0,.Previous_Error=0};
static Motor_PID Motor_B = {.P=46,.I=46*10,.D=46/10,.PWM=0,.Last_Error=0,.Previous_Error=0};
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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_Delay(100);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcData,1);
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)&target_rps, 16);
	HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for(;;)
  {
    /* USER CODE END WHILE */
		current_rps[0] =	(short)__HAL_TIM_GET_COUNTER(&htim2);
		current_rps[1] =	(short)__HAL_TIM_GET_COUNTER(&htim3);
		double Tmp = tim1_count*65536+ __HAL_TIM_GetCounter(&htim1);
		__HAL_TIM_SetCounter(&htim1,0);
		tim1_count = 0;
		Tmp = 72000000 / Tmp;
		current_rps[0] = current_rps[0] * Tmp / 1560;
		current_rps[1] = -current_rps[1]  * Tmp / 1560;
    __HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&current_rps, 16);
		if(motor_flag && system_flag){
			Incremental_PID (&Motor_A, &current_rps[0], &target_rps[0]);
			Incremental_PID (&Motor_B, &current_rps[1], &target_rps[1]);
			limited_Pwm(&Motor_A);limited_Pwm(&Motor_B);
			Set_Pwm(&Motor_A, &Motor_B);
		} else {
			Set_Pwm(&Motor_A, &Motor_B);
		}
		oled_show(current_rps, target_rps, &Voltage, motor_flag && system_flag);
		OLED_Refresh_Gram();
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&htim1))
  {
		++tim1_count;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart1){
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)&target_rps, 16);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	Voltage = adcData*3.3*11*100/4096;
	motor_flag = Voltage > 1110;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcData,1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	HAL_Delay(100);
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) {
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
			Motor_A.PWM=0;
			Motor_B.PWM=0;
			system_flag = !system_flag;
	}
}
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
