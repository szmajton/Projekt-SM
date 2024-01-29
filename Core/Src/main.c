/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "eth.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "lcd16x2.h"
#include "math.h"

#include "bmp2.h"
#include "bmp2_config.h"
#include "bmp2_defs.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <arm_math.h>
#include "pid_controller_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_BUFFER_LEN 28
#define BMP280_DATA_INDEX 1
#define BMP280_ADDRESS_INDEX 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int temp_current_int;
int temp_frac;
int temp_target_int;
int temp_target_frac;
float temp_current, uchyb, target_temp, pid_o;
char buffer[128];
float duty;
#define RX_BUFFER_SIZE 128
char rxbuffer[RX_BUFFER_SIZE];
char ostrxbuffer[RX_BUFFER_SIZE];
int flag =0;
static uint32_t p_env;
uint8_t msg[3];
uint16_t Sizemsg = 3;
int czy_ustawiono =0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Odczytywanie danych z czujnika */
void set_pwm(TIM_HandleTypeDef *htim, float pow)
{
	if(pow==0){
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,0);
	}
	uint32_t cp = htim->Init.Period;
	uint32_t pv = (cp*pow)/100.f;
	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,(uint32_t)pv);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim3)
	{
		temp_current = BMP2_ReadTemperature_degC(&bmp2dev_1);
		temp_current_int = (int)temp_current;
		temp_frac = (int)((temp_current - temp_current_int)*1000);
		if(flag ==1){
			flag=0;
		}
		duty = PID_GetOutput(&hpid1, target_temp, temp_current);
		set_pwm(&htim3, duty);
		int size = sprintf(buffer, "Temp:  %f [C]\n\r", temp_current);
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 200);
	}
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI4_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Inicjalizacja danych czujnika BMP280 */
  BMP2_Init(&bmp2dev_1);

  HAL_UART_Receive_IT(&huart3, msg, Sizemsg);
 // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); /* Sygnał PWM */
 // HAL_TIM_Base_Start_IT(&htim1);
 // HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); /*Enkoder */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  PID_Init(&hpid1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); /* Sygnał PWM */
  HAL_TIM_Base_Start_IT(&htim3);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
int niepoprawne;
/* Wysylanie danych do UART i sprawdzanie czy wartosc jest poprawna */
HAL_UART_Receive_IT(&huart3, msg, Sizemsg);
if(msg[0]>47 && msg[0]<58){
	int dziesiatki = msg[0] - '0';
	if(msg[1]>47 && msg[1]<58){
		int jednosci = msg[1] - '0';
		if(msg[2]>47 && msg[2]<58){
			int dziesietne = msg[2] - '0';
			niepoprawne = 0;
			target_temp = 10.0 * dziesiatki + jednosci + dziesietne / 10.0;
			czy_ustawiono = 1;
	}
  }
}

/* Jesli jest zly format wysylanej wartosci */
if(niepoprawne == 1){
	char buf[20];
	uint8_t err_msg = sprintf(buf, "Zly format\n\r");
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, err_msg, 100);
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
	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  HAL_Delay(100);
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
