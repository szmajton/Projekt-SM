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
#include "BMPXX80.h"
#include "lcd16x2.h"
#include "math.h"
#include "bmp280.h"
#include "bmp280_defs.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <arm_math.h>


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
float target_temp;
float current_temp;
uint16_t duty = 0;
_Bool czy_ustawiono = false;
arm_pid_instance_f32 PID;
/* Zmienne do komunikacji z UART */
int flag = 0;
char data[4];
uint8_t want_uart;
uint8_t msg[3];
uint16_t Sizemsg = 3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Odczytywanie danych z czujnika */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr , uint8_t *reg_data , uint16_t length)
{
  HAL_StatusTypeDef status = HAL_OK;
  int32_t iError = BMP280_OK;
  uint8_t txarray[SPI_BUFFER_LEN] = {0,};
  uint8_t rxarray[SPI_BUFFER_LEN] = {0,};
  uint8_t stringpos;
 txarray[0] = reg_addr;
 HAL_GPIO_WritePin( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_RESET );
 status = HAL_SPI_TransmitReceive( &hspi4 , (uint8_t*)(&txarray), (uint8_t*)(&rxarray), length+1, 5);
 while( hspi4.State == HAL_SPI_STATE_BUSY ) {};
 HAL_GPIO_WritePin( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_SET );
 for (stringpos = 0; stringpos < length; stringpos++)
 {
	 *(reg_data + stringpos) = rxarray[stringpos + BMP280_DATA_INDEX];
 }

}
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr , uint8_t *reg_data , uint16_t length)
{
HAL_StatusTypeDef status = HAL_OK;
int32_t iError = BMP280_OK;
uint8_t txarray[SPI_BUFFER_LEN * BMP280_ADDRESS_INDEX];
uint8_t stringpos;
txarray[0] = reg_addr;
for (stringpos = 0; stringpos < length; stringpos++)
{
	txarray[stringpos+BMP280_DATA_INDEX] = reg_data[stringpos];
}
 HAL_GPIO_WritePin( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_RESET );
 status = HAL_SPI_Transmit( &hspi4 , (uint8_t*)(&txarray), length*2, 100);
 while( hspi4.State == HAL_SPI_STATE_BUSY ) {};
 HAL_GPIO_WritePin( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_SET );
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
	/* Inicjalizacja parametrów czujnika i  zmienne do odczytania temperatury */
	struct bmp280_uncomp_data ucomp_data;
	struct bmp280_dev bmp;
	struct bmp280_config conf;
	int8_t rslt;
	double temp;
	char buffer[40];
	uint8_t size;

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
  HAL_UART_Receive_IT(&huart3, msg, Sizemsg);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); /* Sygnał PWM */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); /*Enkoder */
  /* Inicjalizacja danych czujnika BMP280 */
  bmp.delay_ms = HAL_Delay;
  bmp.dev_id = 0;
  bmp.intf = BMP280_SPI_INTF;
  bmp.read = spi_reg_read;
  bmp.write = spi_reg_write;
  rslt = bmp280_init(&bmp);
  rslt = bmp280_get_config(&conf, &bmp);
  conf.filter = BMP280_FILTER_COEFF_2;
  conf.os_temp = BMP280_OS_4X;
  conf.odr = BMP280_ODR_1000_MS;
  rslt = bmp280_set_config(&conf, &bmp);
  rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
	 rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
	 current_temp = temp;
	/*Jeśli nie ustalono jeszcze temperatury */
	 if(czy_ustawiono == false)
	 {
		 target_temp = current_temp;
	  }
	 if (duty > 100) {
	     duty = 100;
	  } else if (duty < -100) {
	     duty = -100;
	  }
	 /* Sending temperature to terminal */
	  size = sprintf(buffer, "Temp:  %f [C]\n\r", current_temp);
	  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 200);
	  if(target_temp > current_temp){
	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 100*duty);
	     }
	  bmp.delay_ms(1000);
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
_Bool niepoprawne;
/* Wysylanie danych do UART i sprawdzanie czy wartosc jest poprawna */
HAL_UART_Receive_IT(&huart3, msg, Sizemsg);
if(msg[0]>47 && msg[0]<58){
	int dziesiatki = msg[0] - '0';
	if(msg[1]>47 && msg[1]<58){
		int jednosci = msg[1] - '0';
		if(msg[2]>47 && msg[2]<58){
			int dziesietne = msg[2] - '0';
			niepoprawne = false;
			target_temp = 10.0 * dziesiatki + jednosci + dziesietne / 10.0;
			czy_ustawiono = true;
	}
  }
}

/* Jesli jest zly format wysylanej wartosci */
if(niepoprawne == true){
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
