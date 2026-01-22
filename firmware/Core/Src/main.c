/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    GPIO_TypeDef* trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef* echo_port;
    uint16_t      echo_pin;
} HCSR04_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RIGHT_STOP 	150
#define LEFT_STOP	149
#define SERVO_CW 	130
#define SERVO_CCW 	170

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

uint32_t hcsr04_measure_cm(HCSR04_t* s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HCSR04_t sensors[4] = {
//TRIG_PORT, TRIG_PIN, ECHO_PORT, ECHO_PIN
    {GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1}, // left sensor 1
    {GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5}, // right sensor 2
    {GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7}, // front sensor 3
    {GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1}  // back sensor 4
};
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // right motor
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);// left motor
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  	    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  	DWT->CYCCNT = 0;
  	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  printf("UART debug ready!\r\n");

  // stop
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RIGHT_STOP);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LEFT_STOP);
  HAL_Delay(1000);

  // go forward
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_CW);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_CCW);
  HAL_Delay(2000);

  // stop
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RIGHT_STOP);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LEFT_STOP);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  for (int i = 0; i < 4; i++)
	      {
	          uint32_t dist = hcsr04_measure_cm(&sensors[i]);
	          printf("Sensor %d: %lu cm\r\n", i + 1, dist);
	          HAL_Delay(60);
	      }

	      printf("----\r\n");
	      HAL_Delay(300);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}


uint32_t hcsr04_measure_cm(HCSR04_t* s)
{
    uint32_t start_tick, end_tick;
    uint32_t timeout;

    // TRIG 10 мкс
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    DWT->CYCCNT = 0;
    while (DWT->CYCCNT < (SystemCoreClock / 1000000) * 10);
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

    // Ждём фронт (max ~25 ms)
    timeout = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET)
    {
        if ((DWT->CYCCNT - timeout) > (SystemCoreClock / 40)) // ~25 ms
            return 0;
    }

    start_tick = DWT->CYCCNT;

    // Ждём спад
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET)
    {
        if ((DWT->CYCCNT - start_tick) > (SystemCoreClock / 40))
            return 0;
    }

    end_tick = DWT->CYCCNT;

    uint32_t pulse_us =
        (end_tick - start_tick) / (SystemCoreClock / 1000000);

    return pulse_us * 34 / 2000;
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
