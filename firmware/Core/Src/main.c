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
#include <stdlib.h>
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

#define LEFT_FW 170 // SERVO_CCW
#define LEFT_FW_SLOW 155
#define LEFT_STOP	149
#define LEFT_BW_SLOW 145
#define LEFT_BW 130 // SERVO_CW

#define RIGHT_FW 130
#define RIGHT_FW_SLOW 145
#define RIGHT_STOP 	150
#define RIGHT_BW_SLOW 155
#define RIGHT_BW 170

#define LED_PORT GPIOA
#define LED_PIN_R GPIO_PIN_8
#define LED_PIN_G GPIO_PIN_9
#define LED_PIN_B GPIO_PIN_10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum  {
	CAR_WANDER_FW, CAR_WANDER_TURN, CAR_TURNING_LEFT, CAR_TURNING_RIGHT, CAR_ESCAPE
} car_state;

typedef enum {
	ENV_NONE, ENV_OBSTACLE_FRONT, ENV_OBSTACLE_LEFT, ENV_OBSTACLE_RIGHT, ENV_OBSTACLE_BACK
} env_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t hcsr04_measure_cm(HCSR04_t* s);
void motors_stop();
void motors_turn_left();
void motors_turn_right();
void motors_forward_fast();
void motors_forward_slow();
void set_led_rgb(uint8_t r, uint8_t g, uint8_t b);
env_state classify_env(uint32_t *dist);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HCSR04_t sensors[4] = {
//TRIG_PORT, TRIG_PIN, ECHO_PORT, ECHO_PIN
    {GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1}, // Right sensor 1
    {GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5}, // Left sensor 2
    {GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7}, // Front sensor 3
    {GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1}  // Back sensor 4
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
  // random seed for wandering behaviour
  srand(HAL_GetTick());
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
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RIGHT_FW);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LEFT_FW);
  HAL_Delay(2000);

  // stop
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RIGHT_STOP);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LEFT_STOP);

  // Initialize states
  car_state c_state = CAR_WANDER_FW;
  env_state env = ENV_NONE;

  uint32_t last_wander_tick = HAL_GetTick();
  uint32_t wander_interval = 2000; // 2s
  int wander_direction = 0;        // 0 = Left, 1 = Right
  int obstacle_confidence = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	uint32_t dist[4];

	// measure distance from sensors
	for (int i = 0; i < 4; i++)
	{
	  dist[i] = hcsr04_measure_cm(&sensors[i]);
	  printf("Sensor %d: %lu cm\r\n", i + 1, dist[i]); // log to serial port
	  HAL_Delay(5);
	}
	printf("----\r\n");
	//HAL_Delay(100);

	// assign environment state based on measurements
	env_state detected_env = classify_env(dist);
	if (detected_env != ENV_NONE) {
	    obstacle_confidence++;
	} else {
	    obstacle_confidence = 0; // Reset if the path is clear
	}

	if (obstacle_confidence >= 5) { // Only change state if seen 3 times
	    env = detected_env;
	} else {
	    env = ENV_NONE;
	}

	// movement behaviour
	switch (env){
		case ENV_NONE:
			if(c_state == CAR_ESCAPE)
			{
				c_state = CAR_WANDER_FW;
				last_wander_tick = HAL_GetTick();
			}
			if (c_state == CAR_WANDER_FW || c_state == CAR_WANDER_TURN)
			{
				if ((HAL_GetTick() - last_wander_tick) > wander_interval)
				{
					last_wander_tick = HAL_GetTick();

					if (c_state == CAR_WANDER_FW) {
						// Switch to Turning
						c_state = CAR_WANDER_TURN;
						wander_interval = 300 + (rand() % 500);
						wander_direction = rand() % 2;
					}
					else {
						// Switch back to Forward
						c_state = CAR_WANDER_FW;
						wander_interval = 1000 + (rand() % 2000);
					}
				}
			}
			break;
		case ENV_OBSTACLE_FRONT:
			c_state = CAR_TURNING_LEFT;
			break;
		case ENV_OBSTACLE_LEFT:
			c_state = CAR_TURNING_RIGHT;
			break;
		case ENV_OBSTACLE_RIGHT:
			c_state = CAR_TURNING_LEFT;
			break;
		case ENV_OBSTACLE_BACK:
			c_state = CAR_ESCAPE;
			break;
		default:
			motors_stop();
			break;
	}
	// control motors based on the envirnment
	switch (c_state){
		case CAR_TURNING_LEFT:
			motors_turn_left();
			set_led_rgb(1, 1, 0); // yellow for turning
			break;
		case CAR_TURNING_RIGHT:
			motors_turn_right();
			set_led_rgb(1, 1, 0); // yellow
			break;
		case CAR_WANDER_FW:
			motors_forward_slow();
			set_led_rgb(0, 1, 0); // green for wandering
			break;
		case CAR_WANDER_TURN:
			if(wander_direction == 0)
				motors_turn_left();
			else
				motors_turn_right();
			set_led_rgb(0, 0, 1); // blue for turning while wandering
			break;
		case CAR_ESCAPE:
			motors_forward_fast();
			set_led_rgb(1, 0, 0); // red for escaping from obstacle
			break;
		default:
			motors_stop();
			set_led_rgb(0, 0, 0); // off
			break;
	}


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
            return 99;
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

void motors_stop()
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RIGHT_STOP); // Right wheel stop
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LEFT_STOP); // Left wheel stop
}

void motors_turn_left()
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RIGHT_FW_SLOW); // Right wheel forward
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LEFT_BW_SLOW); // Left wheel backwards
}

void motors_turn_right()
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RIGHT_BW_SLOW); // Right wheel backwards
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LEFT_FW_SLOW); // Left wheel forward
}

void motors_forward_fast()
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RIGHT_FW); // Right wheel forward
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LEFT_FW); // Left wheel forward
}

void motors_forward_slow()
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RIGHT_FW_SLOW); // Right wheel slow forward
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LEFT_FW_SLOW); // Left wheel slow forward
}

env_state classify_env(uint32_t *dist)
{
    if (dist[2] < 15) return ENV_OBSTACLE_FRONT;
    if (dist[3] < 35) return ENV_OBSTACLE_BACK;
    if (dist[0] < 15) return ENV_OBSTACLE_RIGHT;
    if (dist[1] < 15) return ENV_OBSTACLE_LEFT;

    return ENV_NONE;
}

void set_led_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    // 1 = ON, 0 = OFF
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_R, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_G, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_B, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
