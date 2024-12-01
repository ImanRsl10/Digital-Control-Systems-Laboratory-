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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 TIM11 -> PSC = 2 (42 / 3 = 14M), ARR = 999 (14 M/K = 14K)
 CCR = 9990 to have 0.1% D.C
 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define A0 800
#define A1 1400
#define A2 2000
#define MAX_U 12.0f
#define MAX_DUTY 1000
#define F_APB 84000000
#define Kp 0.01
#define KI 0.08
#define Ts 0.01
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
uint32_t counter = 0, tau_counter = 0;
int16_t pos_counter = 0;
uint32_t direction = 0;
uint32_t deadBand_CRR = 0, CCR1 = 0, CCR2 = 0;
int32_t velocity = 0;
uint32_t v1 = 0, v2 = 0, prev_v, curr_v = 0;
uint32_t K_num = 0, tau = 0;
uint32_t t1 = 0, t2 = 0, th = 0; //Computing tau
int16_t angle = 0; //Angle
float distance = 0.0f;
uint32_t target_velocities[5] = {A0, A1, A2, A1, A0};
uint32_t target_v_counter = 0, tagert_index_counter = 0;
uint16_t set_point = A0;
uint16_t set_position = 270;
float u = 0.0f, prev_u1 = 0.0f, prev_u2 = 0.0f;
float velocity_error = 0.0f, prev_v_error = 0.0f;
float position_error = 0.0f, prev_pos_error1 = 0.0f, prev_pos_error2 = 0.0f;
const uint16_t PPR = 1024;
const float a = 0.5067f, b = -0.4677f, c = 0.7713f, d = 0.3805f;
float K = 0.05f;
// TIM11 for PWM & TIM6 for A (Encoder)
// ARR_11 = 999, CCR_11 = 0
// ARR_6 = 9999 CCR_6 = 15
// 2 GPIO inputs for A & B (A is EXTI)
// 1 GPIO EXTI for pushBotton
typedef enum
{
	IDLE = 0, WAIT_Z = 1, COMPUTE_ANGLE = 2, CONTROL_VELOCITY = 3, WAIT = 4, CONTROL_ANGLE = 5
} lock_states_t;

lock_states_t state = IDLE;
// DEAD_BAND = 1, WAIT_0 = 2, K = 3, WAIT_1 = 4, Compute_K = 5, WAIT_TAU = 6, Compute_tau = 7,
typedef enum
{
	OK = 0, NOT_READY = 1
} result_t;
//functions:
result_t Compute_deadband(void);
void Compute_v1(void);
void add_CRR(void);
void calc_K(void);
void config_timer(void);
result_t compute_tau(void);
float get_velocity(void);
float get_angle(void);
uint8_t getDir(void);
//Control functions
void setDir(uint8_t dir);
void setDuty(uint16_t duty);
void turnLED(uint8_t led, uint8_t status);
void setTs(uint8_t Tss);
void Vout2PWM(float u);
void control_velocity();
void control_position();
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
  MX_TIM6_Init();
  MX_TIM11_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
//  setDuty(50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  setTs(Ts * 1000);
  while (1)
  {
	  //*Board Test*//
//	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
//	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
//	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(state)
	  {
	  case IDLE:
		  TIM11->CCR1 = 300;
		  HAL_Delay(2000);
		  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
		  state = WAIT_Z;
		  break;
	  case WAIT_Z:
		  break;
	  case COMPUTE_ANGLE:
		  break;
	  case CONTROL_VELOCITY:
		  state = CONTROL_VELOCITY;
		  break;
	  case WAIT:
		  TIM11->CCR1 = 0;
		  HAL_Delay(2000);
		  state = CONTROL_ANGLE;
		  break;
	  case CONTROL_ANGLE:
		  state = CONTROL_ANGLE;
		  break;
	  }
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Encoder: A: PE5, B: PF6, Z: PA0
	if(GPIO_Pin == GPIO_PIN_5)
	{
		counter++;
//		pos_counter++;
		direction = getDir();
		if(direction == 0)
		{
			pos_counter++;
			if(pos_counter > 1024)
				pos_counter = 0;
		}
		else
		{
			pos_counter--;
			if(pos_counter < 0)
				pos_counter = 1024;
		}
		angle = get_angle();
	}
	if(GPIO_Pin == GPIO_PIN_0)
	{
		if(state == WAIT_Z)
		{
			pos_counter = 0;
			state = COMPUTE_ANGLE;
		}
	}
	if(GPIO_Pin == GPIO_PIN_2)
	{
		if(state == COMPUTE_ANGLE)
		{
			state = CONTROL_VELOCITY;
			HAL_TIM_Base_Start_IT(&htim7);
		}
	}
	if(GPIO_Pin == GPIO_PIN_3)
	{
		if(state == CONTROL_VELOCITY)
		{
			state = WAIT;
//			HAL_TIM_Base_Start_IT(&htim7);
		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		target_v_counter++;
		if(target_v_counter > 1000)
		{
			target_v_counter = 0;
			tagert_index_counter++;
			if(tagert_index_counter >= 5)
			{
				tagert_index_counter = 0;
			}
			set_point = target_velocities[tagert_index_counter];
		}
		velocity = get_velocity();
		counter = 0;
	}
	else if(htim->Instance == TIM7)
	{
		if(state == CONTROL_VELOCITY)
		{
			velocity_error = set_point - velocity;

			control_velocity();

			prev_v_error = velocity_error;

//			prev_u2 = prev_u1;
			prev_u1 = u;

			Vout2PWM(u);
		}
		else if(state == CONTROL_ANGLE)
		{
			position_error = set_position - angle;
			if(position_error > 0.001 || position_error < -0.001)
				control_position();

			prev_pos_error1 = position_error;

			prev_u1 = u;

			Vout2PWM(u);
		}
//		if(state == DEAD_BAND)
//			TIM11->CCR1 += 100;
//		else if(state == WAIT_1)
//		{
//			prev_v = curr_v;
//			curr_v = velocity;
//			if(curr_v - prev_v < 30)
//			{
//				v2 = curr_v;
//				state = Compute_K;
//			}
//		}
	}
}

result_t Compute_deadband(void)
{
	if(velocity > 0)
	{
		deadBand_CRR = TIM11->CCR1; //300
		TIM11->CCR1 = 0;
		return OK;
	}
	else
	{
		return NOT_READY;
	}
}

void Compute_v1(void)
{
	TIM11->CCR1 = deadBand_CRR + 100;
	CCR1 = TIM11->CCR1;
	HAL_Delay(1000);
	v1 = velocity;
	TIM11->CCR1 = CCR1 * 2; //Like Step function
	CCR2 = TIM11->CCR1;
}

void calc_K()
{
	K_num = ((v2 - v1)*TIM11->ARR)/((CCR2 - CCR1) * 12);
	// K_num is 326
}

void config_timer()
{
	TIM11->CCR1 = CCR1;
	HAL_Delay(2000);
	TIM11->CCR1 = CCR1 * 2; //Like Step function
	CCR2 = TIM11->CCR1;
	t1 = 0;
}

result_t compute_tau(void)
{
	th = v1 + (63*(v2 - v1)/100);
	if(velocity >= th)
	{
		t2 = tau_counter;
		tau = t2 - t1;
		return OK;
	}
	else
	{
		return NOT_READY;
	}
}

float get_velocity(void)
{
	uint32_t round = 0;
	float v = 0.0;
	round = counter * 60 * 1000;
	v = round / 1024;
//	counter = 0;
	return v;
}

float get_angle(void)
{
	distance = ((pos_counter % 1024) * 360) / 1024;
	return distance;
}

uint8_t getDir(void)
{
	return HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6);
}

void setDir(uint8_t dir)
{
	// write to PE6
	// dir = 0 CW dir = 1 CCW
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, dir);
}

void setDuty(uint16_t duty)
{
	// DC%
	TIM11->CCR1 = (duty * TIM11->ARR)/MAX_DUTY;
}

void turnLED(uint8_t led , uint8_t status)
{
	if(led == 0)
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, status);
	else
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, status);
}

void setTs(uint8_t Tss)
{
	float fs = 1/Tss;
	TIM7->ARR = (F_APB / ((TIM7->PSC + 1) * fs)) - 1;
}

void Vout2PWM(float u)
{
	float duty = 0.0f;
	uint8_t dir;
	if(u >= 0.0f)
	{
		duty = u / MAX_U;
		dir = 1;
	}
	else
	{
		duty = (-u) / MAX_U;
		dir = 0;
	}
	TIM11->CCR1 = (uint32_t) (duty * TIM11->ARR);
	setDir(dir);
//	uint8_t dir;
//	float v = u;
//	if(u < 0)
//	{
//		dir = 1;
//		v = -u;
//	}
//	else
//	{
//		v = u;
//		dir = 0;
//	}
//	setDir(dir);
//	v = (v * MAX_DUTY) / MAX_U;
//	TIM11->CCR1 = v;
//	setDuty(v);
}

void control_velocity()
{
	u = prev_u1 + velocity_error * (Kp + KI * Ts) - prev_v_error * Kp;
	if(u > MAX_U)
		u = MAX_U;
	else if(u < -MAX_U)
		u = -MAX_U;
//	prev_u1 = u;
}

void control_position()
{
	u = (b * prev_pos_error1 + a * position_error - d * prev_u1) / c;
//	u = K * position_error;
//	if(u < 3.6 && u > 0.1)
//		u = 3.6;
//	if(u > -3.6 && u < -0.1)
//		u = -3.6;
//	if(u > MAX_U)
//		u = MAX_U;
//	else if(u < -MAX_U)
//		u = -MAX_U;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of velocity_error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL velocity_error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param velocity_error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param velocity_error line source number
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
