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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "hbridge.h"
#include "PID.h"
#include "wire.h"
#include "ds18b20.h"
#include "DigitalFilters.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Soft Timers
#define HEART_BEAT_TIMER_PERIOD 500
#define TEMP_COILS_PERIOD		749
#define SHOW_TEMP				10000
#define UC_SAMPLE_PERIOD		1
#define VELOCITY_PERIOD			(UC_SAMPLE_PERIOD * 4)
#define CONTROL_TIMER_PERIOD	(VELOCITY_PERIOD * 4)

// ADC
#define ADC_MAX_VALUE 			4095
#define ADC_OFFSET_HALL 		2044
#define ADC_OFFSET_CURRENT 		1505
#define VREF 					3.3
#define SENSITIVITY_HALL 		3.125 // mV/Gauss
#define GAUSS_TO_TESLA 			1e-4
#define SENSITIVITY_CURRENT 	0.2 // V/A

// PID
#define PID_Kp 					4
#define PID_Ki 					6
#define PID_Kd 					0
#define PID_WIND_UP 			50

#define VELOCITY_SET			30

// HBridge Control
#define DEAD_ZONE 				100
#define MAX_PWM					1
#define PROP_CONST				0.011
#define POSITION_TOLERANCE		2.0f

// UART Buffer
#define LINE_MAX_LENGTH 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile static uint16_t ADC_CURRENT[2], ADC_HALL[3];
float CURRENT[2];
extern uint8_t ROM_NO[8];
uint32_t stop1, start1, stop2, start2;
float sonic_speed, x, xset, velocity, uc_left, HALL[3];
CircularBuffer filter_median, filter_moving_average;
FilterLowPass lpf_position, lpf_velocity;
uint8_t uart_rx_buffer;
uint8_t input_done = 0;
int8_t direction = 1;
uint8_t lastZeroCrossing = 0;
static char line_buffer[LINE_MAX_LENGTH + 1];
static uint32_t line_length;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
float HallCalculateTesla(uint16_t ADC_val);
static float CalculateSoundSpeed(float temp1);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float CalculateSoundSpeed(float temp)
{
	return 331.8f + 0.6f * temp;
}

float CalculateCurrent(int16_t ADC_CURRENT)
{
	return (ADC_CURRENT * VREF / ADC_MAX_VALUE) / SENSITIVITY_CURRENT;
}

float HallCalculateTesla(uint16_t ADC_val)
{
	return (float) ((ADC_val - ADC_OFFSET_HALL) * VREF / ADC_MAX_VALUE * 1000
			/ SENSITIVITY_HALL * GAUSS_TO_TESLA);
}

int16_t HBridgeCalculatePWM_max(int32_t HALL)
{
	if (HALL < DEAD_ZONE && HALL > -DEAD_ZONE)
		return 0;
	else if (HALL > DEAD_ZONE)
		return MAX_PWM;
	else
		return -MAX_PWM;
}

int16_t HBridgeCalculatePWM_prop(int32_t HALL)
{
	return HALL * PROP_CONST;
}

void LineAppend(uint8_t value)
{
	if (value == '\r' || value == '\n')
	{
		// odebraliśmy znak końca linii
		if (line_length > 0)
		{
			// dodajemy 0 na końcu linii
			line_buffer[line_length] = '\0';
			// przetwarzamy dane
			xset = atoi(line_buffer);
			input_done = 1;
			// zaczynamy zbieranie danych od nowa
			line_length = 0;
		}
	}
	else
	{
		if (line_length >= LINE_MAX_LENGTH)
		{
			// za dużo danych, usuwamy wszystko co odebraliśmy dotychczas
			line_length = 0;
		}
		// dopisujemy wartość do bufora
		line_buffer[line_length++] = value;
	}
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

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART3_UART_Init();
	MX_TIM4_Init();
	MX_TIM8_Init();
	MX_I2C1_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	// UART
	// ADC
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC_HALL, 3);

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_CURRENT, 2);

//	float HALL[3] = { 0 };

	// H-Bridge
	HBridge coil1, coil2;
	int16_t coil1_PWM = 0, coil2_PWM = 0;
	HBridgeInit(&coil1, DIR1_GPIO_Port, DIR1_Pin, &htim3, TIM_CHANNEL_2);
	HBridgeInit(&coil2, DIR2_GPIO_Port, DIR2_Pin, &htim4, TIM_CHANNEL_1);

	// PI Regulator
	PID pid;
	PIDInit(&pid, PID_Kp, PID_Ki, PID_Kd, PID_WIND_UP);
	int16_t PID_output = 0;
	// Measure temperature for USonic Sensors
	uint8_t ds[2][8] =
	{ 0 };
	int16_t rslt = OW_Search_First();
	uint8_t i = 0;
	while (rslt)
	{
		// print device found
		for (int j = 0; j < 8; j++)
		{
			ds[i][j] = ROM_NO[j];
		}
		i++;
		rslt = OW_Search_Next();
	}
	ds18b20_start_measure(ds[0]);
	ds18b20_start_measure(ds[1]);
	HAL_Delay(750);
	float temp_coils = ds18b20_get_temp(ds[0]) / 16.0f;
	float temp_air = ds18b20_get_temp(ds[1]) / 16.0f;
	sonic_speed = CalculateSoundSpeed(temp_air) / 20000.0f;

	uint8_t wait_or_get = 1;
	uint8_t critical_temp = 0;
	uint8_t send_once = 0;
	uint8_t x_calculate = 0;

	// Start the Timers
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_Delay(100);

	// Init the moving average filters
	FilterMedianInit(&filter_median);
	FilterMovingAverageInit(&filter_moving_average);
	FilterLowPassInit(&lpf_position, 0.01);
	FilterLowPassInit(&lpf_velocity, 0.02);

	// Software Timers
	uint32_t TimerHeartBeat = HAL_GetTick();
	uint32_t TimerControl = HAL_GetTick();
	uint32_t TimerTempCoils = HAL_GetTick();
	uint32_t TimerShowTemp = HAL_GetTick();
	uint32_t TimerUCSample = HAL_GetTick();
	uint32_t TimerVelocity = HAL_GetTick();
	// Start UART to receive user input
	HAL_UART_Receive_IT(&huart2, &uart_rx_buffer, 1);
	uint8_t Message[64];
	uint8_t Length;
//	int32_t x_int = 0, x_int_prev = 0;
	int vel_set = VELOCITY_SET;
	float x_prev = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if ((HAL_GetTick() - TimerUCSample) > UC_SAMPLE_PERIOD)
		{

			static uint16_t start_message = 0;
			start1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
			stop1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			uint16_t input1 = FilterMedianUpdate(&filter_median,
					(stop1 - start1));
			//uint16_t input2 = FilterMovingAverageUpdate(&filter_moving_average, input1);
			//uint16_t input2 = FilterMovingAverageUpdate(&filter_moving_average, input1);
			x = FilterLowPassUpdate(&lpf_position, input1) * sonic_speed;

			if (start_message > 1000)
			{
				x_calculate = 1;
			}
			else
			{
				start_message++;
			}

			TimerUCSample = HAL_GetTick();
		}

		if ((HAL_GetTick() - TimerVelocity) > VELOCITY_PERIOD)
		{
			velocity = abs(
					FilterLowPassUpdate(&lpf_velocity,
							(x - x_prev) / (0.001 * VELOCITY_PERIOD)));
			x_prev = x;
			TimerVelocity = HAL_GetTick();
		}

		if (!send_once && !critical_temp && x_calculate)
		{
			Length = sprintf((char*) Message,
					"Actual x = %.1f Temp: %.1f\n\rGive x:\n\r", x, temp_coils);
			HAL_UART_Transmit_DMA(&huart2, Message, Length);
			send_once = 1;
		}
		// Main Control
		if (((HAL_GetTick() - TimerControl) > CONTROL_TIMER_PERIOD)
				&& input_done && !critical_temp)
		{
			HALL[0] = HallCalculateTesla(ADC_HALL[0]);
			HALL[1] = HallCalculateTesla(ADC_HALL[1]);
			HALL[2] = HallCalculateTesla(ADC_HALL[2]);
			if (abs(xset - x) >= POSITION_TOLERANCE)
			{

				PID_output = abs(
						PIDCalculate(&pid, (int) vel_set, (int) velocity));

				// Go left
				if (x > xset)
				{
					coil1_PWM = HBridgeCalculatePWM_max(
							(ADC_HALL[0] - ADC_OFFSET_HALL));
					coil2_PWM = HBridgeCalculatePWM_max(
							-(ADC_HALL[2] - ADC_OFFSET_HALL));
				}
				// Go right
				else if (x < xset)
				{
					coil1_PWM = HBridgeCalculatePWM_max(
							-(ADC_HALL[0] - ADC_OFFSET_HALL));
					coil2_PWM = HBridgeCalculatePWM_max(
							(ADC_HALL[2] - ADC_OFFSET_HALL));
				}
				HBridgeControl(&coil1, PID_output * coil1_PWM);
				HBridgeControl(&coil2, PID_output * coil2_PWM);
				CURRENT[0] = CalculateCurrent(
						ADC_CURRENT[0] - ADC_OFFSET_CURRENT) * 2;
				CURRENT[1] = CalculateCurrent(
						ADC_CURRENT[1] - ADC_OFFSET_CURRENT) * 2;

				Length = sprintf((char*) Message,
						"x: %.1f PWM1: %d PWM2: %d I1: %.1f I2: %.1f B1: %.4f B2: %.4f\n\r", x,
						coil1_PWM * PID_output, coil2_PWM * PID_output, CURRENT[0], CURRENT[1], HALL[0], HALL[2]);
				HAL_UART_Transmit_DMA(&huart2, Message, Length);

			}
			else
			{
				// Stop
				PIDReset(&pid);
				HBridgeControl(&coil1, 0);
				HBridgeControl(&coil2, 0);

//				Length = sprintf((char*)Message, "Done!\n\r");
//				HAL_UART_Transmit_DMA(&huart2, Message, Length);

				input_done = 0;
				send_once = 0;
			}
			TimerControl = HAL_GetTick();
		}

		// Termometers
		if ((HAL_GetTick() - TimerTempCoils) > TEMP_COILS_PERIOD)
		{
			// Start the measure
			if (wait_or_get)
			{
				ds18b20_start_measure(ds[0]);
				wait_or_get = 0;
			}
			// Get the value
			else if (!wait_or_get)
			{
				temp_coils = ds18b20_get_temp(ds[0]) / 16.0f;
				wait_or_get = 1;
				//Check if the coils are not too hot
				// If so, disable the control
				if (temp_coils > 45.0f && !critical_temp)
				{
					critical_temp = 1;
					HBridgeControl(&coil1, 0);
					HBridgeControl(&coil2, 0);
					Length = sprintf((char*) Message,
							"Coils' temperature is too high: %.1f\n\r",
							temp_coils);
					HAL_UART_Transmit_DMA(&huart2, Message, Length);
				}
				// After cooling enable the control
				else if (critical_temp && temp_coils < 35.0f)
				{
					critical_temp = 0;
					Length = sprintf((char*) Message,
							"Coils cooled down and are ready to use!\n\r");
					HAL_UART_Transmit_DMA(&huart2, Message, Length);

					input_done = 0;
					send_once = 0;
				}

			}
			TimerTempCoils = HAL_GetTick();
		}

		if (((HAL_GetTick() - TimerShowTemp) > SHOW_TEMP) && critical_temp)
		{
			Length = sprintf((char*) Message, "Actual T: %.1f\n\r", temp_coils);
			HAL_UART_Transmit_DMA(&huart2, Message, Length);
			printf("Actual T: %.1f\n\r", temp_coils);
			TimerShowTemp = HAL_GetTick();
		}

		// Blinking LD2 to show correct flow of the program
		if ((HAL_GetTick() - TimerHeartBeat) > HEART_BEAT_TIMER_PERIOD)
		{
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			TimerHeartBeat = HAL_GetTick();
		}

		// TEST GITAAAAAAAAAA
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void)
{
	/* ADC3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(ADC3_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(ADC3_IRQn);
	/* ADC1_2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
	/* TIM3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	/* TIM4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	/* USART2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		LineAppend(uart_rx_buffer);
		HAL_UART_Receive_IT(huart, &uart_rx_buffer, 1);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

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
