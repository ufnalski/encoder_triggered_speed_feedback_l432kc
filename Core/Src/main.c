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
#include "dac.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ssd1306.h"
#include "pid_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define OLED_REFRESH_RATE 400 // ms
#define UPDATE_OLED 1

// https://sekigon-gonnoc.github.io/web-serial-plotter/
#define SERIAL_PLOTTER_REFRESH_RATE 5 // ms

#define SLAVE_TICKS_PER_SECOND 1000000
#define TICKS_PER_REVOLUTION 5  // Stabilus
//#define PULSE_TIMING_ARR (0xFFFF / CONTROL_UPDATE_DIV)  // see main.h

#define SAMPLING_TIME 0.002 // TIM6
#define TIME_CONSTANT_REF_SHAPE ( 0.03 * 2 )
#define BETA_COEFF_REF_SHAPE ( SAMPLING_TIME / TIME_CONSTANT_REF_SHAPE )

#define BETA_COEFF_DISPLAY ( SAMPLING_TIME / 0.1 )

#define PWM_ARR 1000
#define PWM_MID_CCR 500

#define PID_KP 0.5
#define PID_KI 10.0

#define PID_KD 0.0
#define PID_TAU 0.06
#define CONTROL_SIGNAL_MAX 500.0
#define CONTROL_SIGNAL_MIN -500.0
#define REF_SPEED_MAX 6001.0  // rpm
#define REF_SPEED_MIN -6001.0  // rpm
#define REF_SPEED_INCRENEMT 1000.0
#define ENC_PERIOD_NOM 2000.0

#define REF_PRESET_STEPS 10
#define REF_PRESET_STEP_LENGTH 2000  // ms

#define SMOOTHING_THRESHOLD 100

#define DO_SMOOTHING 0

#define DEBOUNCE_DIR_TIME 50  // ms

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t softTimerOLED;
char lcd_line[32];

uint32_t softTimerUART;
float systick_time;  // for serial plotter

uint32_t softTimerDebounceDir;
GPIO_PinState button_state = GPIO_PIN_SET;

volatile uint32_t enc_period;
volatile uint32_t enc_period_table[4];
volatile double velocity_signed;
volatile double speed_unsigned;
volatile double velocity_filtered;
volatile double ref_speed;
volatile double ref_speed_shaped;
volatile double ref_speed_raw = 0;

volatile uint8_t smoothing_flag = 0;

volatile uint16_t pwm_ccr = PWM_MID_CCR;
volatile uint8_t up_down_dir = 0;
volatile uint32_t softTimerDebounceRef;

PID_t pid;
volatile int16_t control_signal;

uint32_t softTimerRefSTEPS;
volatile uint8_t preset_ref_sequence_flag = 1;
double preset_ref_sequence[REF_PRESET_STEPS] =
{ 0, 6000, -6000, 0, 4000, -4000, 0, -2000, 0, 2000 };
volatile uint8_t preset_idx = 0;

volatile uint16_t overflow_cnt = 0;
volatile uint16_t overflow_cnt_previous = 0;
volatile uint8_t overflow_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int8_t TIM_ReadEncDir(TIM_TypeDef *TIMx);

double LowPassFilter(double _in_lpf, double _beta_coeff);
double LowPassFilterRefStage1(double _in_lpf, double _beta_coef);
double LowPassFilterRefStage2(double _in_lpf, double _beta_coef);

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM15_Init();
	MX_TIM2_Init();
	MX_TIM6_Init();
	MX_DAC1_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);

	HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);  // pulse timing for encoder

	HAL_TIM_OC_Start_IT(&htim15, TIM_CHANNEL_2);  // zero speed detection

	HAL_TIM_Base_Start_IT(&htim6);  // digital filter pacing

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_MID_CCR);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // PWM_L
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_ARR - PWM_MID_CCR);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // PWM_R

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(20, 2);
	ssd1306_WriteString("ufnalski.edu.pl", Font_6x8, White);
	ssd1306_SetCursor(16, 18);
	ssd1306_WriteString("Encoder triggered", Font_6x8, White);
	ssd1306_SetCursor(6, 34);
	ssd1306_WriteString("speed control system", Font_6x8, White);
	ssd1306_SetCursor(8, 50);
	ssd1306_WriteString("(Stabilus POWERISE)", Font_6x8, White);
	ssd1306_UpdateScreen();

	HAL_Delay(4000);

	softTimerOLED = HAL_GetTick();
	softTimerUART = HAL_GetTick();
	softTimerDebounceDir = HAL_GetTick();
	softTimerDebounceRef = HAL_GetTick();
	softTimerRefSTEPS = HAL_GetTick();

	PID_Init_Bartek_s_Lab(&pid, PID_KP, PID_KI, PID_KD, PID_TAU,
	CONTROL_SIGNAL_MIN, CONTROL_SIGNAL_MAX);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (((HAL_GetTick() - softTimerRefSTEPS) > REF_PRESET_STEP_LENGTH)
				&& (preset_ref_sequence_flag == 1))
		{
			softTimerRefSTEPS = HAL_GetTick();

			preset_idx++;
			preset_idx %= REF_PRESET_STEPS;
			ref_speed_raw = preset_ref_sequence[preset_idx];
		}

		if ((HAL_GetTick() - softTimerUART) > SERIAL_PLOTTER_REFRESH_RATE)
		{
			softTimerUART = HAL_GetTick();

			// Web Serial Plotter (sekigon-gonnoc)
			// https://github.com/sekigon-gonnoc/web-serial-plotter
			systick_time = ((float) HAL_GetTick()) / 1000.0f;
			printf(
					"Systick_time:%.3f,Ref_speed_shaped:%.2f,Angular_speed:%.2f,Enc_period:%lu,Control_signal:%d,Smoothing_flag:%d, kP:%.3f, kI:%.3f\r\n",
					systick_time, ref_speed_shaped, velocity_signed, enc_period,
					-10 * control_signal, smoothing_flag, pid.kp, pid.ki);
		}

		if (((HAL_GetTick() - softTimerOLED) > OLED_REFRESH_RATE) && UPDATE_OLED)
		{
			softTimerOLED = HAL_GetTick();

			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

			ssd1306_Fill(Black);
			ssd1306_SetCursor(16, 0);
			ssd1306_WriteString("Encoder triggered", Font_6x8, White);

			ssd1306_SetCursor(6, 12);
			sprintf(lcd_line, "speed control system");
			ssd1306_WriteString(lcd_line, Font_6x8, White);

			ssd1306_SetCursor(5, 25);
			sprintf(lcd_line, "ENC_DIR = %i  ", TIM_ReadEncDir(TIM1));
			ssd1306_WriteString(lcd_line, Font_6x8, White);

			ssd1306_SetCursor(5, 35);
			sprintf(lcd_line, "T_PULSE = %lu", enc_period);

			ssd1306_WriteString(lcd_line, Font_6x8, White);

			ssd1306_SetCursor(5, 47);
			sprintf(lcd_line, "W = %.0f [rpm]/%d", velocity_signed, pwm_ccr);
			ssd1306_WriteString(lcd_line, Font_6x8, White);

			ssd1306_SetCursor(5, 56);
			sprintf(lcd_line, "W = %.2f [rpm x1000]",
					velocity_filtered / 1000.0);
			ssd1306_WriteString(lcd_line, Font_6x8, White);

			ssd1306_UpdateScreen();
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

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
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

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if ((GPIO_Pin == WHITE_BUTTON_Pin)
			&& (HAL_GetTick() - softTimerDebounceRef > 200))
	{
		preset_ref_sequence_flag = 0;
		softTimerDebounceRef = HAL_GetTick();
		if (up_down_dir == 0)
		{
			if (ref_speed_raw >= REF_SPEED_MIN + REF_SPEED_INCRENEMT)
			{
				ref_speed_raw -= REF_SPEED_INCRENEMT;
			}
			else
			{
				up_down_dir = 1;
			}
		}
		else
		{
			if (REF_SPEED_MAX - ref_speed_raw >= REF_SPEED_INCRENEMT)
			{
				ref_speed_raw += REF_SPEED_INCRENEMT;
			}
			else
			{
				up_down_dir = 0;
			}
		}

	}
}

int8_t TIM_ReadEncDir(TIM_TypeDef *TIMx)
{
	if (((TIMx->CR1) & (0x00000001 << 4)) == 0)  // DIR bit
	{
		return 1;
	}
	else
	{
		return -1;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM15)
	{
		HAL_GPIO_TogglePin(LOGIC_ANALYZER_GPIO_Port, LOGIC_ANALYZER_Pin);
		enc_period = TIM15->CCR1 + (PULSE_TIMING_ARR * overflow_cnt);
		overflow_flag = (overflow_cnt != 0);
		overflow_cnt_previous = overflow_cnt;
		overflow_cnt = 0;

		enc_period_table[3] = enc_period_table[2];
		enc_period_table[2] = enc_period_table[1];
		enc_period_table[1] = enc_period_table[0];
		enc_period_table[0] = enc_period;

		if (abs(
				(int32_t) enc_period
						- (int32_t) (enc_period_table[1]))< SMOOTHING_THRESHOLD)
		{
			smoothing_flag = 1;
		}
		else
		{
			smoothing_flag = 0;
		}

		if ((smoothing_flag == 1) && (DO_SMOOTHING))
		{
			speed_unsigned = (double) SLAVE_TICKS_PER_SECOND
					/ (double) TICKS_PER_REVOLUTION
					/ (((double) enc_period_table[0]
							+ (double) enc_period_table[1]
							+ (double) enc_period_table[2]
							+ (double) enc_period_table[3]) / 4.0) * 60.0;
		}
		else
		{
			speed_unsigned = (double) SLAVE_TICKS_PER_SECOND
					/ (double) TICKS_PER_REVOLUTION / (double) enc_period
					* 60.0;
		}

		velocity_signed = ((double) TIM_ReadEncDir(TIM1)) * speed_unsigned;

		if (overflow_flag == 0)
		{
			PI_Adapt_Bartek_s_Lab(&pid,
			PID_KP * ENC_PERIOD_NOM / (double) enc_period,
					PID_KI * ENC_PERIOD_NOM * ENC_PERIOD_NOM
							/ (double) enc_period / (double) enc_period);
		}
		else
		{
			PI_Adapt_Bartek_s_Lab(&pid,
			PID_KP * (double) ENC_PERIOD_NOM / (double) PULSE_TIMING_ARR,
					PID_KI * (double) ENC_PERIOD_NOM * (double) ENC_PERIOD_NOM
							/ (double) PULSE_TIMING_ARR
							/ (double) PULSE_TIMING_ARR);
		}

		control_signal = (int16_t) PID_Controller_Bartek_s_Lab(&pid,
				ref_speed_shaped, velocity_signed,
				(double) enc_period / (double) SLAVE_TICKS_PER_SECOND);
		pwm_ccr = control_signal + 500;

//		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pwm_ccr * 4);
//		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
//				speed_unsigned / 2);

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_ccr);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_ARR - pwm_ccr);
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM15)
	{

		overflow_cnt++;
		if (overflow_cnt_previous < overflow_cnt)
		{
			enc_period = PULSE_TIMING_ARR * overflow_cnt;
		}

		speed_unsigned = (double) SLAVE_TICKS_PER_SECOND
				/ (double) TICKS_PER_REVOLUTION / enc_period * 60.0;

		velocity_signed = ((double) TIM_ReadEncDir(TIM1)) * speed_unsigned;

//		enc_period_table[3] = enc_period_table[2];
//		enc_period_table[2] = enc_period_table[1];
//		enc_period_table[1] = enc_period_table[0];
//		enc_period_table[0] = enc_period;

		PI_Adapt_Bartek_s_Lab(&pid,
		PID_KP * (double) ENC_PERIOD_NOM / (double) PULSE_TIMING_ARR,
				PID_KI * (double) ENC_PERIOD_NOM * (double) ENC_PERIOD_NOM
						/ (double) PULSE_TIMING_ARR / (double) PULSE_TIMING_ARR);

		control_signal = (int16_t) PID_Controller_Bartek_s_Lab(&pid,
				ref_speed_shaped, velocity_signed,
				(double) PULSE_TIMING_ARR / (double) SLAVE_TICKS_PER_SECOND);
		pwm_ccr = control_signal + 500;

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pwm_ccr * 4);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
				speed_unsigned / 2);

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_ccr);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_ARR - pwm_ccr);
	}
}

double LowPassFilterRefStage1(double _in_lpf, double _beta_coeff)
{
	static double out_lpf = 0;
	if ((_in_lpf == INFINITY) || (_in_lpf == -INFINITY))
	{
		_in_lpf = 0;
	}
	out_lpf = _in_lpf * _beta_coeff + out_lpf * (1 - _beta_coeff);
	return out_lpf;
}

double LowPassFilterRefStage2(double _in_lpf, double _beta_coeff)
{
	static double out_lpf = 0;
	if ((_in_lpf == INFINITY) || (_in_lpf == -INFINITY))
	{
		_in_lpf = 0;
	}
	out_lpf = _in_lpf * _beta_coeff + out_lpf * (1 - _beta_coeff);
	return out_lpf;
}

double LowPassFilter(double _in_lpf, double _beta_coeff)
{
	static double out_lpf = 0;
	if ((_in_lpf == INFINITY) || (_in_lpf == -INFINITY))
	{
		_in_lpf = 0;
	}
	out_lpf = _in_lpf * _beta_coeff + out_lpf * (1 - _beta_coeff);
	return out_lpf;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
		velocity_filtered = LowPassFilter(velocity_signed, BETA_COEFF_DISPLAY); // just for OLED display

		if ((HAL_GetTick() - softTimerDebounceDir) > DEBOUNCE_DIR_TIME)
		{
			softTimerDebounceDir = HAL_GetTick();
			GPIO_PinState button_state_new = HAL_GPIO_ReadPin(
			SPEED_SELECT_GPIO_Port, SPEED_SELECT_Pin);
			if ((button_state_new == GPIO_PIN_SET) && (button_state =
					GPIO_PIN_SET))
			{
				ref_speed = ref_speed_raw;
			}
			else
			{
				ref_speed = -ref_speed_raw;
			}
			button_state = button_state_new;
		}

		ref_speed_shaped = LowPassFilterRefStage1(ref_speed,
		BETA_COEFF_REF_SHAPE);
		ref_speed_shaped = LowPassFilterRefStage2(ref_speed_shaped,
		BETA_COEFF_REF_SHAPE);
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
#ifdef USE_FULL_ASSERT
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
