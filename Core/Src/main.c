/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>© Copyright (c) 2019 STMicroelectronics.
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
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BITS              12
#define ADC_MAX_VALUE         ((1 << ADC_BITS) - 1)   // 4095
#define VREF_MV               2500.0f

#define MIN_CURRENT_MA        4.0f
#define MAX_CURRENT_MA        20.0f
#define MIN_VOLTAGE_ADC_MV    100.0f     // conforme CN0336 (0.1V)
#define MAX_VOLTAGE_ADC_MV    2400.0f    // conforme CN0336 (2.4V)

#define NUM_SAMPLES           30

#define AD7091R_CS_Pin        ADC_CS_Pin
#define AD7091R_CS_Port       ADC_CS_GPIO_Port
#define AD7091R_CMD_RESET     0xFFFF
#define AD7091R_CMD_NORMAL    0x0020
//#define CALIBRATED_RAW_MIN    (uint16_t)325  // Exemplo: leitura do ADC para 4mA
//#define CALIBRATED_RAW_MAX    (uint16_t)3875 // Exemplo: leitura do ADC para 20mA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char writeValue[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
uint16_t AD7091R_ReadData(void);
uint16_t get_filtered_reading(void);
float raw_to_voltage_mV(uint16_t raw_value);
float voltage_to_current_mA(float voltage_mv);
void serialPrint(const char* message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void AD7091R_Init(void)
{
    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_RESET);
    uint16_t reset_cmd_tx = AD7091R_CMD_RESET;
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&reset_cmd_tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_RESET);
    uint16_t normal_cmd_tx = AD7091R_CMD_NORMAL;
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&normal_cmd_tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_SET);
}

uint16_t AD7091R_ReadData(void)
{
    uint8_t rx_buf[2] = {0};
    uint16_t adc_raw_16bits, adc_value_12bits;

    HAL_Delay(100);

    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Receive(&hspi1, rx_buf, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_SET);  // LED erro
        HAL_Delay(100);
        HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_RESET);
        serialPrint("Erro na comunicacao SPI!\r\n");
        return 0;
    }
    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_TogglePin(ld1_GPIO_Port, ld1_Pin);  // LED VERDE SUCESSO

    adc_raw_16bits = (rx_buf[0] << 8) | rx_buf[1];
    adc_value_12bits = (adc_raw_16bits >> 2) & 0x0FFF;

    return adc_value_12bits;
}

uint16_t get_filtered_reading(void)
{
    uint32_t sum = 0;
    for(uint8_t i = 0; i < NUM_SAMPLES; i++) {
        sum += AD7091R_ReadData();
        HAL_Delay(100);
    }
    return (uint16_t)(sum / NUM_SAMPLES);
}

float raw_to_voltage_mV(uint16_t raw_value)
{
    return ((float)raw_value / ADC_MAX_VALUE) * VREF_MV;
}

float voltage_to_current_mA(float voltage_mv)
{
    if (voltage_mv < MIN_VOLTAGE_ADC_MV) voltage_mv = MIN_VOLTAGE_ADC_MV;
    if (voltage_mv > MAX_VOLTAGE_ADC_MV) voltage_mv = MAX_VOLTAGE_ADC_MV;

    return 4.0f + ((voltage_mv - MIN_VOLTAGE_ADC_MV) * (16.0f / (MAX_VOLTAGE_ADC_MV - MIN_VOLTAGE_ADC_MV)));
}

void serialPrint(const char* message)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint16_t raw_value = get_filtered_reading();
    float voltage_mv = raw_to_voltage_mV(raw_value);
    float current_ma = voltage_to_current_mA(voltage_mv);
    float percentage = ((current_ma - MIN_CURRENT_MA) / (MAX_CURRENT_MA - MIN_CURRENT_MA)) * 100.0f;

    sprintf(writeValue, "ADC:%4u | V:%.2fmV | I:%.2fmA | %%:%.1f\r\n", raw_value, voltage_mv, current_ma, percentage);
    serialPrint(writeValue);
    serialPrint("---\r\n");

    HAL_Delay(1000);

    HAL_GPIO_TogglePin(ld3_GPIO_Port, ld3_Pin);  // LED AZUL - Loop ativo
    HAL_Delay(1000);
  }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
