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
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>  // Para sprintf
#include <string.h> // Para strlen

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Definições do AD7091R e da escala 4-20 mA
#define ADC_BITS         12                                // O AD7091R é um ADC de 12 bits
#define ADC_MAX_VALUE    ((1 << ADC_BITS) - 1)             // Valor máximo para 12 bits (4095)
#define VREF_MV          2500.0F                           // Tensão de referência do AD7091R em mV (2.5V típica para o CN0336)

// Definições para a escala 4-20 mA
#define MIN_CURRENT_MA   4.0F                              // Corrente mínima em mA
#define MAX_CURRENT_MA   20.0F                             // Corrente máxima em mA



#define MIN_VOLTAGE_ADC_MV 0.0F                            // Tensão mínima esperada na entrada do ADC (0V)
#define MAX_VOLTAGE_ADC_MV VREF_MV                         // Tensão máxima esperada na entrada do ADC (2.5V = VREF)


#define AD7091R_CS_Pin       ADC_CS_Pin
#define AD7091R_CS_Port      ADC_CS_GPIO_Port
#define AD7091R_CONVST_Pin   GPIO_PIN_1 // Exemplo: PA1
#define AD7091R_CONVST_Port  GPIOA      // Exemplo: GPIOA
#define AD7091R_CMD_RESET    0xFFFF
#define AD7091R_CMD_STANDBY  0x0000
#define AD7091R_CMD_NORMAL   0x0020

// Parâmetros de medição (para a função get_filtered_reading)
#define NUM_SAMPLES 16         // Número de amostras para filtro de média móvel
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char writeValue[128]; // Buffer para mensagens UART (aumentado para mensagens mais longas)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void AD7091R_Init(void);
uint16_t AD7091R_ReadData(void);
uint16_t get_filtered_reading(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Inicializa o ADC AD7091R
  * @note  Esta função configura o AD7091R para o modo de operação normal.
  * Assume que os pinos CS e CONVST já foram configurados como GPIO Output
  * e o SPI já foi inicializado via STM32CubeMX.
  */
void AD7091R_Init(void)
{
    // 1. Reset do ADC (comando 0xFFFF)
    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_RESET); // CS LOW
    uint16_t reset_cmd_tx = AD7091R_CMD_RESET; // O comando é de 16 bits
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&reset_cmd_tx, 2, HAL_MAX_DELAY); // Envia 2 bytes
    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_SET); // CS HIGH
    HAL_Delay(1); // Tempo de recuperação do reset

    // 2. Configurar para modo normal de operação (comando 0x0020)
    // O AD7091R entra em modo normal de operação após reset ou comando NORMAL.
    // Para leituras contínuas, não é estritamente necessário enviar este comando
    // a cada ciclo de leitura, apenas na inicialização.

    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_RESET); // CS LOW
    uint16_t normal_cmd_tx = AD7091R_CMD_NORMAL; // O comando é de 16 bits
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&normal_cmd_tx, 2, HAL_MAX_DELAY); // Envia 2 bytes
    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_SET); // CS HIGH
    HAL_Delay(1); // Pequeno atraso

    // Garante que CONVST está em estado alto inicialmente
    HAL_GPIO_WritePin(AD7091R_CONVST_Port, AD7091R_CONVST_Pin, GPIO_PIN_SET);
}

/**
  * @brief Lê um valor do AD7091R.
  * Realiza um pulso no pino CONVST para iniciar a conversão
  * e então lê os 16 bits de dados do ADC via SPI, extraindo os 12 bits válidos.
  * @retval Valor digital de 12 bits (0 a 4095). Retorna 0 em caso de erro SPI.
  */
uint16_t AD7091R_ReadData(void)
{
    uint8_t rx_buf[2] = {0}; // Buffer para receber 2 bytes (16 bits)
    uint16_t adc_raw_16bits; // Valor bruto de 16 bits lido
    uint16_t adc_value_12bits; // Valor final de 12 bits

    // 1. Iniciar conversão (pulso baixo no CONVST)
    HAL_GPIO_WritePin(AD7091R_CONVST_Port, AD7091R_CONVST_Pin, GPIO_PIN_RESET); // CONVST LOW
    HAL_Delay(1);
    HAL_GPIO_WritePin(AD7091R_CONVST_Port, AD7091R_CONVST_Pin, GPIO_PIN_SET);   // CONVST HIGH

    // 2. Esperar conversão completar
    HAL_Delay(1);

    // 3. Ler resultado via SPI
    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_RESET); // CS LOW

    // Envia 2 bytes dummy (0x00) e recebe 2 bytes do ADC.
    if (HAL_SPI_Receive(&hspi1, rx_buf, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        // --- INÍCIO DA LÓGICA DO LED DE ERRO (VERMELHO) ---
        HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_SET); // Garante CS HIGH em caso de erro

        // Pisca o LED VERMELHO uma vez para indicar falha
        HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_SET); // Liga LED Vermelho
        HAL_Delay(100); // Fica ligado por 100ms
        HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_RESET); // Desliga LED Vermelho

        // Para evitar que o LED VERDE pisque em caso de erro, garantimos que ele esteja desligado
        HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_RESET);

        serialPrint("Erro na comunicacao SPI!\r\n"); // Mensagem de erro para o terminal
        return 0; // Retorna 0 ou um valor de erro
    }
    // --- FIM DA LÓGICA DO LED DE ERRO (VERMELHO) ---

    HAL_GPIO_WritePin(AD7091R_CS_Port, AD7091R_CS_Pin, GPIO_PIN_SET); // CS HIGH

    // --- INÍCIO DA LÓGICA DO LED DE SUCESSO (VERDE) ---
    // Pisca o LED VERDE uma vez para indicar sucesso
    HAL_GPIO_TogglePin(ld1_GPIO_Port, ld1_Pin);
    HAL_Delay(100);
    /*HAL_GPIO_WritePin(ld1_GPIO_Port, ld1_Pin, GPIO_PIN_SET); // Liga LED Verde
    HAL_Delay(500); // Fica ligado por 100ms
    HAL_GPIO_WritePin(ld1_GPIO_Port, ld1_Pin, GPIO_PIN_RESET); // Desliga LED Verde*/

    // Para evitar que o LED VERMELHO pisque, garantimos que ele esteja desligado
    HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_RESET);
    // --- FIM DA LÓGICA DO LED DE SUCESSO (VERDE) ---

    // Combinar bytes e extrair os 12 bits de dados
    adc_raw_16bits = (uint16_t)(rx_buf[0] << 8) | rx_buf[1];
    adc_value_12bits = (adc_raw_16bits >> 2) & 0x0FFF; // Desloca 2 bits para a direita e máscara para 12 bits

    return adc_value_12bits;
}

/**
  * @brief Obtém leitura filtrada com média móvel
  * @return Valor filtrado de 12 bits.
  */
uint16_t get_filtered_reading(void)
{
    uint32_t sum = 0;
    for(uint8_t i = 0; i < NUM_SAMPLES; i++) {
        sum += AD7091R_ReadData();
        HAL_Delay(1); // Pequeno atraso entre amostras
    }
    return (uint16_t)(sum / NUM_SAMPLES);
}

/**
  * @brief Converte valor bruto do ADC para tensão em milivolts (mV).
  * @param raw_value Valor digital de 12 bits lido do ADC.
  * @return Tensão em milivolts.
  */
float raw_to_voltage_mV(uint16_t raw_value)
{
    // Tensão (mV) = (Valor ADC / Valor Máximo ADC) * VREF (mV)
    return ((float)raw_value / ADC_MAX_VALUE) * VREF_MV;
}

/**
  * @brief Converte tensão em milivolts para corrente em mA (escala 4-20 mA).
  * Assume uma relação linear entre a tensão lida pelo ADC (0-2500mV)
  * e a corrente (4-20mA).
  * @param voltage_mv Tensão medida em milivolts.
  * @return Corrente calculada em mA.
  */
float voltage_to_current_mA(float voltage_mv)
{
    // A fórmula de mapeamento linear é:
    // Y = Y_min + ( (X - X_min) * (Y_max - Y_min) ) / (X_max - X_min)
    // Onde:
    // Y = current_ma (saída)
    // X = voltage_mv (entrada)
    // Y_min = MIN_CURRENT_MA = 4.0F
    // Y_max = MAX_CURRENT_MA = 20.0F
    // X_min = MIN_VOLTAGE_ADC_MV = 0.0F (assumindo 0V para 4mA na entrada do ADC)
    // X_max = MAX_VOLTAGE_ADC_MV = 2500.0F (assumindo 2.5V para 20mA na entrada do ADC)

    // garanta que a tensão está dentro da faixa esperada para evitar erros.
    if (voltage_mv < MIN_VOLTAGE_ADC_MV) voltage_mv = MIN_VOLTAGE_ADC_MV;
    if (voltage_mv > MAX_VOLTAGE_ADC_MV) voltage_mv = MAX_VOLTAGE_ADC_MV;

    return MIN_CURRENT_MA + ((voltage_mv - MIN_VOLTAGE_ADC_MV) * (MAX_CURRENT_MA - MIN_CURRENT_MA)) / (MAX_VOLTAGE_ADC_MV - MIN_VOLTAGE_ADC_MV);
}


/**
  * @brief  Função para enviar strings para o terminal serial via UART.
  * @param  message: Ponteiro para a string a ser enviada.
  * @retval None
  */
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
  /* USER CODE BEGIN 2 */
  AD7091R_Init(); // Inicializa o ADC AD7091R
  serialPrint("Sistema de aquisicao de dados 4-20 mA com STM32H753ZI inicializado...\r\n");
  serialPrint("Configuracao completa. Aguardando leituras...\r\n");
  serialPrint("---\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1. Obter leitura filtrada do ADC (valor RAW)
    uint16_t raw_value = get_filtered_reading();
    sprintf(writeValue, "RAW ADC (0-%d): %u\r\n", ADC_MAX_VALUE, raw_value);
    serialPrint(writeValue);

    // 2. Converter o valor RAW para Tensão em mV
    float voltage_mv = raw_to_voltage_mV(raw_value);
    sprintf(writeValue, "Tensao lida (mV): %.2f\r\n", voltage_mv);
    serialPrint(writeValue);

    // 3. Converter a Tensão para Corrente em mA
    float current_ma = voltage_to_current_mA(voltage_mv);
    sprintf(writeValue, "Corrente Calculada (mA): %.2f\r\n", current_ma);
    serialPrint(writeValue);

    // 4. Conversão da Corrente para Porcentagem (0-100%)
    float percentage = ((current_ma - MIN_CURRENT_MA) / (MAX_CURRENT_MA - MIN_CURRENT_MA)) * 100.0F;
    sprintf(writeValue, "Valor em Porcentagem (0-100%%): %.2f%%\r\n", percentage);
    serialPrint(writeValue);

    serialPrint("---\r\n"); // Separador para as leituras
    HAL_Delay(1000); // Espera 1 segundo antes da próxima leitura
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
