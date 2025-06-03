/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */



/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Private defines -----------------------------------------------------------*/
#define ntc_Pin GPIO_PIN_2
#define ntc_GPIO_Port GPIOA
#define pot_Pin GPIO_PIN_4

#define pot_GPIO_Port GPIOA
#define ntcAlto_Pin GPIO_PIN_8
#define ntcAlto_GPIO_Port GPIOD
#define ntcBajo_Pin GPIO_PIN_9
#define ntcBajo_GPIO_Port GPIOD
#define ntcMedio_Pin GPIO_PIN_11
#define ntcMedio_GPIO_Port GPIOD
#define potBajo_Pin GPIO_PIN_12
#define potBajo_GPIO_Port GPIOD
#define potAlto_Pin GPIO_PIN_13
#define potAlto_GPIO_Port GPIOD
#define potMedio_Pin GPIO_PIN_15
#define potMedio_GPIO_Port GPIOD


// Tiempo en ms que permanecen encendidas las luces al iniciar el sistema
#define TENCENDIDO 2000
// Tiempo en ms que permanecen apagadas las luces antes iniciar las transiciones
#define TAPAGADO 2000


// CAMBIAMOS ESTO
// Nivel umbral entre los rango bajo y medio de las lecturas del termistor NTC
//#define NTC_INFERIOR 3600
// Nivel umbral entre los rango medio y alto de las lecturas del termistor NTC
//#define NTC_SUPERIOR 3500
// Nivel umbral entre los rango bajo y medio de las lecturas del potenciómetro
//#define POT_INFERIOR 1366
// Nivel umbral entre los rango medio y alto de las lecturas del potenciómetro
//#define POT_SUPERIOR 2730
//

// -----------------------------ZONAS DE CAMBIO------------------------------*/
// Defino zonas de cambio

// NTC
#define NTC_INF_INF 3615
#define NTC_INF_SUP 3585

#define NTC_SUP_INF 3515
#define NTC_SUP_SUP 3485

// POTE
#define POT_INF_INF 1350
#define POT_INF_SUP 1382

#define POT_SUP_INF 2715
#define POT_SUP_SUP 2745



// Cantidad de canales que se muestrean
#define CANALES 2
// Cantidad de muestras a promediar por canal
#define MUESTRAS 16
// Tamaño mínimo requerido de buffer
#define BUFFER_MINIMO (MUESTRAS * CANALES)
// Cantidad de buffers mínimos que se emplean
#define CANTIDAD_BUFFER_MINIMO 2
// Tamaño del buffer a imlementar
#define BUFFER_TOTAL (CANTIDAD_BUFFER_MINIMO * BUFFER_MINIMO)


/* USER CODE END PD */



/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

// GLOBALES
// Buffer para almacenar las muestras
uint16_t dma_buffer[BUFFER_TOTAL];
// Valor promedio de las muestras del termistor NTC
volatile uint16_t ntc_avg;
// Valor promedio de las muestras del potenciometro
volatile uint16_t pot_avg;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);



/* USER CODE BEGIN PFP */

void inicio(void); // Prueba e inicialización del sistema
void apagarTodos(void); // Apaga todos los LED
void encenderTodos(void); // Enciende todos los LED
void promediar(uint16_t *buffer); // Calcula el promedio de las muestras
void mostrarNtc(void); // Maneja los LED indicadores para el termistor NTC
void mostrarPot(void); // Maneja los LED indicadores para el potenciómetro
void mostrar(void); // Maneja los LED indicadores de rangos

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
  MX_DMA_Init();
  MX_ADC1_Init();



  // -----------------------------INICIALIZACION DE PLACA------------------------------*/
  /* USER CODE BEGIN 2 */

  inicio(); // Probamos los LED e Inicializamos
  HAL_GPIO_WritePin(potMedio_GPIO_Port, potMedio_Pin, GPIO_PIN_SET);


  /* USER CODE END 2 */





  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	// Gestionamos los LED acorde a los valores promedio obtenidos
	mostrar();


  }
  /* USER CODE END 3 */
}

// -----------------------------END MAIN CODE------------------------------*/








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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ntcAlto_Pin|ntcBajo_Pin|ntcMedio_Pin|poteBajo_Pin
                          |poteAlto_Pin|LD5_Pin|poteMedio_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ntcAlto_Pin ntcBajo_Pin ntcMedio_Pin poteBajo_Pin
                           poteAlto_Pin LD5_Pin poteMedio_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = ntcAlto_Pin|ntcBajo_Pin|ntcMedio_Pin|poteBajo_Pin
                          |poteAlto_Pin|LD5_Pin|poteMedio_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}




// -----------------------------DEFINICION DE FUNCIONES------------------------------*/
/* USER CODE BEGIN 4 */







// Rutina de antención a la interrupción de buffer a mitad
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
// Promediamos la primera mitad del buffer (el primer bloque de tamaño mínimo)
promediar(&dma_buffer[0]);
}
// Rutina de antención a la interrupción de buffer a tope
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
// Promediamos la segunda mitad del buffer (el segundo bloque de tamaño mínimo)
promediar(&dma_buffer[BUFFER_MINIMO]);
}
// Prueba e inicialización del sistema
void inicio(void){
// Encendemos todos los LED y esperamos un tiempo
encenderTodos();
HAL_Delay(TENCENDIDO);
// Apagamos todos los LED y esperamos un tiempo
apagarTodos();
HAL_Delay(TAPAGADO);
// Iniciamos las conversiones
HAL_ADC_Start_DMA(&hadc1, (uint32_t*) dma_buffer, BUFFER_TOTAL);
}
// Apaga todos los LED
void apagarTodos(void){
HAL_GPIO_WritePin( ntcBajo_GPIO_Port, ntcBajo_Pin, GPIO_PIN_RESET); // Verde A
HAL_GPIO_WritePin(ntcMedio_GPIO_Port, ntcMedio_Pin, GPIO_PIN_RESET); // Amarillo A
HAL_GPIO_WritePin( ntcAlto_GPIO_Port, ntcAlto_Pin, GPIO_PIN_RESET); // Rojo A

HAL_GPIO_WritePin( potBajo_GPIO_Port, potBajo_Pin, GPIO_PIN_RESET); // Verde B
HAL_GPIO_WritePin(potMedio_GPIO_Port, potMedio_Pin, GPIO_PIN_RESET); // Amarillo B
HAL_GPIO_WritePin( potAlto_GPIO_Port, potAlto_Pin, GPIO_PIN_RESET); // Rojo B
}
// Enciende todos los LED
void encenderTodos(void){
HAL_GPIO_WritePin( ntcBajo_GPIO_Port, ntcBajo_Pin, GPIO_PIN_SET); // Verde A
HAL_GPIO_WritePin(ntcMedio_GPIO_Port, ntcMedio_Pin, GPIO_PIN_SET); // Amarillo A
HAL_GPIO_WritePin( ntcAlto_GPIO_Port, ntcAlto_Pin, GPIO_PIN_SET); // Rojo A
HAL_GPIO_WritePin( potBajo_GPIO_Port, potBajo_Pin, GPIO_PIN_SET); // Verde B
HAL_GPIO_WritePin(potMedio_GPIO_Port, potMedio_Pin, GPIO_PIN_SET); // Amarillo B
HAL_GPIO_WritePin( potAlto_GPIO_Port, potAlto_Pin, GPIO_PIN_SET); // Rojo B
}
// Calcula los promedios
void promediar(uint16_t *buffer) {
// Inicializamos variables para acumular la suma de las muestras
uint32_t ntc_sum = 0, pot_sum = 0;
// Ralizamos la suma incrementando el puntero a las muestras
for (int i = 0; i < MUESTRAS; ++i) {
ntc_sum += buffer[0];
pot_sum += buffer[1];
buffer += 2;
}
// Dividimos las sumas para obtener el promedio
ntc_avg = ntc_sum / MUESTRAS;
pot_avg = pot_sum / MUESTRAS;
}




// -----------------------------CAMBIO CORRECIÓN POR HISTÉRESIS------------------------------*/



// NTC

// Maneja los LED de rango para el termistor NTC
void mostrarNtc(void){

// De Amarillo a Verde
if ( (NTC_INF_INF < ntc_avg) && HAL_GPIO_ReadPin(ntcMedio_GPIO_Port, ntcMedio_Pin)==1 ) {
HAL_GPIO_WritePin( ntcBajo_GPIO_Port, ntcBajo_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin( ntcMedio_GPIO_Port, ntcMedio_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin( ntcAlto_GPIO_Port, ntcAlto_Pin, GPIO_PIN_RESET);
}

// De Amarillo a Rojo
else if ( (NTC_SUP_SUP > ntc_avg) && HAL_GPIO_ReadPin(ntcMedio_GPIO_Port, ntcMedio_Pin)==1 ) {
HAL_GPIO_WritePin( ntcBajo_GPIO_Port, ntcBajo_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin( ntcMedio_GPIO_Port, ntcMedio_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin( ntcAlto_GPIO_Port, ntcAlto_Pin, GPIO_PIN_SET);
}

// De Verde a Amarillo
else if ( (NTC_INF_SUP > ntc_avg) && HAL_GPIO_ReadPin(ntcBajo_GPIO_Port, ntcBajo_Pin)==1 ) {
HAL_GPIO_WritePin( ntcBajo_GPIO_Port, ntcBajo_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(ntcMedio_GPIO_Port, ntcMedio_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin( ntcAlto_GPIO_Port, ntcAlto_Pin, GPIO_PIN_RESET);
}

// De Rojo a Amarillo
else if ( (NTC_SUP_INF < ntc_avg) && HAL_GPIO_ReadPin(ntcAlto_GPIO_Port, ntcAlto_Pin)==1 ) {
HAL_GPIO_WritePin( ntcBajo_GPIO_Port, ntcBajo_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(ntcMedio_GPIO_Port, ntcMedio_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin( ntcAlto_GPIO_Port, ntcAlto_Pin, GPIO_PIN_RESET);
}


}




// POTE

// Maneja los LED de rango para el potenciómetro
void mostrarPot(void){

// De Amarillo a Verde
if ( (POT_INF_INF > pot_avg) && HAL_GPIO_ReadPin(potMedio_GPIO_Port, potMedio_Pin)==1 ) {
HAL_GPIO_WritePin( potBajo_GPIO_Port, potBajo_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(potMedio_GPIO_Port, potMedio_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin( potAlto_GPIO_Port, potAlto_Pin, GPIO_PIN_RESET);
}

// De Amarillo a Rojo
else if ( (POT_SUP_SUP < pot_avg) && HAL_GPIO_ReadPin(potMedio_GPIO_Port, potMedio_Pin)==1 ) {
HAL_GPIO_WritePin( potBajo_GPIO_Port, potBajo_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(potMedio_GPIO_Port, potMedio_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin( potAlto_GPIO_Port, potAlto_Pin, GPIO_PIN_SET);
}

// De Verde a Amarillo
else if ( (POT_INF_SUP < pot_avg) && HAL_GPIO_ReadPin(potBajo_GPIO_Port, potBajo_Pin)==1 ) {
HAL_GPIO_WritePin( potBajo_GPIO_Port, potBajo_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(potMedio_GPIO_Port, potMedio_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin( potAlto_GPIO_Port, potAlto_Pin, GPIO_PIN_RESET);
}

// De Rojo a Amarillo
else if ( (POT_SUP_INF < pot_avg) && HAL_GPIO_ReadPin(potAlto_GPIO_Port, potAlto_Pin)==1 ) {
HAL_GPIO_WritePin( potBajo_GPIO_Port, potBajo_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(potMedio_GPIO_Port, potMedio_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin( potAlto_GPIO_Port, potAlto_Pin, GPIO_PIN_RESET);
}


}
// Gestiona los LED indicadores de rangpo
void mostrar(void){
// Manejamos los LED indicadores de rango para cada variable
mostrarNtc(); // termisto NTC
mostrarPot(); // Potenciómetro
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
