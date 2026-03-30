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
#include "dma.h"
#include "spi.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ltc6811.h"
#include "bms.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t ov;
  uint8_t uv;
  uint8_t ot;
  uint8_t ut;
  uint8_t open;
  uint8_t comm;
} AMS_Fault_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TOTAL_IC 3
#define CELL_PER_IC 12
#define TOTAL_CELL (TOTAL_IC * CELL_PER_IC)
#define CS_LOW() HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)
#define OV_THRESHOLD 4.20f
#define UV_THRESHOLD 2.50f
#define SPIKE_THRESHOLD 0.5f
#define FAULT_COUNT_TH 3   // 连续3次触发
#define RECOVER_COUNT_TH 3 // 连续3次恢复
#define COMM_FAULT_TH 3

#define CMD_WRCFGA 0x001  // 写入配置寄存器组A
#define CMD_RDCFGA 0x002  // 读取配置寄存器组A
#define CMD_RDCVA 0x0004
#define CMD_RDCVB 0x0006
#define CMD_RDCVC 0x0008
#define CMD_RDCVD 0x000A
#define CMD_RDSTATA 0x010 // 读取状态寄存器组A
#define CMD_RDSTATB 0x012 // 读取状态寄存器组B
#define CMD_CLRSTAT 0x713 // 清除所有状态寄存器故障标志位
#define CMD_ADCV 0x04c0    // 启动所有电池单体电压测量
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t cell_raw[TOTAL_CELL]; // 存储所有电池的电压
CellFault_t fault[TOTAL_CELL];
AMS_Fault_t ams_fault;
float last_voltage[TOTAL_CELL] = {0};
uint8_t ov_cnt[TOTAL_CELL] = {0};
uint8_t uv_cnt[TOTAL_CELL] = {0};
uint8_t open_cnt[TOTAL_CELL] = {0};
uint8_t spike_cnt[TOTAL_CELL] = {0};
uint8_t comm_cnt = 0;
uint8_t uart_dma_busy = 0;
static uint16_t pec15Table[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LTC6811_Init(void);
void LTC6811_write_config(uint8_t *config);
int LTC6811_read_cells(uint16_t *cell);
uint16_t pec15_calc(uint8_t *data, int len);
void spi_txrx(uint8_t *tx, uint8_t *rx, uint16_t len);
void LTC6811_wakeup(void);
void LTC6811_cmd(uint8_t cmd0, uint8_t cmd1);
void LTC6811_read_reg(uint8_t cmd0, uint8_t cmd1, uint8_t *rx);
void uart_dma_transmit(const char *message);
//void check_voltage_fault(uint16_t *raw);
//void check_open_wire(uint16_t *raw);
//void check_spike(uint16_t *raw);
uint8_t check_pec(uint8_t *data, uint16_t len);
//void BMS_FaultDetect(uint16_t *cell_raw);
float raw_to_voltage(uint16_t raw);
void LTC6811_clear_status(void);
void LTC6811_read_status(void);
void LTC6811_start_conversion(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uart_dma_transmit(const char *message) {
  // 等待上一次传输完成
  uint32_t timeout = 1000; // 1秒超时
  while (uart_dma_busy && timeout--) {
    HAL_Delay(1);
  }

  uart_dma_busy = 1;
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)message, strlen(message));
}



uint8_t check_pec(uint8_t *data, uint16_t len) {
  uint16_t received = (data[len - 2] << 8) | data[len - 1];
  uint16_t calc = pec15_calc(data, len - 2);

  return (received == calc);
}

float raw_to_voltage(uint16_t raw) {
  // LTC6811电压测量分辨率：1mV/LSB
  return raw * 0.0001f;
}

// 初始化PEC15查找表，MCU上电时仅需调用一次
void init_PEC15_Table(void) {
  uint16_t remainder;
  uint16_t i;
  int bit;

  for (i = 0; i < 256; i++) {
    remainder = i << 7;
    for (bit = 8; bit > 0; bit--) {
      if (remainder & 0x4000) {
        remainder = (remainder << 1) ^ 0x4599;
      } else {
        remainder <<= 1;
      }
    }
    pec15Table[i] = remainder & 0x7FFF;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // 初始化PEC15查找表
  init_PEC15_Table();
  // 初始化LTC6811
  LTC6811_Init();

  // 初始化变量
  memset(cell_raw, 0, sizeof(cell_raw));
  memset(fault, 0, sizeof(fault));
  memset(last_voltage, 0, sizeof(last_voltage));
  memset(ov_cnt, 0, sizeof(ov_cnt));
  memset(uv_cnt, 0, sizeof(uv_cnt));
  memset(open_cnt, 0, sizeof(open_cnt));
  memset(spike_cnt, 0, sizeof(spike_cnt));
  comm_cnt = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    LTC6811_start_conversion();
    HAL_Delay(3);
    // 读取15节电池
    if (LTC6811_read_cells(cell_raw) == 0)
    {
      // 故障检测
      /*
      BMS_FaultDetect(cell_raw);

      // 打印故障信息
      for (int i = 0; i < 15; i++) {
        if (fault[i].ov) {
          char buffer[64];
          snprintf(buffer, sizeof(buffer), "Cell %d OV!\r\n", i + 1);
          uart_dma_transmit(buffer);
        }
        if (fault[i].uv) {
          char buffer[64];
          snprintf(buffer, sizeof(buffer), "Cell %d UV!\r\n", i + 1);
          uart_dma_transmit(buffer);
        }
        if (fault[i].open) {
          char buffer[64];
          snprintf(buffer, sizeof(buffer), "Cell %d OPEN!\r\n", i + 1);
          uart_dma_transmit(buffer);
        }
        if (fault[i].spike) {
          char buffer[64];
          snprintf(buffer, sizeof(buffer), "Cell %d SPIKE!\r\n", i + 1);
          uart_dma_transmit(buffer);
        }
      }*/

      // 打印电压
      char buffer[128];
      snprintf(buffer, sizeof(buffer), "Cell Voltages:\r\n");
      uart_dma_transmit(buffer);

      for (int i = 0; i < TOTAL_CELL; i++) {
        float voltage = cell_raw[i] * 0.0001f;
        snprintf(buffer, sizeof(buffer), "Cell %02d: %.4f V\r\n", i + 1, voltage);
        uart_dma_transmit(buffer);
      }

      snprintf(buffer, sizeof(buffer), "----------------------\r\n");
      uart_dma_transmit(buffer);
    } else {
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "LTC6811 Read Error!\r\n");
      uart_dma_transmit(buffer);
      HAL_Delay(10);
      comm_cnt++;
      if (comm_cnt >= COMM_FAULT_TH) {
        ams_fault.comm = 1;
        snprintf(buffer, sizeof(buffer), "Communication Fault!\r\n");
        uart_dma_transmit(buffer);
      }
    }

    HAL_Delay(100); // 1秒读取一次
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
