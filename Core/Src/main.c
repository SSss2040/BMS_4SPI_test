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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ltc6811.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t ov;    // 过压
  uint8_t uv;    // 欠压
  uint8_t open;  // 开路
  uint8_t spike; // 突变
} CellFault_t;
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
#define TOTAL_IC 1
#define CELL_PER_IC 5
#define TOTAL_CELL 5
#define CS_LOW() HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)
#define OV_THRESHOLD 4.20f
#define UV_THRESHOLD 2.50f
#define SPIKE_THRESHOLD 0.5f
#define FAULT_COUNT_TH 3   // 连续3次触发
#define RECOVER_COUNT_TH 3 // 连续3次恢复
#define COMM_FAULT_TH 3
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LTC6811_Init(void);
void LTC6811_write_config(uint8_t *config);
int LTC6811_read_15cells(uint16_t *cell);
uint16_t pec15_calc(uint8_t len, uint8_t *data);
void spi_txrx(uint8_t *tx, uint8_t *rx, uint16_t len);
void LTC6811_wakeup(void);
void LTC6811_cmd(uint8_t cmd0, uint8_t cmd1);
void LTC6811_read_reg(uint8_t cmd0, uint8_t cmd1, uint8_t *rx);
void uart_dma_transmit(const char *message);
void check_voltage_fault(uint16_t *raw);
void check_open_wire(uint16_t *raw);
void check_spike(uint16_t *raw);
uint8_t check_pec(uint8_t *data, uint16_t len);
void BMS_FaultDetect(uint16_t *cell_raw);
float raw_to_voltage(uint16_t raw);
    /* USER CODE END PFP */

    /* Private user code
       ---------------------------------------------------------*/
    /* USER CODE BEGIN 0 */
    void LTC6811_Init(void) {
  // 1. 唤醒芯片
  LTC6811_wakeup();
  HAL_Delay(10);

  // 2. 配置CFGR0-CFGR5寄存器（根据实际需求配置）
  // 这里可以配置GPIO、ADC模式等
  uint8_t config[6] = {0};

  // 示例配置：启用所有GPIO，ADC模式为7kHz
  config[0] = 0x00; // GPIO设置
  config[1] = 0x00;
  config[2] = 0x00;
  config[3] = 0x00;
  config[4] = 0x00;
  config[5] = 0x00;

  // 写入配置寄存器
  LTC6811_write_config(config);

  // 3. 启动ADC转换
  LTC6811_cmd(0x03, 0x60); // 启动所有电池ADC转换
  HAL_Delay(10);
}

void LTC6811_write_config(uint8_t *config) {
  uint8_t tx[4 + 6 + 2]; // 命令 + 数据 + PEC
  uint8_t rx[4 + 6 + 2];

  // 写配置命令 (WRCFG)
  tx[0] = 0x00;
  tx[1] = 0x01;

  // 计算命令PEC
  uint16_t pec = pec15_calc(2, tx);
  tx[2] = pec >> 8;
  tx[3] = pec & 0xFF;

  // 复制配置数据
  memcpy(&tx[4], config, 6);

  // 计算数据PEC
  pec = pec15_calc(6, config);
  tx[10] = pec >> 8;
  tx[11] = pec & 0xFF;

  spi_txrx(tx, rx, sizeof(tx));
}

int LTC6811_read_15cells(uint16_t *cell) {
  uint8_t rxA[TOTAL_IC * 8];
  uint8_t rxB[TOTAL_IC * 8];

  // 唤醒
  LTC6811_wakeup();
  HAL_Delay(1);

  // 启动ADC转换
  LTC6811_cmd(0x03, 0x60);
  HAL_Delay(5); // 增加延时确保转换完成

  // 读取A/B组数据
  LTC6811_read_reg(0x00, 0x04, rxA); // RDCVA
  LTC6811_read_reg(0x00, 0x06, rxB); // RDCVB

  // 检查PEC错误
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    if (check_pec(&rxA[ic * 8], 8) || check_pec(&rxB[ic * 8], 8)) {
      return -1; // PEC错误
    }
  }

  int idx = 0;
  for (int ic = TOTAL_IC - 1; ic >= 0; ic--) {
    uint8_t *a = &rxA[ic * 8];
    uint8_t *b = &rxB[ic * 8];

    uint16_t v[6];

    // A组（C1~C3）
    v[0] = (a[1] << 8) | a[0];
    v[1] = (a[3] << 8) | a[2];
    v[2] = (a[5] << 8) | a[4];

    // B组（C4~C6）
    v[3] = (b[1] << 8) | b[0];
    v[4] = (b[3] << 8) | b[2];
    v[5] = (b[5] << 8) | b[4];

    // 只取前5节
    for (int i = 0; i < 5; i++) {
      if (idx < TOTAL_CELL)
        cell[idx++] = v[i];
    }
  }

  return 0; // 成功
}

uint16_t pec15_calc(uint8_t len, uint8_t *data) {
  uint16_t remainder = 16;
  uint16_t polynomial = 0x4599;

  for (uint8_t i = 0; i < len; i++) {
    remainder ^= (data[i] << 7);
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (remainder & 0x4000)
        remainder = (remainder << 1) ^ polynomial;
      else
        remainder = (remainder << 1);
    }
  }
  return remainder << 1;
}

void spi_txrx(uint8_t *tx, uint8_t *rx, uint16_t len) {
  CS_LOW();
  for (volatile int i = 0; i < 10; i++)
    ;
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 100);
  CS_HIGH();
  for (volatile int i = 0; i < 10; i++)
    ;
}

void LTC6811_wakeup(void) {
  CS_LOW();
  HAL_Delay(1);
  CS_HIGH();
}

void LTC6811_cmd(uint8_t cmd0, uint8_t cmd1) {
  uint8_t buf[4];

  buf[0] = cmd0;
  buf[1] = cmd1;

  uint16_t pec = pec15_calc(2, buf);
  buf[2] = pec >> 8;
  buf[3] = pec & 0xFF;

  CS_LOW();
  HAL_SPI_Transmit(&hspi1, buf, 4, 100);
  CS_HIGH();
}

void LTC6811_read_reg(uint8_t cmd0, uint8_t cmd1, uint8_t *rx) {
  uint8_t tx[4 + TOTAL_IC * 8];
  uint8_t rx_buf[4 + TOTAL_IC * 8];

  // 命令
  tx[0] = cmd0;
  tx[1] = cmd1;

  uint16_t pec = pec15_calc(2, tx);
  tx[2] = pec >> 8;
  tx[3] = pec & 0xFF;

  // 填充dummy
  for (int i = 4; i < sizeof(tx); i++)
    tx[i] = 0xFF;

  spi_txrx(tx, rx_buf, sizeof(tx));

  // 拷贝有效数据（去掉前4字节）
  for (int i = 0; i < TOTAL_IC * 8; i++)
    rx[i] = rx_buf[i + 4];
}

void uart_dma_transmit(const char *message) {
  // 等待上一次传输完成
  uint32_t timeout = 1000; // 1秒超时
  while (uart_dma_busy && timeout--) {
    HAL_Delay(1);
  }

  uart_dma_busy = 1;
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)message, strlen(message));
}

void check_voltage_fault(uint16_t *raw) {
  for (int i = 0; i < TOTAL_CELL; i++) {
    float v = raw_to_voltage(raw[i]);

    // 过压检测
    if (v > OV_THRESHOLD) {
      if (ov_cnt[i] < FAULT_COUNT_TH)
        ov_cnt[i]++;
      if (ov_cnt[i] >= FAULT_COUNT_TH)
        fault[i].ov = 1;
    } else {
      if (ov_cnt[i] > 0)
        ov_cnt[i]--;
      if (ov_cnt[i] == 0)
        fault[i].ov = 0;
    }

    // 欠压检测
    if (v < UV_THRESHOLD) {
      if (uv_cnt[i] < FAULT_COUNT_TH)
        uv_cnt[i]++;
      if (uv_cnt[i] >= FAULT_COUNT_TH)
        fault[i].uv = 1;
    } else {
      if (uv_cnt[i] > 0)
        uv_cnt[i]--;
      if (uv_cnt[i] == 0)
        fault[i].uv = 0;
    }
  }
}

void check_open_wire(uint16_t *raw) {
  for (int i = 0; i < 15; i++) {
    if (raw[i] < 1000) // <0.1V
    {
      if (open_cnt[i] < FAULT_COUNT_TH)
        open_cnt[i]++;

      if (open_cnt[i] >= FAULT_COUNT_TH)
        fault[i].open = 1;
    } else {
      if (open_cnt[i] > 0)
        open_cnt[i]--;

      if (open_cnt[i] == 0)
        fault[i].open = 0;
    }
  }
}

void check_spike(uint16_t *raw) {
  for (int i = 0; i < 15; i++) {
    float v = raw[i] * 0.0001f;

    if (fabsf(v - last_voltage[i]) > SPIKE_THRESHOLD) {
      if (spike_cnt[i] < FAULT_COUNT_TH)
        spike_cnt[i]++;

      if (spike_cnt[i] >= FAULT_COUNT_TH)
        fault[i].spike = 1;
    } else {
      if (spike_cnt[i] > 0)
        spike_cnt[i]--;

      if (spike_cnt[i] == 0)
        fault[i].spike = 0;
    }

    last_voltage[i] = v;
  }
}

uint8_t check_pec(uint8_t *data, uint16_t len) {
  uint16_t received = (data[len - 2] << 8) | data[len - 1];
  uint16_t calc = pec15_calc(len - 2, data);

  return (received != calc);
}

void BMS_FaultDetect(uint16_t *cell_raw) {
  check_voltage_fault(cell_raw);
  check_open_wire(cell_raw);
  check_spike(cell_raw);
}

float raw_to_voltage(uint16_t raw) {
  return raw * 0.0001f; // LTC6811的转换因子
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

  printf("BMS System Started\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 读取15节电池
    if (LTC6811_read_15cells(cell_raw) == 0) // 假设函数返回0表示成功
    {
      // 故障检测
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
      }

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
      comm_cnt++;
      if (comm_cnt >= COMM_FAULT_TH) {
        ams_fault.comm = 1;
        snprintf(buffer, sizeof(buffer), "Communication Fault!\r\n");
        uart_dma_transmit(buffer);
      }
    }

    HAL_Delay(1000); // 1秒读取一次
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
