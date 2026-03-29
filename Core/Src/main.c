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

#define CMD_WRCFGA 0x0001  // 写入配置寄存器组A
#define CMD_RDCFGA 0x0002  // 读取配置寄存器组A
#define CMD_RDCV 0x0003    // 读取所有12节电池的电压测量值
#define CMD_RDSTATA 0x0008 // 读取状态寄存器组A
#define CMD_RDSTATB 0x0009 // 读取状态寄存器组B
#define CMD_CLRSTAT 0x000A // 清除所有状态寄存器故障标志位
#define CMD_ADCV 0x0260    // 启动所有电池单体电压测量
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
int LTC6811_read_cells(uint16_t *cell);
uint16_t pec15_calc(uint8_t *data, int len);
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
static uint16_t pec15Table[256];
void LTC6811_clear_status(void);
void LTC6811_read_status(void);

/* USER CODE END PFP */

/* Private user code           ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LTC6811_Init(void) {
  // 1. 唤醒芯片
  LTC6811_wakeup();
  HAL_Delay(10);

  // 2. 配置CFGR0-CFGR5寄存器
  uint8_t config[6] = {0};

  // CFGR0: GPIO配置、基准源控制、ADC模式
  config[0] = 0x00; // GPIO1-5输入，REFON=0，SWTRD=0，ADCOPT=0（标准模式）

  // CFGR1: 欠压阈值低8位（示例：2.5V = 2500mV）
  uint16_t vuv = 2500; // 欠压阈值2.5V
  config[1] = vuv & 0xFF;

  // CFGR2: 欠压阈值高4位 + 过压阈值低4位
  uint16_t vov = 4200; // 过压阈值4.2V
  config[2] = ((vuv >> 8) & 0x0F) | ((vov & 0x0F) << 4);

  // CFGR3: 过压阈值高8位
  config[3] = (vov >> 4) & 0xFF;

  // CFGR4: 放电定时器 + DCC高4位
  config[4] = 0x00; // 放电定时器0分钟，DCC高4位0

  // CFGR5: DCC低8位（所有电池放电关闭）
  config[5] = 0x00;

  // 写入配置寄存器
  LTC6811_write_config(config);

  // 3. 清除状态寄存器
  LTC6811_clear_status();

  // 4. 启动ADC转换
  LTC6811_cmd(CMD_ADCV >> 8, CMD_ADCV & 0xFF);
  HAL_Delay(10);
}

void LTC6811_write_config(uint8_t *config) {
  uint8_t tx[4 + 6 + 2]; // 命令 + 数据 + PEC
  uint8_t rx[4 + 6 + 2];

  // 写配置命令 (WRCFGA)
  tx[0] = CMD_WRCFGA >> 8;
  tx[1] = CMD_WRCFGA & 0xFF;

  // 计算命令PEC - 修正参数顺序
  uint16_t pec = pec15_calc(tx, 2);
  tx[2] = pec >> 8;
  tx[3] = pec & 0xFF;

  // 复制配置数据
  memcpy(&tx[4], config, 6);

  // 计算数据PEC - 修正参数顺序
  pec = pec15_calc(config, 6);
  tx[10] = pec >> 8;
  tx[11] = pec & 0xFF;

  spi_txrx(tx, rx, sizeof(tx));
}

void LTC6811_clear_status(void) {
  uint8_t tx[4];
  uint8_t rx[4];

  // 清除状态命令 (CLRSTAT)
  tx[0] = CMD_CLRSTAT >> 8;
  tx[1] = CMD_CLRSTAT & 0xFF;

  // 计算PEC - 修正参数顺序
  uint16_t pec = pec15_calc(tx, 2); // 修正这里
  tx[2] = pec >> 8;
  tx[3] = pec & 0xFF;

  spi_txrx(tx, rx, sizeof(tx));
}

int LTC6811_read_cells(uint16_t *cell) {
  uint8_t rx_buf[TOTAL_IC * 8 + 4]; // 数据 + PEC

  // 1. 唤醒芯片
  LTC6811_wakeup();
  HAL_Delay(1);

  // 2. 启动ADC转换（根据规格书使用0x0260）
  LTC6811_cmd(0x02, 0x60);
  HAL_Delay(5); // 等待转换完成

  // 3. 读取所有电池电压（RDCV命令）
  uint8_t tx[4];
  tx[0] = CMD_RDCV >> 8;
  tx[1] = CMD_RDCV & 0xFF;

  // 计算命令PEC
  uint16_t pec = pec15_calc(tx, 2);
  tx[2] = pec >> 8;
  tx[3] = pec & 0xFF;

  // 发送命令并接收数据
  CS_LOW();
  HAL_SPI_Transmit(&hspi1, tx, 4, 100);
  HAL_SPI_Receive(&hspi1, rx_buf, 8, 100); // 读取8字节数据
  CS_HIGH();

  // 4. 检查PEC错误
  if (check_pec(rx_buf, 8)) {
    return -1; // PEC错误
  }

  // 5. 解析数据（每个芯片返回12节电池电压，我们只取前5节）
  for (int i = 0; i < TOTAL_CELL; i++) {
    cell[i] = (rx_buf[i * 2 + 1] << 8) | rx_buf[i * 2];
  }

  return 0; // 成功
}

uint16_t pec15_calc(uint8_t *data, int len) {
  uint16_t remainder = 0x0010; // 初始值
  int i;

  for (i = 0; i < len; i++) {
    uint8_t address = ((remainder >> 7) ^ data[i]) & 0xFF;
    remainder = (remainder << 8) ^ pec15Table[address];
  }
  return remainder & 0x7FFF;
}

void spi_txrx(uint8_t *tx, uint8_t *rx, uint16_t len) {
  CS_LOW();

  // 添加小延时确保CS稳定
  for (volatile int i = 0; i < 10; i++)
    ;

  // SPI模式3：CPOL=1, CPHA=1
  // 时钟空闲为高，数据在下降沿输出，上升沿锁存
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 1000);

  CS_HIGH();

  // 添加小延时
  for (volatile int i = 0; i < 10; i++)
    ;
}

void LTC6811_wakeup(void) {
  CS_LOW();
  HAL_Delay(2);
  CS_HIGH();
  HAL_Delay(1);
}

void LTC6811_cmd(uint8_t cmd0, uint8_t cmd1){
  uint8_t buf[4];

  buf[0] = cmd0;
  buf[1] = cmd1;

  // 修正：第一个参数应该是 buf 指针，不是长度
  uint16_t pec = pec15_calc(buf, 2); // 修正这里
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

  // 修正：第一个参数应该是 tx 指针，不是长度
  uint16_t pec = pec15_calc(tx, 2); // 修正这里
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

void LTC6811_read_status(void) {
  uint8_t tx[4];
  uint8_t rx_buf[TOTAL_IC * 8 + 4];

  // 读取状态寄存器组A (RDSTATA)
  tx[0] = CMD_RDSTATA >> 8;
  tx[1] = CMD_RDSTATA & 0xFF;

  // 计算PEC
  uint16_t pec = pec15_calc(tx, 2);
  tx[2] = pec >> 8;
  tx[3] = pec & 0xFF;

  // 发送命令并接收数据
  CS_LOW();
  HAL_SPI_Transmit(&hspi1, tx, 4, 100);

  for (int i = 0; i < TOTAL_IC; i++) {
    HAL_SPI_Receive(&hspi1, &rx_buf[i * 8], 8, 100);
  }
  CS_HIGH();

  // 检查PEC并解析状态
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    if (!check_pec(&rx_buf[ic * 8], 8)) {
      // 解析状态寄存器（示例：检查过压/欠压标志）
      uint8_t *status = &rx_buf[ic * 8];
      // 根据规格书解析状态位
      // 这里可以添加具体的状态解析代码
    }
  }
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
  uint16_t calc = pec15_calc(data, len - 2);

  return (received != calc);
}

void BMS_FaultDetect(uint16_t *cell_raw) {
  check_voltage_fault(cell_raw);
  check_open_wire(cell_raw);
  check_spike(cell_raw);
}

float raw_to_voltage(uint16_t raw) {
  // LTC6811电压测量分辨率：1mV/LSB
  return raw * 0.001f;
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
  while (1)
  {
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
        float voltage = cell_raw[i] * 0.001f;
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
