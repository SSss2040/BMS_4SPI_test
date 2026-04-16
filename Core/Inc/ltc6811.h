#ifndef LTC6811_H
#define LTC6811_H

#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include <stdint.h>


#define TOTAL_IC 1
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

#define CMD_WRCFGA 0x0001  // 写入配置寄存器组A
#define CMD_RDCFGA 0x0002  // 读取配置寄存器组A
#define CMD_RDCVA 0x0004
#define CMD_RDCVB 0x0006
#define CMD_RDCVC 0x0008
#define CMD_RDCVD 0x000A
#define CMD_RDSTATA 0x0010 // 读取状态寄存器组A
#define CMD_RDSTATB 0x0012 // 读取状态寄存器组B
#define CMD_CLRSTAT 0x0713 // 清除所有状态寄存器故障标志位
#define CMD_ADCV 0x0360    // 启动所有电池单体电压测量
#define CMD_ADAX 0x0460    // 启动所有辅助GPIO/AUX电压测量
#define CMD_RDAUXA 0x000C  // 读取辅助寄存器组A
#define CMD_RDAUXB 0x000E  // 读取辅助寄存器组B

void LTC6811_Init(void);
void LTC6811_write_config_all(uint8_t config[TOTAL_IC][6]);
void LTC6811_clear_status(void);
void LTC6811_start_conversion(void);
void LTC6811_start_aux_conversion(void);
int LTC6811_read_cells(uint16_t *cell);
int LTC6811_read_aux(uint16_t *aux);
uint16_t pec15_calc(uint8_t *data, int len);
uint8_t check_pec(uint8_t *data, int len);
void init_PEC15_Table(void);
void spi_txrx(uint8_t *tx, uint8_t *rx, uint16_t len);
void LTC6811_wakeup(void);
void LTC6811_read_status(void);
void LTC6811_send_cmd(uint16_t cmd);

#endif // LTC6811_H