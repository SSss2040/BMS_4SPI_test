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

#define CMD_WRCFGA 0x001  // 写入配置寄存器组A
#define CMD_WRCFGB 0x024  // 写入配置寄存器组B*
#define CMD_RDCFGA 0x002  // 读取配置寄存器组A
#define CMD_RDCFGB 0x026  // 读取配置寄存器组B*
#define CMD_RDCVA 0x004   // 读取电池电压寄存器组A
#define CMD_RDCVB 0x006   // 读取电池电压寄存器组B
#define CMD_RDCVC 0x008   // 读取电池电压寄存器组C
#define CMD_RDCVD 0x00A   // 读取电池电压寄存器组D
#define CMD_RDCVE 0x009   // 读取电池电压寄存器组E*
#define CMD_RDCVF 0x00B   // 读取电池电压寄存器组F*
#define CMD_RDAUXA 0x00C  // 读取辅助寄存器组A
#define CMD_RDAUXB 0x00E  // 读取辅助寄存器组B
#define CMD_RDAUXC 0x00D  // 读取辅助寄存器组C*
#define CMD_RDAUXD 0x00F  // 读取辅助寄存器组D*
#define CMD_RDSTATA 0x010 // 读取状态寄存器组A
#define CMD_RDSTATB 0x012 // 读取状态寄存器组B
#define CMD_WRSCTRL 0x014 // 写入S控制寄存器组
#define CMD_WRPWM 0x020   // 写入PWM寄存器组
#define CMD_WRPSB 0x01D   // 写入PWM/S控制寄存器组B*
#define CMD_RDSCTRL 0x016 // 读取S控制寄存器组
#define CMD_RDPWM 0x022   // 读取PWM寄存器组
#define CMD_RDPSB 0x01F   // 读取PWM/S控制寄存器组B*
#define CMD_STSCTRL 0x019 // 启动S控制脉冲并轮询状态
#define CMD_CLRSCTRL 0x018 // 清除S控制寄存器组
#define CMD_ADCV 0x04C0   // 启动电池电压ADC转换并轮询状态 (默认: 所有电池, 27kHz, 无放电)
#define CMD_ADOW 0x0528   // 启动开路ADC转换并轮询状态
#define CMD_CVST 0x0507   // 启动自测试电池电压转换并轮询状态
#define CMD_ADOL 0x0401   // 启动重叠测量电池7电压
#define CMD_ADAX 0x0460   // 启动GPIO ADC转换并轮询状态 (默认: GPIO1-5, 2nd Ref, 27kHz)
#define CMD_ADAXD 0x0400  // 启动GPIO ADC转换与数字冗余并轮询状态
#define CMD_AXST 0x0407   // 启动自测试GPIO转换并轮询状态
#define CMD_ADSTAT 0x0468 // 启动状态组ADC转换并轮询状态
#define CMD_ADSTATD 0x0408 // 启动状态组ADC转换与数字冗余并轮询状态
#define CMD_STATST 0x040F // 启动自测试状态组转换并轮询状态
#define CMD_ADCVAX 0x04F0 // 启动组合电池电压和GPIO1, GPIO2转换并轮询状态
#define CMD_ADCVSC 0x04E8 // 启动组合电池电压和SC转换并轮询状态
#define CMD_CLRCELL 0x711 // 清除电池电压寄存器组
#define CMD_CLRAUX 0x712  // 清除辅助寄存器组
#define CMD_CLRSTAT 0x713 // 清除状态寄存器组
#define CMD_PLADC 0x714   // 轮询ADC转换状态
#define CMD_DIAGN 0x715   // 诊断MUX并轮询状态
#define CMD_WRCOMM 0x721  // 写入COMM寄存器组
#define CMD_RDCOMM 0x722  // 读取COMM寄存器组
#define CMD_STCOMM 0x723  // 启动I2C/SPI通信

void LTC6811_Init(void);
void LTC6811_write_config_all(uint8_t config[TOTAL_IC][6]);
void LTC6811_clear_status(void);
void LTC6811_start_conversion(void);
int LTC6811_read_cells(uint16_t *cell);
uint16_t pec15_calc(uint8_t *data, int len);
uint8_t check_pec(uint8_t *data, int len);
void init_PEC15_Table(void);
void spi_txrx(uint8_t *tx, uint8_t *rx, uint16_t len);
void LTC6811_wakeup(void);
void LTC6811_read_status(void);
void LTC6811_send_cmd(uint16_t cmd);

#endif // LTC6811_H