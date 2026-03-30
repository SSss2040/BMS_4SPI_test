#include "ltc6811.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>
#include <stdint.h>

static uint16_t pec15Table[256];

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

void LTC6811_start_conversion(void) {
  LTC6811_wakeup();
  LTC6811_cmd(CMD_ADCV >> 8, CMD_ADCV & 0xFF);
}

int LTC6811_read_cells(uint16_t *cell) {
  uint8_t cmd[4];    // 命令 + PEC
  uint8_t rx_buf[8]; // 6字节数据 + 2字节PEC（单IC单组）
  const uint16_t rdcv_cmds[4] = {CMD_RDCVA, CMD_RDCVB, CMD_RDCVC, CMD_RDCVD}; // 4组命令，分别读取12节电池的电压

  // 1. 唤醒
  //LTC6811_wakeup();
  //HAL_Delay(1);

  // 2. 启动全部Cell ADC转换
 // LTC6811_cmd(CMD_ADCV >> 8, CMD_ADCV & 0xFF);
  //HAL_Delay(5); // 等待转换完成（取决于滤波模式）

  // 3. 循环读取4个寄存器组，共12节电池
  for (int grp = 0; grp < 4; grp++) {
    // 构造命令帧
    cmd[0] = rdcv_cmds[grp] >> 8;
    cmd[1] = rdcv_cmds[grp] & 0xFF;
    uint16_t pec = pec15_calc(&cmd[0], 2);
    cmd[2] = pec >> 8;
    cmd[3] = pec & 0xFF;

    // SPI 传输
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_SPI_Receive(&hspi1, rx_buf, 8, 100);
    CS_HIGH();

    // PEC 校验（对前6字节数据计算PEC，与后2字节比较）
    uint16_t calc_pec = pec15_calc(rx_buf, 6);
    uint16_t recv_pec = (rx_buf[6] << 8) | rx_buf[7];
    if (calc_pec != recv_pec) {
      return -1; // PEC错误
    }

    // 解析3节电池电压（每节2字节，12位有效数据）
    for (int i = 0; i < 3; i++) {
      cell[grp * 3 + i] = (rx_buf[i * 2 + 1] << 8) | rx_buf[i * 2];
      // 电压值 = cell[x] * 0.0001 V (100μV/LSB)
    }
  }

  return 0;
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
  HAL_Delay(5);
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
  for (volatile int i = 0; i < 10; i++);
  HAL_SPI_Transmit(&hspi1, buf, 4, 100);
  for (volatile int i = 0; i < 10; i++);
  CS_HIGH();
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