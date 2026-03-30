#include "ltc6811.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>
#include <stdint.h>

static uint16_t pec15Table[256];

void LTC6811_Init(void) {
  // 1. 唤醒
  LTC6811_wakeup();
  HAL_Delay(5);

  // 2. 配置数组（3颗）
  uint8_t config[TOTAL_IC][6];

  for (int ic = 0; ic < TOTAL_IC; ic++) {
    uint16_t vuv = 2500;
    uint16_t vov = 4200;

    config[ic][0] = 0x00;
    config[ic][1] = vuv & 0xFF;
    config[ic][2] = ((vuv >> 8) & 0x0F) | ((vov & 0x0F) << 4);
    config[ic][3] = (vov >> 4) & 0xFF;
    config[ic][4] = 0x00;
    config[ic][5] = 0x00;
  }

  // 3. 写配置（一次性写3颗）
  LTC6811_write_config_all(config);

  // 4. 清状态
  LTC6811_clear_status();

  // 5. 启动ADC（整链一起）
  LTC6811_start_conversion();
  HAL_Delay(3);
}

void LTC6811_write_config_all(uint8_t config[TOTAL_IC][6]) {
  uint8_t tx[4 + TOTAL_IC * 8];
  uint8_t rx[sizeof(tx)];

  // 命令
  tx[0] = 0x00;
  tx[1] = 0x01;

  uint16_t pec = pec15_calc(tx, 2);
  tx[2] = pec >> 8;
  tx[3] = pec & 0xFF;

  // 每个IC配置
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    int offset = 4 + ic * 8;

    memcpy(&tx[offset], config[ic], 6);

    pec = pec15_calc(config[ic], 6);
    tx[offset + 6] = pec >> 8;
    tx[offset + 7] = pec & 0xFF;
  }

  CS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), 100);
  CS_HIGH();
}

void LTC6811_clear_status(void) {
  LTC6811_send_cmd(CMD_CLRSTAT);
}

void LTC6811_send_cmd(uint16_t cmd) {
  uint8_t tx[4];

  tx[0] = cmd >> 8;
  tx[1] = cmd & 0xFF;

  uint16_t pec = pec15_calc(tx, 2);
  tx[2] = pec >> 8;
  tx[3] = pec & 0xFF;

  CS_LOW();
  HAL_SPI_Transmit(&hspi1, tx, 4, 100);
  CS_HIGH();
}

void LTC6811_start_conversion(void) {
  LTC6811_wakeup();
  LTC6811_cmd(CMD_ADCV >> 8, CMD_ADCV & 0xFF);
}

int LTC6811_read_cells(uint16_t *cell) {
  uint8_t tx[4 + 8 * TOTAL_IC];
  uint8_t rx[4 + 8 * TOTAL_IC];

  const uint16_t cmds[4] = {
      0x0004, // RDCVA
      0x0006, // RDCVB
      0x0008, // RDCVC
      0x000A  // RDCVD
  };

  for (int grp = 0; grp < 4; grp++) {
    memset(tx, 0, sizeof(tx));

    // 1. 命令
    tx[0] = cmds[grp] >> 8;
    tx[1] = cmds[grp] & 0xFF;

    uint16_t pec = pec15_calc(tx, 2);
    tx[2] = pec >> 8;
    tx[3] = pec & 0xFF;

    // 2. 后面全部是dummy（自动0）

    // 3. SPI通信
    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), 100);
    CS_HIGH();

    // 4. 解析每个IC
    for (int ic = 0; ic < TOTAL_IC; ic++) {
      // ⚠️ 注意顺序：远端IC在前
      int rx_offset = 4 + ic * 8;

      uint8_t *data = &rx[rx_offset];

      // PEC校验
      uint16_t calc = pec15_calc(data, 6);
      uint16_t recv = (data[6] << 8) | data[7];
      if (calc != recv)
        return -1;

      // 存入cell数组
      for (int i = 0; i < TOTAL_IC; i++) {
        int cell_index = (TOTAL_IC - 1 - ic) * 12 + // IC顺序翻转
                         grp * 3 + i;

        cell[cell_index] = (data[2 * i + 1] << 8) | data[2 * i];
      }
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