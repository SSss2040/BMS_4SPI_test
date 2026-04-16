#include "ltc6811.h"
#include "spi.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdint.h>

static uint16_t pec15Table[256];

void LTC6811_Init(void) {
  // 初始化PEC15表
  init_PEC15_Table();

  // 1. 唤醒
  LTC6811_wakeup();
  HAL_Delay(5);

  // 2. 配置数组（3颗）
  uint8_t config[TOTAL_IC][6];

  for (int ic = 0; ic < TOTAL_IC; ic++) {
    uint16_t vuv =
        (uint16_t)((UV_THRESHOLD / 0.0016f) - 1); // UV_THRESHOLD = 2.50f V
    uint16_t vov =
        (uint16_t)((OV_THRESHOLD / 0.0016f)); // OV_THRESHOLD = 4.20f V

    config[ic][0] = 0x06;       // ADCOPT=1 (14kHz/3kHz模式), REFON=1, GPIO5-1=0
    config[ic][1] = vuv & 0xFF; // VUV[7:0]
    config[ic][2] =
        ((vuv >> 8) & 0x0F) | ((vov & 0x0F) << 4); // VUV[11:8] | VOV[3:0]
    config[ic][3] = (vov >> 4) & 0xFF;             // VOV[11:4]
    config[ic][4] = 0x00;                          // DCC8-1 = 0 (无放电)
    config[ic][5] = 0x00; // DCTO[3:0]=0 (放电超时禁用), DCC12-9=0
  }

  LTC6811_write_config_all(config);

  // 4. 清状态
  LTC6811_clear_status();

  // 5. 启动ADC（整链一起）
  LTC6811_start_conversion();
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
  LTC6811_send_cmd(CMD_ADCV);
  HAL_Delay(3);
}

int LTC6811_read_cells(uint16_t *cell) {


  uint8_t tx[4 + 8 * TOTAL_IC];
  uint8_t rx[4 + 8 * TOTAL_IC];

  const uint16_t cmds[4] = {
      CMD_RDCVA,
      CMD_RDCVB,
      CMD_RDCVC,
      CMD_RDCVD
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
      for (int i = 0; i < 3; i++) {
        int cell_index = (TOTAL_IC - 1 - ic) * 12 + // IC顺序翻转
                         grp * 3 + i;

        cell[cell_index] = (data[2 * i + 1] << 8) | data[2 * i];
      }
    }
  }

  return 0;
}

int LTC6811_read_aux(uint16_t *aux) {
  uint8_t tx[4 + 8 * TOTAL_IC];
  uint8_t rx[4 + 8 * TOTAL_IC];

  const uint16_t cmds[2] = {
      CMD_RDAUXA,
      CMD_RDAUXB
  };

  for (int grp = 0; grp < 2; grp++) {
    memset(tx, 0, sizeof(tx));

    tx[0] = cmds[grp] >> 8;
    tx[1] = cmds[grp] & 0xFF;
    uint16_t pec = pec15_calc(tx, 2);
    tx[2] = pec >> 8;
    tx[3] = pec & 0xFF;

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), 100);
    CS_HIGH();

    for (int ic = 0; ic < TOTAL_IC; ic++) {
      int rx_offset = 4 + ic * 8;
      uint8_t *data = &rx[rx_offset];

      uint16_t calc = pec15_calc(data, 6);
      uint16_t recv = (data[6] << 8) | data[7];
      if (calc != recv)
        return -1;

      for (int i = 0; i < 3; i++) {
        int aux_index = (TOTAL_IC - 1 - ic) * 6 + grp * 3 + i;
        aux[aux_index] = (data[2 * i + 1] << 8) | data[2 * i];
      }
    }
  }

  return 0;
}

void LTC6811_start_aux_conversion(void) {
  LTC6811_wakeup();
  LTC6811_send_cmd(CMD_ADAX);
  HAL_Delay(10);
}

uint16_t pec15_calc(uint8_t *data, int len) {
  uint16_t remainder = 0x0010;

  for (int i = 0; i < len; i++) {
    uint8_t address = ((remainder >> 7) ^ data[i]) & 0xFF;
    remainder = (remainder << 8) ^ pec15Table[address];
  }

  return remainder << 1;
}

void LTC6811_wakeup(void) {
  CS_LOW();
  HAL_Delay(2);
  CS_HIGH();
  HAL_Delay(2);
}

void LTC6811_read_status(void) {
  uint8_t tx[4 + 8 * TOTAL_IC] = {0};
  uint8_t rx[4 + 8 * TOTAL_IC];

  tx[0] = CMD_RDSTATA >> 8;
  tx[1] = CMD_RDSTATA & 0xFF;

  uint16_t pec = pec15_calc(tx, 2);
  tx[2] = pec >> 8;
  tx[3] = pec & 0xFF;

  CS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), 100);
  CS_HIGH();

  // 检查PEC并解析状态
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    if (!check_pec(&rx[4 + ic * 8], 8)) {
      // 解析状态寄存器（示例：检查过压/欠压标志）
      uint8_t *status = &rx[4 + ic * 8];
      // 根据规格书解析状态位
      // 这里可以添加具体的状态解析代码
    }
  }
}

uint8_t check_pec(uint8_t *data, int len) {
  uint16_t received = (data[len - 2] << 8) | data[len - 1];
  uint16_t calc = pec15_calc(data, len - 2);
  return (received == calc);
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
    pec15Table[i] = remainder & 0xFFFF;
  }
}