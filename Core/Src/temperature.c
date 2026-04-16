#include "temperature.h"
#include "ltc6811.h"
#include <math.h>

#define NTC_PULLUP_R         10000.0f
#define NTC_REF_VOLTAGE      4.096f
#define NTC_BETA             3950.0f
#define NTC_REF_TEMP_K       298.15f
#define GPIO4_AUX_INDEX      3

void Temperature_Init(void) {
  // VREF2 已经在 LTC6811 初始化中被使能。
  // 如果需要单独启用辅助通道转换，可在读取前调用 Temperature_ReadGPIO4
}

int Temperature_ReadGPIO4(float *temperature_c) {
  uint16_t aux[6] = {0};

  LTC6811_start_aux_conversion();
  int aux_result = LTC6811_read_aux(aux);
  if (aux_result != 0) {
    // 添加调试输出：打印 aux_result 和可能的 PEC 信息
    // 注意：这里需要 uart_dma_transmit 函数，但 temperature.c 可能没有包含
    // 临时返回错误码，建议在 main.c 中处理
    return aux_result; // 返回 -1 表示 PEC 失败
  }

  uint16_t raw = aux[GPIO4_AUX_INDEX];
  if (raw == 0) {
    return -2;
  }

  *temperature_c = LTC6811_GPIO4_RawToTemperature(raw);

  if (!isfinite(*temperature_c)) {
    return -3;
  }

  return 0;
}

float LTC6811_GPIO4_RawToVoltage(uint16_t raw) {
  return raw * 0.0001f;
}

float LTC6811_GPIO4_RawToTemperature(uint16_t raw) {
  float voltage = LTC6811_GPIO4_RawToVoltage(raw);
  if (voltage <= 0.001f || voltage >= NTC_REF_VOLTAGE - 0.001f) {
    return NAN;
  }

  float resistance = NTC_PULLUP_R * voltage / (NTC_REF_VOLTAGE - voltage);
  float ln_ratio = logf(resistance / NTC_PULLUP_R);
  float temp_k = (NTC_BETA * NTC_REF_TEMP_K) / (NTC_BETA + NTC_REF_TEMP_K * ln_ratio);

  return temp_k - 273.15f;
}
