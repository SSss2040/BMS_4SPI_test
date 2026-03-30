#include "bms.h"
#include "ltc6811.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static float last_voltage[TOTAL_CELL] = {0};
static uint8_t ov_cnt[TOTAL_CELL] = {0};
static uint8_t uv_cnt[TOTAL_CELL] = {0};
static uint8_t open_cnt[TOTAL_CELL] = {0};
static uint8_t spike_cnt[TOTAL_CELL] = {0};

void check_voltage_fault(uint16_t *raw, CellFault_t *fault) {
  for (int i = 0; i < TOTAL_CELL; i++) {
    float v = raw_to_voltage(raw[i]);

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

void check_open_wire(uint16_t *raw, CellFault_t *fault) {
  for (int i = 0; i < TOTAL_CELL; i++) {
    if (raw[i] < 1000) {
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

void check_spike(uint16_t *raw, CellFault_t *fault) {
  for (int i = 0; i < TOTAL_CELL; i++) {
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

void BMS_FaultDetect(uint16_t *cell_raw, CellFault_t *fault) {
  check_voltage_fault(cell_raw, fault);
  check_open_wire(cell_raw, fault);
  check_spike(cell_raw, fault);
}

float raw_to_voltage(uint16_t raw) {
  return raw * 0.0001f;
}