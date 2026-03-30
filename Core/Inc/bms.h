#ifndef BMS_H
#define BMS_H

#include <stdint.h>

#define TOTAL_CELL 12
#define OV_THRESHOLD 4.20f
#define UV_THRESHOLD 2.50f
#define SPIKE_THRESHOLD 0.5f
#define FAULT_COUNT_TH 3

typedef struct {
  uint8_t ov;
  uint8_t uv;
  uint8_t open;
  uint8_t spike;
} CellFault_t;

void check_voltage_fault(uint16_t *raw, CellFault_t *fault);
void check_open_wire(uint16_t *raw, CellFault_t *fault);
void check_spike(uint16_t *raw, CellFault_t *fault);
void BMS_FaultDetect(uint16_t *cell_raw, CellFault_t *fault);
float raw_to_voltage(uint16_t raw);

#endif // BMS_H