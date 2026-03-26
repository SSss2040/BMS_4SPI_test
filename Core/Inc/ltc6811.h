// ltc6811.h
#ifndef __LTC6811_H__
#define __LTC6811_H__

#include <stdint.h>

void LTC6811_wakeup(void);
void LTC6811_cmd(uint8_t cmd0, uint8_t cmd1);
void LTC6811_read_reg(uint8_t cmd0, uint8_t cmd1, uint8_t *rx);

#endif