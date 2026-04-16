/*
 * temperature.h
 *
 *  Created on: 2026-04-16
 *      Author: GitHub Copilot
 */

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void Temperature_Init(void);
int Temperature_ReadGPIO4(float *temperature_c);
float LTC6811_GPIO4_RawToVoltage(uint16_t raw);
float LTC6811_GPIO4_RawToTemperature(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif // TEMPERATURE_H
