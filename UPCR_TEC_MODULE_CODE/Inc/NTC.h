#ifndef _NTC_H
#define _NTC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f0xx_hal.h"

#define TABLE_SIZE 303

#define KP 10 // Proportional
#define KI 10 // Integral
#define KD 10 // Derivative

#define timeChange 10
typedef struct {
    const uint16_t x;
    const uint16_t y;
} temp_lookup_t;

uint32_t now, lastTime;
uint16_t lastInput;
int32_t error, lastErr, Iterm, dErr;

uint16_t temp_to_adc(uint16_t temp); //return equivalent ADC value to a given temperature
int32_t compute_temp_PID(uint16_t Setpoint, uint16_t Input); //compute PID given a setpoint (equivalent ADC value) and the input sensor

extern const temp_lookup_t temp_table[TABLE_SIZE]; //lookup table relating ADC readings to temperature. Generated with python script @./temperature lookup table generator/temptable.py

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0xx_IT_H */