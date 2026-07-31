#ifndef TIME_DOMAIN_H
#define TIME_DOMAIN_H

#include <stdint.h>
#include "main.h"
#include "ILI9341_STM32_Driver.h"
#include <stdio.h>                    // for sprintf
#include "ILI9341_STM32_Driver.h"     // for ILI9341 functions
#include "ILI9341_GFX.h"
extern TIM_HandleTypeDef htim3;       // add this

/* ── Externals defined in main.c ────────────────────────────────────────── */
extern SPI_HandleTypeDef  hspi1;
extern volatile uint8_t   spi_dma_done;
extern uint16_t           displayed_data1[];
extern uint16_t           displayed_data2[];

/* ── Public API ─────────────────────────────────────────────────────────── */
void     Draw_Axes(void);
void     Plot_Waveforms(void);
uint16_t Find_Trigger(uint16_t *buf, uint16_t len,
                      uint16_t threshold, uint16_t hysteresis,
                      uint16_t holdoff);
void Draw_Time_Labels(void);
#endif /* TIME_DOMAIN_H */